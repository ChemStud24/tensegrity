import numpy as np
import torch

from gnn_simulator.model_predictive_control.astar_planner import TensegrityAStarPlanner
from gnn_simulator.model_predictive_control.tensegrity_mppi_planner import TensegrityMPPIPlanner
# from gnn_simulator.model_predictive_control.mjc_mppi_planner import MjcMPPIPlanner
from gnn_simulator.utilities import torch_quaternion


class HybridAStarMPPIPlanner(torch.nn.Module):

    def __init__(self,
                 obstacles,
                 boundary,
                 goal,
                 grid_step,
                 tol,
                 astar_params,
                 mppi_params,
                 logger,
                 mppi_idle_time=1e10,
                 mppi_idle_dist=2.,
                 planner_type='mppi_turn_prims'):
        super().__init__()

        assert planner_type in ['mppi_only', 'astar_only', 'mppi_astar', 'mppi_turn_prims']
        self.planner_type = planner_type

        self.mppi_idle_time = mppi_idle_time
        self.mppi_idle_dist = mppi_idle_dist
        self.idle_timestamp = 0
        self.idle_com = None
        self.prev_primitive = (None, None, None)
        self.prev_mode = None

        self.logger = logger

        self.astar_planner = TensegrityAStarPlanner(
            obstacles=obstacles,
            boundary=boundary,
            grid_step=grid_step,
            goal=goal,
            tol=tol,
            **astar_params
        )
        self.mppi_controller = TensegrityMPPIPlanner(
        # self.mppi_controller = MjcMPPIPlanner(
            obstacles=obstacles,
            boundary=boundary,
            grid_step=grid_step,
            goal=goal,
            tol=tol,
            logger=logger,
            **mppi_params
        )

    @staticmethod
    def end_pts_to_pose(end_pts):
        end_pts = end_pts.reshape(-1, 3, 1)
        pos = (end_pts[::2] + end_pts[1::2]) / 2
        prin = end_pts[1::2] - end_pts[::2]
        prin /= np.linalg.norm(prin, axis=1, keepdims=True)
        quat = torch_quaternion.compute_quat_btwn_z_and_vec(torch.from_numpy(prin)).numpy()

        pose = np.hstack((pos, quat)).reshape(-1, 7)
        return pose

    def box_obstacle_costs(self, curr_state):
        return self.mppi_controller.box_obstacle_costs(curr_state)

    def dist_costs(self, curr_state):
        return self.astar_planner.dist_costs(curr_state)

    def set_goals(self, goal):
        self.astar_planner.set_goals(goal)
        self.mppi_controller.set_goals(goal)

    def set_pose_by_endpts(self, end_pts):
        self.astar_planner.set_pose_by_endpts(end_pts)
        self.mppi_controller.set_pose_by_endpts(end_pts)

    def reset_sim_pose(self, curr_pose, rest_lengths, motor_speeds, batch_size=1):
        self.mppi_controller.reset_sim_pose(curr_pose, rest_lengths, motor_speeds, batch_size)

    def reset_sim_state(self, curr_state, motor_speeds, rest_lengths, batch_size=1):
        self.mppi_controller.reset_sim_state(curr_state, motor_speeds, rest_lengths, batch_size)

    def plan(self, prev_n_pose_time_tups, rest_lens, motor_speeds):
        curr_pose, curr_timestamp = prev_n_pose_time_tups[-1]
        com = curr_pose.reshape(-1, 7)[:, :2].mean(axis=0, keepdims=True)
        curr_dir = self.mppi_controller.get_curr_dir(curr_pose)
        rev_dir = -self.mppi_controller.get_curr_dir(curr_pose)
        goal_dir = self.mppi_controller.get_goal_dir(curr_pose)
        curr_angle = self.mppi_controller.rel_2d_angle(curr_dir, goal_dir).cpu().item()
        rev_angle = self.mppi_controller.rel_2d_angle(rev_dir, goal_dir).cpu().item()
        heading = self.mppi_controller.rel_2d_angle(curr_dir.cpu().clone(), torch.tensor([[0.0, 1.0]])).cpu().item()
        print("COM:", com.flatten(), "Angle:", np.rad2deg(curr_angle), "Reverse Angle:", np.rad2deg(rev_angle),
              "Heading:", np.rad2deg(heading))
        print("Curr Dir:", curr_dir.flatten().cpu().numpy(), "Goal Dir:", goal_dir.flatten().cpu().numpy())

        if self.planner_type == 'mppi_only':
            return self._plan_mppi_only(prev_n_pose_time_tups, rest_lens, motor_speeds)
        elif self.planner_type == 'astar_only':
            return self._plan_astar_only(prev_n_pose_time_tups, rest_lens, motor_speeds)
        elif self.planner_type == 'mppi_astar':
            return self._plan_mppi_astar(prev_n_pose_time_tups, rest_lens, motor_speeds, com)
        else:
            return self._plan_mppi_turn_prims(
                prev_n_pose_time_tups, rest_lens, motor_speeds, com, curr_angle, rev_angle)

    def _plan_mppi_astar(self, prev_n_pose_time_tups, rest_lens, motor_speeds, com):
        curr_pose, curr_timestamp = prev_n_pose_time_tups[-1]

        if self.prev_mode == 'astar':
            self.idle_timestamp = curr_timestamp
            self.idle_com = com

        if self.idle_com is None:
            self.idle_com = com.copy()
            self.idle_timestamp = curr_timestamp

        idle_time = curr_timestamp - self.idle_timestamp
        d = np.linalg.norm(com - self.idle_com, axis=1).item()

        if d >= self.mppi_idle_dist or idle_time > self.mppi_idle_time:
            self.idle_timestamp = curr_timestamp
            self.idle_com = com

        if (d >= self.mppi_idle_dist or idle_time <= self.mppi_idle_time) and self.prev_primitive[0] != 'ccw':
            actions, states, batch_states = self.mppi_controller.plan(
                prev_n_pose_time_tups, rest_lens, motor_speeds)
            actions = actions.cpu().clone().numpy()
            self.prev_mode = 'mppi'

            return 'mppi', actions, (states, batch_states)
        else:
            self.logger.info(f"MPPI stuck, running A* planner.")
            gait, path = self.astar_planner.plan(
                prev_n_pose_time_tups, rest_lens, motor_speeds)
            self.prev_primitive = gait

            self.logger.info(f'Run primitive {gait}.')

            self.idle_timestamp = curr_timestamp
            self.idle_com = com
            self.prev_mode = 'astar'

            return 'astar', gait, path

    def _plan_mppi_turn_prims(self, prev_n_pose_time_tups, rest_lens, motor_speeds, com, curr_angle, rev_angle):
        goal = np.array(self.astar_planner.goal[:2]).reshape(1, 2)
        mppi_bandwidth = np.pi / 2
        if (np.linalg.norm(goal - com) < 1.0 
            or -mppi_bandwidth / 2 <= curr_angle <= mppi_bandwidth / 2 
            or -mppi_bandwidth / 2 <= rev_angle <= mppi_bandwidth / 2):
            actions, states, batch_states = self.mppi_controller.plan(
                prev_n_pose_time_tups, rest_lens, motor_speeds)
            actions = actions.cpu().clone().numpy()
            self.prev_mode = 'mppi'

            return 'mppi', actions, (states, batch_states)
        elif mppi_bandwidth / 2 < rev_angle < mppi_bandwidth:
            gait = self.astar_planner.gaits[0]
            path = []
            print('cw')
            return 'astar', gait, path
        else:
            gait = self.astar_planner.gaits[1]
            path = []
            print('ccw')
            return 'astar', gait, path

    def _plan_mppi_only(self, prev_n_pose_time_tups, rest_lens, motor_speeds):
        actions, states, batch_states = self.mppi_controller.plan(
            prev_n_pose_time_tups, rest_lens, motor_speeds)
        actions = actions.cpu().clone().numpy()
        self.prev_mode = 'mppi'

        return 'mppi', actions, (states, batch_states)

    def _plan_astar_only(self, prev_n_pose_time_tups, rest_lens, motor_speeds):
        gait, path = self.astar_planner.plan(
            prev_n_pose_time_tups, rest_lens, motor_speeds)
        self.prev_primitive = gait

        self.logger.info(f'Run primitive {gait}.')

        return 'astar', gait, path