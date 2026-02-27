from typing import Dict, Tuple, Union, Optional

import torch

from gnn_simulator.actuation.pid import PID
from gnn_simulator.linear_contact.collision_detector import get_detector
from gnn_simulator.linear_contact.collision_response import CollisionResponseGenerator
from gnn_simulator.linear_contact.learnable_collision_response import DiffCollisionResponseGenerator
from gnn_simulator.robots.tensegrity import TensegrityRobot
from gnn_simulator.simulators.abstract_simulator import AbstractPhysicsSimulator
from gnn_simulator.utilities import torch_quaternion


class TensegrityRobotSimulator(AbstractPhysicsSimulator):

    def __init__(self,
                 tensegrity_cfg,
                 gravity,
                 contact_params,
                 learn_contact_params=False):
        robot = self.build_robot(tensegrity_cfg)
        super().__init__(robot)

        self.gravity = gravity
        if isinstance(gravity, list):
            gravity = torch.tensor(gravity, dtype=self.dtype)
            self.gravity = gravity.reshape(1, 3, 1)

        self.pids = None

        contact_params = {k: torch.tensor(v, dtype=self.dtype) for k, v in contact_params.items()}
        if learn_contact_params:
            contact_params = torch.nn.ParameterDict(contact_params)
            self.collision_resp_gen = DiffCollisionResponseGenerator()
            self.collision_resp_gen.set_contact_params('default', contact_params)
        else:
            self.collision_resp_gen = CollisionResponseGenerator()
            self.collision_resp_gen.set_contact_params('default', contact_params)

    def reset(self, **kwargs):
        if 'act_lens' in kwargs:
            cables = self.robot.actuated_cables.values()
            for i, cable in enumerate(cables):
                cable.actuation_length = kwargs['act_lens'][:, i: i + 1].clone()

        if 'motor_speeds' in kwargs:
            cables = self.robot.actuated_cables.values()
            for i, cable in enumerate(cables):
                cable.motor.motor_state.omega_t = kwargs['motor_speeds'][:, i: i + 1].clone()

    @property
    def num_rods(self):
        return self.robot.num_rods

    def build_robot(self, cfg):
        return TensegrityRobot(cfg)

    def to(self, device):
        super().to(device)

        self.robot = self.robot.to(device)
        self.gravity = self.gravity.to(device)

        if self.collision_resp_gen:
            self.collision_resp_gen.to(device)

        # for k, pid in self.pids.items():
        #    self.pids[k] = pid.to(device)

        return self

    @property
    def rigid_bodies(self):
        return self.robot.rods

    def get_body_vecs(self, curr_state, acting_pts):
        num_bodies = len(self.robot.rods)
        pos = torch.hstack([curr_state[:, i * 13: i * 13 + 3]
                            for i in range(num_bodies)])
        body_vecs = acting_pts - pos

        return body_vecs

    def compute_forces(self,
                       external_forces: torch.Tensor,
                       external_pts: torch.Tensor
                       ) -> Tuple[torch.Tensor, Optional[torch.Tensor], Optional[torch.Tensor]]:
        net_spring_forces, spring_forces, acting_pts \
            = self.robot.compute_cable_forces()
        gravity_forces = torch.hstack([rod.mass * self.gravity
                                       for rod in self.robot.rods.values()])
        net_forces = net_spring_forces + gravity_forces

        return net_forces, spring_forces, acting_pts

    def compute_torques(self,
                        forces: torch.Tensor,
                        body_vecs: torch.Tensor
                        ) -> Tuple[torch.Tensor, Optional[torch.Tensor]]:
        shape = forces.shape
        torques = torch.cross(
            body_vecs.view(-1, 3, shape[2]),
            forces.view(-1, 3, shape[2]),
            dim=1
        ).view(shape)

        net_torques = torques.sum(dim=2, keepdim=True)

        return net_torques, torques

    def compute_accelerations(self,
                              net_force: torch.Tensor,
                              net_torque: torch.Tensor
                              ) -> Tuple[torch.Tensor, torch.Tensor]:
        batch_size = net_force.shape[0]

        rods = self.robot.rods.values()
        masses = torch.hstack([
            rod.mass for rod in rods
        ]).repeat(batch_size, 1, 1).reshape(-1, 1, 1)
        I_world_inv = torch.hstack([
            rod.I_world_inv.reshape(-1, 9) for rod in rods
        ]).reshape(-1, 3, 3)

        lin_acc = (net_force.reshape(-1, 3, 1)
                   / masses).reshape(batch_size, -1, 1)
        ang_acc = (I_world_inv @ net_torque.reshape(-1, 3, 1)
                   ).reshape(batch_size, -1, 1)

        return lin_acc, ang_acc

    def time_integration(self,
                         lin_acc: torch.Tensor,
                         ang_acc: torch.Tensor,
                         dt: float) -> torch.Tensor:
        curr_state = self.get_curr_state().reshape(-1, 13, 1)
        pos, quat = curr_state[:, :3], curr_state[:, 3:7]
        lin_vel, ang_vel = curr_state[:, 7:10], curr_state[:, 10:]

        next_lin_vel = lin_vel + dt * lin_acc.reshape(-1, 3, 1)
        next_ang_vel = ang_vel + dt * ang_acc.reshape(-1, 3, 1)

        next_pos = pos + dt * next_lin_vel
        next_quat = torch_quaternion.update_quat(quat, next_ang_vel, dt)

        batch_size = lin_acc.shape[0]
        next_state = torch.hstack([
            next_pos,
            next_quat,
            next_lin_vel,
            next_ang_vel
        ]).reshape(batch_size, -1, 1)

        return next_state

    def compute_contact_deltas(self,
                               pre_next_state: torch.Tensor,
                               dt: Union[torch.Tensor, float]
                               ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        delta_v, delta_w, toi = [], [], []
        for i, rod in enumerate(self.robot.rods.values()):
            rod_pre_next_state = pre_next_state[:, i * 13: (i + 1) * 13]
            detector = get_detector(rod, self.collision_resp_gen.ground)
            _, _, dv, dw, dtoi = self.collision_resp_gen.resolve_contact_ground(rod,
                                                                                rod_pre_next_state,
                                                                                dt,
                                                                                detector)
            delta_v.append(dv)
            delta_w.append(dw)
            toi.append(dtoi)

        delta_v = torch.vstack(delta_v)
        delta_w = torch.vstack(delta_w)
        toi = torch.vstack(toi)

        return delta_v, delta_w, toi

    def resolve_contacts(self,
                         pre_next_state: torch.Tensor,
                         dt: Union[torch.Tensor, float],
                         delta_v,
                         delta_w,
                         toi) -> torch.Tensor:
        rods = self.robot.rods.values()
        n = len(rods)
        pre_next_state_ = torch.vstack([pre_next_state[:, i * 13: (i + 1) * 13] for i in range(n)])
        size = pre_next_state.shape[0]
        num = int(delta_v.shape[0] // size)

        lin_vel = pre_next_state_[:, 7:10, ...] + delta_v
        ang_vel = pre_next_state_[:, 10:, ...] + delta_w
        pos = torch.vstack([r.pos for r in rods]) + dt * lin_vel - delta_v * toi
        # pos2 = pre_next_state_[:, :3] + delta_v * (dt - toi)

        # curr_quat = torch.vstack([r.quat for r in rods])
        # prin_axis = torch_quaternion.compute_quat_btwn_z_and_vec(curr_quat)
        # prin_ang_vel = torch.linalg.vecdot(ang_vel, prin_axis, dim=1).unsqueeze(1) * prin_axis
        # ang_vel = ang_vel - self.rolling_friction * prin_ang_vel

        # quat = torch_quaternion.update_quat(curr_quat, ang_vel, dt)
        # quat = torch_quaternion.update_quat(quat, -delta_w, toi)
        quat = torch_quaternion.update_quat(pre_next_state_[:, 3:7], delta_w, dt - toi)

        next_state_ = torch.hstack([pos, quat, lin_vel, ang_vel])
        next_state = torch.hstack([next_state_[i * size: (i + 1) * size] for i in range(num)])

        return next_state

    def _compute_cable_length(self, cable):
        sites_dict = self.robot.system_topology.sites_dict
        end_pt0 = sites_dict[cable.end_pts[0]]
        end_pt1 = sites_dict[cable.end_pts[1]]

        x_dir = end_pt1 - end_pt0
        length = x_dir.norm(dim=1, keepdim=True)
        x_dir = x_dir / length

        return length, x_dir

    def apply_control(self, control_signals, dt):
        if control_signals is None:
            return

        if isinstance(control_signals, torch.Tensor):
            control_signals = {
                f'cable_{i}': control_signals[:, i: i + 1, None]
                for i in range(control_signals.shape[1])
            }
        elif isinstance(control_signals, list):
            control_signals = {
                f'cable_{i}':
                    ctrl.reshape(-1, 1, 1)
                    if isinstance(ctrl, torch.Tensor)
                    else torch.tensor(ctrl, dtype=self.dtype).reshape(-1, 1, 1)
                for i, ctrl in enumerate(control_signals)
            }

        for name, control in control_signals.items():
            if not isinstance(control, torch.Tensor):
                control = torch.tensor(
                    control,
                    dtype=self.dtype,
                    device=self.device
                ).reshape(-1, 1)

            measure_name = self.robot.cable_map[name]
            measure_cable = self.robot.cables[measure_name]
            cable = self.robot.cables[name]

            curr_length, _ = self.robot.compute_cable_length(measure_cable)
            cable.update_rest_length(control, curr_length, dt)

    def step_with_target_gait(self,
                              curr_state: torch.Tensor,
                              dt: Union[torch.Tensor, float],
                              external_forces: Dict = None,
                              external_pts: Dict = None,
                              target_gait_dict: Dict = None):
        if isinstance(dt, float):
            dt = torch.tensor(
                [[[dt]]],
                dtype=curr_state.dtype,
                device=curr_state.device
            )

        # If no external forces provided, assume 0 force
        if external_forces is None or external_pts is None:
            size = (curr_state.shape[0], 3, 1)
            external_forces = torch.zeros(size, dtype=self.dtype, device=curr_state.device)
            external_pts = torch.zeros(size, dtype=self.dtype, device=curr_state.device)

        # Compute state and all other properties
        self.update_state(curr_state)

        controls = {}
        for name, target in target_gait_dict.items():
            if not isinstance(target, torch.Tensor):
                target = torch.tensor(
                    target,
                    dtype=self.dtype,
                    device=self.device
                ).reshape(-1, 1).detach()

            measure_name = self.robot.cable_map[name]
            measure_cable = self.robot.cables[measure_name]
            cable = self.robot.actuated_cables[name]

            curr_length, _ = self.robot.compute_cable_length(measure_cable)

            control, pos = self.pids[f"pid_{name}"].update_control_by_target_gait(
                curr_length,
                target,
                cable.rest_length
            )

            controls[name] = control.detach()

        self.robot.cables.update(self.robot.actuated_cables)

        # Apply control signals
        self.apply_control(controls, dt)

        # Compute all (and net) forces
        net_force, forces, acting_pts = self.compute_forces(external_forces, external_pts)

        # Compute all (and net) torques
        body_vecs = self.get_body_vecs(curr_state, acting_pts)
        net_torque, _ = self.compute_torques(forces, body_vecs)

        # Compute the current linear and angular accelerations from net force and torque
        lin_acc, ang_acc = self.compute_accelerations(net_force, net_torque)

        # Compute next rod state (pos, lin vel, quat, ang vel)
        pre_next_state = self.time_integration(lin_acc, ang_acc, dt)

        # Resolve contacts
        delta_v, delta_w, toi = self.compute_contact_deltas(pre_next_state, dt)
        next_state = self.resolve_contacts(pre_next_state,
                                           dt,
                                           delta_v,
                                           delta_w,
                                           toi)

        return next_state, list(controls.values())

    def run_with_target_gait(self, curr_state, dt, target_gait_dict, pid_params, max_steps=1000):
        self.pids = {f"pid_{k}": PID(**pid_params)
                     for k in self.robot.actuated_cables.keys()}

        step = 0
        controls = torch.tensor(1.)
        states, graphs = [], []

        while (controls != 0.0).any() and step < max_steps:
            step += 1
            (curr_state, curr_graph), controls = self.step_with_target_gait(
                curr_state,
                dt,
                target_gait_dict=target_gait_dict
            )
            states.append(curr_state.clone())
            graphs.append(curr_graph.clone())
            controls = torch.hstack(controls).detach()

        return states, graphs

    def run_until_stable(self, dt, tol=2e-2, max_time=10):
        with torch.no_grad():
            time = 0.0
            curr_state = self.get_curr_state()
            num_bodies = len(self.robot.rods)
            vels = torch.ones(num_bodies * 3, dtype=self.dtype)

            while (torch.abs(vels) > tol).any():
                if time > max_time:
                    raise Exception('Stability could not be reached within 5 seconds')

                curr_state = self.step(curr_state, dt)
                # print(curr_state.flatten())

                time += dt
                vels = torch.hstack([
                    curr_state[:, i * 13 + 7: i * 13 + 10]
                    for i in range(num_bodies)
                ])

                # print(time, torch.abs(vels).max())

            self.update_state(curr_state)

        print("Stabilization complete")

        return curr_state

    def reset_pids(self):
        if hasattr(self, 'pids') and self.pids is not None:
            for k, pid in self.pids.items():
                pid.reset()

    def reset_actuation(self):
        for k, cable in self.robot.actuated_cables.items():
            cable.reset_cable()


class Tensegrity5dRobotSimulator(TensegrityRobotSimulator):

    def __init__(self,
                 tensegrity_cfg,
                 gravity,
                 contact_params,
                 learn_contact_params=False):
        super().__init__(tensegrity_cfg,
                         gravity,
                         contact_params,
                         learn_contact_params)
        self.rolling_friction = self.collision_resp_gen.contact_params.rolling_friction

    def resolve_contacts(self,
                         pre_next_state: torch.Tensor,
                         dt: Union[torch.Tensor, float],
                         delta_v,
                         delta_w,
                         toi) -> torch.Tensor:
        curr_state_ = self.get_curr_state().reshape(-1, 13, 1)
        pre_next_state_ = pre_next_state.reshape(-1, 13, 1)

        lin_vel = pre_next_state_[:, 7:10] + delta_v
        ang_vel = pre_next_state_[:, 10:] + delta_w
        pos = curr_state_[:, :3] + dt * lin_vel - delta_v * toi

        curr_quat = curr_state_[:, 3:7]
        prin_axis = torch_quaternion.quat_as_rot_mat(curr_quat)[..., 2:3]
        prin_ang_vel = torch.linalg.vecdot(ang_vel, prin_axis, dim=1).unsqueeze(1) * prin_axis
        ang_vel = ang_vel - self.rolling_friction * prin_ang_vel

        quat = torch_quaternion.update_quat(curr_quat, ang_vel, dt)
        quat = torch_quaternion.update_quat(quat, -delta_w, toi)

        next_state = torch.hstack([
            pos, quat, lin_vel, ang_vel
        ]).reshape(pre_next_state.shape)

        return next_state

    def step(self,
             curr_state: torch.Tensor,
             dt: Union[torch.Tensor, float],
             external_forces: Optional[torch.Tensor] = None,
             external_pts: Optional[torch.Tensor] = None,
             control_signals: Optional[torch.Tensor] = None,
             **kwargs) -> Tuple[torch.Tensor, torch.Tensor]:
        next_state = super().step(curr_state, dt, external_forces, external_pts, control_signals)

        return next_state, None
