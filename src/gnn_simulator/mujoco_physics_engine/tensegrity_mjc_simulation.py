import json
import random
import shutil
from copy import deepcopy
from pathlib import Path
from typing import Optional, List

import numpy as np
import torch
import tqdm

from gnn_simulator.mujoco_physics_engine.cable_motor import DCMotor
from gnn_simulator.mujoco_physics_engine.mujoco_simulation import AbstractMuJoCoSimulator
from gnn_simulator.mujoco_physics_engine.pid import PID

import mujoco

from gnn_simulator.mujoco_visualizer_utils.mujoco_visualizer import MuJoCoVisualizer
from gnn_simulator.utilities import torch_quaternion

CENTER_SITES = [
    ("s3", "s5"),
    ("s1", "s3"),
    ("s5", "s1"),
    ("s0", "s2"),
    ("s4", "s0"),
    ("s2", "s4"),
    ("s2", "s5"),
    ("s0", "s3"),
    ("s1", "s4")
]

KUN_SITES = [
    ("s_3_5", "s_5_3"),
    ("s_1_3", "s_3_1"),
    ("s_1_5", "s_5_1"),
    ("s_0_2", "s_2_0"),
    ("s_0_4", "s_4_0"),
    ("s_2_4", "s_4_2"),
    ("s_2_5", "s_5_2"),
    ("s_0_3", "s_3_0"),
    ("s_1_4", "s_4_1")
]

REAL_ATTACH_SITES = [
    ("s_3_b5", "s_b5_3"),
    ("s_1_b3", "s_b3_1"),
    ("s_5_b1", "s_b1_5"),
    ("s_0_b2", "s_b2_0"),
    ("s_4_b0", "s_b0_4"),
    ("s_2_b4", "s_b4_2"),
    ("s_3_5", "s_5_3"),
    ("s_1_3", "s_3_1"),
    ("s_1_5", "s_5_1"),
    ("s_0_2", "s_2_0"),
    ("s_0_4", "s_4_0"),
    ("s_2_4", "s_4_2"),
    ("s_2_5", "s_5_2"),
    ("s_0_3", "s_3_0"),
    ("s_1_4", "s_4_1")
]

SIX_BAR_SURFACE_SITES = [
    ("s_0_10", "s_10_0"),
    ("s_1_4", "s_4_1"),
    ("s_2_6", "s_6_2"),
    ("s_1_3", "s_3_1"),
    ("s_4_8", "s_8_4"),
    ("s_2_5", "s_5_2"),
    ("s_5_6", "s_6_5"),
    ("s_7_11", "s_11_7"),
    ("s_7_8", "s_8_7"),
    ("s_0_9", "s_9_0"),
    ("s_9_10", "s_10_9"),
    ("s_3_11", "s_11_3"),
    ("s_0_8", "s_8_0"),
    ("s_0_4", "s_4_0"),
    ("s_1_10", "s_10_1"),
    ("s_3_10", "s_10_3"),
    ("s_9_11", "s_11_9"),
    ("s_7_9", "s_9_7"),
    ("s_2_4", "s_4_2"),
    ("s_1_2", "s_2_1"),
    ("s_3_6", "s_6_3"),
    ("s_6_11", "s_11_6"),
    ("s_5_7", "s_7_5"),
    ("s_5_8", "s_8_5"),
]

SIX_BAR_CENTER_SITES = [
    ("s0", "s10"),
    ("s1", "s4"),
    ("s2", "s6"),
    ("s1", "s3"),
    ("s4", "s8"),
    ("s2", "s5"),
    ("s5", "s6"),
    ("s7", "s11"),
    ("s7", "s8"),
    ("s0", "s9"),
    ("s9", "s10"),
    ("s3", "s11"),
    ("s0", "s8"),
    ("s0", "s4"),
    ("s1", "s10"),
    ("s3", "s10"),
    ("s9", "s11"),
    ("s7", "s9"),
    ("s2", "s4"),
    ("s1", "s2"),
    ("s3", "s6"),
    ("s6", "s11"),
    ("s5", "s7"),
    ("s5", "s8"),
]

CABLE_SITES = {
    "center": CENTER_SITES,
    "kun": KUN_SITES,
    "real_attach": REAL_ATTACH_SITES,
    "six_bar_surface": SIX_BAR_SURFACE_SITES,
    "six_bar_center": SIX_BAR_CENTER_SITES,
}


class TensegrityMuJoCoSimulator(AbstractMuJoCoSimulator):

    def __init__(self,
                 xml_path: Path,
                 visualize: bool = True,
                 render_size: (int, int) = (1280, 1280),
                 render_fps: int = 50,
                 min_len=0.2,
                 max_len=2.7,
                 attach_type='center',
                 num_rods=3,
                 n_actuators=6,
                 sphere_radius=0.175,
                 motor_speed=0.9,
                 winch_r=0.035,
                 scale_factor=10.0,
                 use_remote_viewer: bool = False,
                 remote_port: int = 8765,
                 overlay_callback=None,
                 remote_max_fps: int = None):
        super().__init__(xml_path, visualize, render_size, render_fps,
                         use_remote_viewer=use_remote_viewer,
                         remote_port=remote_port,
                         overlay_callback=overlay_callback,
                         remote_max_fps=remote_max_fps)
        self.num_rods = num_rods
        self.n_actuators = n_actuators
        self.sphere_radius = sphere_radius
        self.scale_factor = scale_factor

        self.min_cable_rest_length = min_len
        self.max_cable_rest_length = max_len
        self.actuator_tendon_ids = list(range(self.n_actuators))
        self.curr_ctrl = [0.0 for _ in range(self.n_actuators)]
        self.pids = [PID() for _ in range(self.n_actuators)]
        self.pid_freq = 0.01
        self.rod_names = {
            0: "r01",
            1: "r23",
            2: "r45"
        }
        self.cable_sites = CABLE_SITES[attach_type]

        self.cable_map = {
            i: i + (self.n_actuators if attach_type == 'real_attach' else 0)
            for i in range(n_actuators)
        }

        self.end_pts = [
            f's{i}' for i in range(2 * self.num_rods)
        ]
        self.stiffness = self.mjc_model.tendon_stiffness.copy()
        self.cable_motors = [DCMotor(np.array(motor_speed)) for _ in range(self.n_actuators)]
        self.winch_r = np.array(winch_r, dtype=np.float64)

    def get_se2(self):
        self.forward()

        end_pts = self.get_endpts()
        com = end_pts.mean(axis=0, keepdims=True)[:, :2]

        left, right = end_pts[::2].mean(axis=0, keepdims=True), end_pts[1::2].mean(axis=0, keepdims=True)
        prin = (right - left)[:, :2]
        prin /= np.linalg.norm(prin, axis=1, keepdims=True)
        angle = np.arctan2(prin[:, 1], prin[:, 0])  # wrt to x-axis

        return com, angle

    def get_pose(self):
        pose = self.mjc_data.qpos.reshape(-1, 7)
        return pose

    def get_vels(self):
        return self.mjc_data.qvel.reshape(-1, 6)

    def get_accels(self):
        return self.mjc_data.qacc.reshape(-1, 6)

    def get_curr_state(self):
        pose = self.get_pose()
        vels = self.get_vels()
        state = np.hstack([pose, vels]).reshape(1, -1, 1)
        return state

    def get_rest_lengths(self):
        return self.mjc_model.tendon_lengthspring[:, :1].flatten().copy()

    def get_cable_lengths(self):
        center_sites = [(int(s[1]) for s in sites) for sites in CENTER_SITES]
        end_pts = self.get_endpts()
        cable_lengths = np.vstack([end_pts[i] - end_pts[j] for i, j in center_sites])
        cable_lengths = np.linalg.norm(cable_lengths, axis=1)
        return cable_lengths

    def get_motor_max_speeds(self):
        return np.array([c.max_omega * c.speed for c in self.cable_motors]).flatten().copy()

    def get_motor_speeds(self):
        return np.concatenate([c.motor_state.omega_t for c in self.cable_motors]).flatten().copy()

    def get_unscaled_se2(self):
        com, angle = self.get_se2()
        com = com / self.scale_factor

        return com, angle

    def get_unscaled_pose(self):
        pose = self.get_pose().copy()  # Must copy to avoid modifying qpos
        pose[:, :3] = pose[:, :3] / self.scale_factor
        return pose

    def get_unscaled_vels(self):
        vels = self.get_vels().copy()  # Must copy to avoid modifying qvel
        vels[:, :3] = vels[:, :3] / self.scale_factor
        return vels

    def get_unscaled_curr_state(self):
        state = self.get_curr_state().copy()  # Must copy to avoid modifying state
        state[:, :3] = state[:, :3] / self.scale_factor
        state[:, 7:10] = state[:, 7:10] / self.scale_factor
        return state

    def get_unscaled_rest_lengths(self):
        return self.mjc_model.tendon_lengthspring[:, :1].flatten().copy() / self.scale_factor

    def get_unscaled_accels(self):
        accels = self.get_accels().copy()  # Must copy to avoid modifying qacc
        accels[:, :3] = accels[:, :3] / self.scale_factor
        return accels

    def get_unscaled_endpts(self):
        end_pts = self.get_endpts() / self.scale_factor
        return end_pts

    def get_unscaled_cable_lengths(self):
        cable_lengths = self.get_cable_lengths()
        cable_lengths = cable_lengths / self.scale_factor
        return cable_lengths

    def bring_to_grnd(self):
        self.forward()
        qpos = self.mjc_data.qpos.copy().reshape(-1, 7)
        end_pts = self.get_endpts().reshape(-1, 3)
        min_z = end_pts[:, 2].min()
        qpos[:, 2] -= min_z - self.sphere_radius
        self.mjc_data.qpos = qpos.reshape(1, -1)

    def reset(self):
        super().reset()
        self.bring_to_grnd()

        for motor in self.cable_motors:
            motor.reset_omega_t()

    def sim_step(self, controls=None):
        # if controls is None:
        #    controls = self.curr_ctrl.copy()
        # print(controls)

        mujoco.mj_forward(self.mjc_model, self.mjc_data)
        for i, sites in enumerate(self.cable_sites):
            rest_length = self.mjc_model.tendon_lengthspring[i, :1]

            s0 = self.mjc_data.sensor(f"pos_{sites[0]}").data
            s1 = self.mjc_data.sensor(f"pos_{sites[1]}").data
            dist = np.linalg.norm(s1 - s0, keepdims=True)

            self.mjc_model.tendon_stiffness[i] = np.zeros_like(self.stiffness[i]) \
                if dist < rest_length else self.stiffness[i]

            if controls is not None and i < self.n_actuators:
                dl = self.cable_motors[i].compute_cable_length_delta(
                    controls[:, i], self.winch_r, self.dt)

                rest_length = rest_length - dl
                # if (self.min_cable_rest_length > rest_length) or (rest_length > self.max_cable_rest_length):
                #     print(f"ERROR: rest length {rest_length} for cable {i} is out of bounds")

                rest_length = np.clip(
                    rest_length,
                    self.min_cable_rest_length,
                    self.max_cable_rest_length
                )

                self.mjc_model.tendon_lengthspring[i] = rest_length

        mujoco.mj_step(self.mjc_model, self.mjc_data)
        mujoco.mj_forward(self.mjc_model, self.mjc_data)
        self.sync_remote_viewer()

    def run_w_ctrls(self, ctrls):
        frames = [{'time': 0.0, 'pos': self.mjc_data.qpos.copy()}]

        for i, ctrl in enumerate(ctrls):
            self.sim_step(ctrl)
            frames.append({'time': i * self.dt, 'pos': self.mjc_data.qpos.copy()})

        return frames

    def get_endpts(self):
        self.forward()
        end_pts = []
        for end_pt_site in self.end_pts:
            end_pt = self.mjc_data.sensor(f"pos_{end_pt_site}").data.copy()
            end_pts.append(end_pt)

        end_pts = np.vstack(end_pts)
        return end_pts

    def detect_ground_endcaps(self):
        end_pts = self.get_endpts()
        aug_end_pts = [[(i, end_pts[i]), (i + 1, end_pts[i + 1])]
                       for i in range(0, len(end_pts), 2)]
        aug_end_pts = [min(e, key=lambda x: x[1].flatten()[2].item()) for e in aug_end_pts]

        ground_endcaps = tuple([a[0] for a in aug_end_pts])

        return ground_endcaps

    def run(self,
            end_time: float = None,
            num_steps: int = None,
            save_path: Path = None,
            pos_sensor_names: Optional[List] = None,
            quat_sensor_names: Optional[List] = None,
            linvel_sensor_names: Optional[List] = None,
            angvel_sensor_names: Optional[List] = None):

        mujoco.mj_forward(self.mjc_model, self.mjc_data)

        end_pts = [self.get_endpts()]
        pos = [self.mjc_data.qpos.copy()]
        vel = [self.mjc_data.qvel.copy()]
        frames = [self.render_frame("front")]
        num_steps_per_frame = int(1 / self.render_fps / self.dt)
        for n in range(num_steps):
            if (n + 1) % 100 == 0:
                print((n + 1) * self.dt)

            self.sim_step()

            mujoco.mj_forward(self.mjc_model, self.mjc_data)
            end_pts.append(self.get_endpts())
            pos.append(self.mjc_data.qpos.copy())
            vel.append(self.mjc_data.qvel.copy())

            if self.visualize and ((n + 1) % num_steps_per_frame == 0 or n == num_steps - 1):
                frame = self.render_frame()
                frames.append(frame.copy())

        self.save_video(Path(save_path, "gt_vid.mp4"), frames)

        return end_pts, pos, vel


class ThreeBarTensegrityMuJoCoSimulator(TensegrityMuJoCoSimulator):

    def run_primitive(self, prim_type, left_range=None, right_range=None):
        prim_gaits = {
            'cw': [[1., 1., 0., 0., 0., 0.], [0., 1., 0., 0., 0., 0.], [0., 1., 1., 0., 0.8, 0.]],
            'roll': [[1., 1., 0.1, 1., 1., 0.1], [0., 1., 1., 0., 1., 0.1]],
            'ccw': [[1., 1., 1., 0., 1., 1.], [1., 0., 1., 0., 1., 1.], [0., 0., 0., 0., 0., 0.]],
            # 'crawl': [[0.1, 0.0, 0.1, 0.0, 0.0, 0.0], [1.0, 0.1, 1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]]
        }
        # prim_gaits = {
        #     'ccw': [[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]],
        #     'cw': [[0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 1], [0, 0, 0.5, 0, 1, 1], [1, 1, 1, 1, 1, 1]],
        #     'roll': [[1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]]
        #     # 'crawl': [[0.1, 0.0, 0.1, 0.0, 0.0, 0.0], [1.0, 0.1, 1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]]
        # }
        rest_target_gait = [1., 1., 1., 1., 1., 1.]

        if "ccw" == prim_type:
            min_length = 100
            range_ = 100
            tol = 0.1
        elif "roll" == prim_type:
            min_length = 90
            range_ = 110
            tol = 0.1
        elif "cw" == prim_type:
            min_length = 80
            range_ = 120
            tol = 0.1
        else:
            min_length = 100
            range_ = 100
            tol = 0.1

        left_range = range_ if left_range is None else left_range
        right_range = range_ if right_range is None else right_range

        for i in range(self.n_actuators):
            range_ = left_range if i % 2 == 0 else right_range
            self.pids[i].min_length = min_length / 100
            self.pids[i].RANGE = range_ / 100
            self.pids[i].tol = tol

        target_gaits = [rest_target_gait] + prim_gaits[prim_type] + [rest_target_gait]

        data, extra_info = self.run_w_target_gaits(target_gaits)
        return data, extra_info


    def run_w_target_gaits(self, target_gaits, save_path=None, max_time_per_gait=10):
        symmetry_mapping = {
            (0, 2, 5): [0, 1, 2, 3, 4, 5], (0, 3, 5): [0, 1, 2, 3, 4, 5],
            (1, 2, 4): [1, 2, 0, 4, 5, 3], (1, 2, 5): [1, 2, 0, 4, 5, 3],
            (0, 3, 4): [2, 0, 1, 5, 3, 4], (1, 3, 4): [2, 0, 1, 5, 3, 4]
        }
        max_steps = max_time_per_gait // self.dt

        if save_path:
            save_path = Path(save_path)
            save_path.mkdir(exist_ok=True)

        self.forward()
        data = [{
            "time": 0.0,
            "end_pts": [
                self.mjc_data.sensor(f"pos_{s}").data.tolist()
                for s in self.end_pts
            ],
            "sites": {
                s: self.mjc_data.sensor(f"pos_{s}").data.tolist()
                for c in self.cable_sites for s in c
            },
            "pos": self.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
            "quat": self.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
            "linvel": self.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
            "angvel": self.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
            # "init_rest_lengths": self.mjc_model.tendon_lengthspring[:6, 0].tolist(),
            "pid": {
                "min_length": self.pids[0].min_length,
                "RANGE": self.pids[0].RANGE,
                "tol": self.pids[0].tol,
                "motor_speed": self.cable_motors[0].speed.item()
            }
        }]
        target_gaits_dicts = []
        extra_data = []
        key_frame_ids = []

        if self.visualize:
            frames = [self.render_frame()]

        num_steps_per_frame = int(1 / self.render_fps / self.dt)
        global_steps = 0
        num_steps = []
        for target_gait in tqdm.tqdm(target_gaits):
            for pid in self.pids:
                pid.reset()
            # print(k)
            step = 0
            controls = [1.]

            ground_endcap_idx = self.detect_ground_endcaps()
            if ground_endcap_idx in symmetry_mapping:
                order = symmetry_mapping[ground_endcap_idx]
            elif target_gait == [1., 1., 1., 1., 1., 1.]:
                order = [0, 1, 2, 3, 4, 5]
            else:
                raise Exception(f"Ground endcaps {ground_endcap_idx} not in symmetry mapping")

            target_gait = [target_gait[o] for o in order]

            target_gaits_dicts.append({
                'idx': global_steps,
                'target_gait': target_gait,
                'info': {
                    'min_length': int(self.pids[0].min_length * 100),
                    'RANGE': int(self.pids[0].RANGE * 100),
                    'tol': self.pids[0].tol,
                    'P': self.pids[0].k_p,
                    'I': self.pids[0].k_i,
                    'D': self.pids[0].k_d,
                    'max_speed': int(self.cable_motors[0].speed * 100),

                }
            })

            while any([c != 0 for c in controls]) and step < max_steps:
                # print(step)
                step += 1
                global_steps += 1

                if step == max_steps:
                    print('reached max steps')
                    break

                # print(global_steps)
                mujoco.mj_forward(self.mjc_model, self.mjc_data)

                if global_steps % (self.pid_freq // self.dt) == 0 or step == 1:
                    controls = []
                    curr_lens = []
                    for i in range(len(target_gait)):
                        pid = self.pids[i]
                        gait = target_gait[i]

                        rest_length = self.mjc_model.tendon_lengthspring[i, 0]
                        key = self.cable_map[i] if hasattr(self, "cable_map") and self.cable_map else i
                        s0 = self.mjc_data.sensor(f"pos_{self.cable_sites[key][0]}").data
                        s1 = self.mjc_data.sensor(f"pos_{self.cable_sites[key][1]}").data
                        curr_length = np.linalg.norm(s1 - s0)

                        ctrl, _ = pid.update_control_by_target_gait(curr_length, gait, rest_length)
                        controls.append(ctrl)
                        curr_lens.append(curr_length)

                # print([c.item() for c in controls])
                # print([c.item() for c in curr_lens])
                # print(self.mjc_model.tendon_lengthspring[:self.n_actuators, 0].flatten())

                extra_data.append({
                    "time": round(self.dt * (global_steps - 1), 4),
                    "rest_lengths": self.mjc_model.tendon_lengthspring[:self.n_actuators, 0].copy().tolist(),
                    "motor_speeds": [c.motor_state.omega_t[0].copy().item() for c in self.cable_motors],
                    "controls": [c.copy().item() for c in controls]
                })

                self.sim_step(np.array(controls).reshape(1, -1))
                self.forward()

                data.append({
                    "time": round(self.dt * global_steps, 4),
                    "end_pts": [
                        self.mjc_data.sensor(f"pos_{s}").data.tolist()
                        for s in self.end_pts
                    ],
                    "sites": {
                        s: self.mjc_data.sensor(f"pos_{s}").data.tolist()
                        for c in self.cable_sites for s in c
                    },
                    "pos": self.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                    "quat": self.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
                    "linvel": self.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                    "angvel": self.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
                    "rest_lengths": self.mjc_model.tendon_lengthspring[:self.n_actuators, 0].copy().tolist(),
                    "motor_speeds": [c.motor_state.omega_t[0].copy() for c in self.cable_motors],
                })

                if self.visualize and (global_steps % num_steps_per_frame == 0):
                    frame = self.render_frame()
                    frames.append(frame)
            num_steps.append(step)
            key_frame_ids.append(global_steps)

        return data[:-1], extra_data

    def get_heading_angle(self):
        end_pts = self.get_endpts()
        left = end_pts[::2].mean(axis=0)
        right = end_pts[1::2].mean(axis=0)
        prin = right - left
        prin /= np.linalg.norm(prin, axis=0)
        angle = np.arctan2(prin[1], prin[0])

        return angle

    def align_prin(self, new_prin, new_com):
        import torch
        from gnn_simulator.utilities import torch_quaternion

        new_prin = torch.from_numpy(new_prin).reshape(1, 3, 1)
        new_com = torch.from_numpy(new_com).reshape(1, 2, 1)
        new_prin[:, 2] = 0

        self.forward()

        pose = self.mjc_data.qpos.reshape(-1, 7, 1)
        pos, quat = pose[:, :3], pose[:, 3:]
        end_pts = self.get_endpts()

        pos = torch.from_numpy(pos)
        quat = torch.from_numpy(quat)
        end_pts = torch.from_numpy(end_pts).reshape(-1, 3, 1)

        mid_left = end_pts[::2].mean(dim=0, keepdim=True)
        mid_right = end_pts[1::2].mean(dim=0, keepdim=True)
        prins = mid_right - mid_left
        prins[:, 2] = 0.
        prins = prins / prins.norm(dim=1, keepdim=True)

        curr_com = torch.mean(pos, dim=0, keepdim=True)

        rot_dir = torch.cross(prins, new_prin, dim=1)
        rot_dir /= rot_dir.norm(dim=1)

        angle = torch.linalg.vecdot(prins, new_prin, dim=1).unsqueeze(1)
        angle = torch.clamp(angle, -1, 1)
        angle = torch.acos(angle) / 2

        rot_quat = torch.hstack([torch.cos(angle), torch.sin(angle) * rot_dir])

        new_pos = torch_quaternion.rotate_vec_quat(rot_quat, pos - curr_com)
        new_pos[:, :2] += new_com
        new_pos[:, 2] += curr_com[:, 2]
        new_quat = torch_quaternion.quat_prod(rot_quat, quat)

        self.mjc_data.qpos = torch.hstack([new_pos, new_quat]).flatten().numpy()
        self.forward()

    def flip_to_next_support_tri(self):
        self.forward()
        pose = self.mjc_data.qpos.reshape(-1, 7, 1)
        pos, quat = pose[:, :3], pose[:, 3:]
        end_pts = self.get_endpts()

        pos = torch.from_numpy(pos).reshape(-1, 3, 1)
        quat = torch.from_numpy(quat).reshape(-1, 4, 1)
        end_pts = torch.from_numpy(end_pts).reshape(-1, 3, 1)

        mid_left = end_pts[::2].mean(dim=0, keepdim=True)
        mid_right = end_pts[1::2].mean(dim=0, keepdim=True)
        prins = mid_right - mid_left
        prins = prins / prins.norm(dim=1, keepdim=True)

        curr_com = torch.mean(pos, dim=0, keepdim=True)

        angle = torch.tensor(torch.pi / 3, dtype=prins.dtype).reshape(1, 1, 1)
        rot_quat = torch.hstack([torch.cos(angle), prins * torch.sin(angle)])

        new_pos = torch_quaternion.rotate_vec_quat(rot_quat, pos - curr_com) + curr_com
        new_quat = torch_quaternion.quat_prod(rot_quat, quat)
        self.mjc_data.qpos = torch.hstack([new_pos, new_quat]).flatten().numpy()

        self.bring_to_grnd()
        self.forward()


class SixBarTensegrityMuJoCoSimulator(TensegrityMuJoCoSimulator):

    def __init__(self, xml_path: Path, attach_type='six_bar_center', visualize=True,
                 use_remote_viewer: bool = False, remote_port: int = 8765,
                 overlay_callback=None):
        super().__init__(xml_path,
                         visualize=visualize,
                         attach_type=attach_type,
                         num_rods=6,
                         n_actuators=24,
                         use_remote_viewer=use_remote_viewer,
                         remote_port=remote_port,
                         overlay_callback=overlay_callback)
        self.rod_names = {
            0: "r01",
            1: "r23",
            2: "r45",
            3: "r67",
            4: "r89",
            5: "r1011"
        }


def gen_3bar_data(output_dir: Path, xml: Path):
    output_dir.parent.mkdir(exist_ok=True)
    output_dir.mkdir(exist_ok=True)

    shutil.copy(xml, output_dir / xml.name)
    dummy_sim = ThreeBarTensegrityMuJoCoSimulator(xml, attach_type='real_attach')

    metadata = {
        'motor_max_omega': [c.max_omega.item() for c in dummy_sim.cable_motors],
    }

    with (output_dir / 'metadata.json').open('w') as fp:
        json.dump(metadata, fp)

    del dummy_sim

    prim_gaits = {
        'cw': [[1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 1, 1, 0, 0.8, 0], [1, 1, 1, 1, 1, 1]],
        'roll': [[1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]],
        'ccw': [[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]],
        # 'crawl': [[0.1, 0.0, 0.1, 0.0, 0.0, 0.0], [1.0, 0.1, 1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]]
    }

    for prim_type in prim_gaits.keys():
        if "ccw" == prim_type:
            min_length = 1.0
            range_ = 1.0
            tol = 0.1
        elif "roll" == prim_type:
            min_length = 0.9
            range_ = 1.1
            tol = 0.1
        elif "cw" == prim_type:
            min_length = 0.8
            range_ = 1.2
            tol = 0.1
        else:
            min_length = 1.0
            range_ = 1.0
            tol = 0.1

        # 20-30 trajectories, of length 15 sampled primitives
        # for

        traj1 = ['ccw', 'cw', 'cw', 'roll']
        traj2 = ['roll', 'ccw', 'ccw', 'ccw']
        traj20 = [...]

        for j in range(20):
            sim = ThreeBarTensegrityMuJoCoSimulator(xml, attach_type='real_attach')
            out = Path(output_dir, f"{prim_type}_{j}")
            out.mkdir(exist_ok=True)
            print(out.name)

            # print(sim.get_endpts())

            for _ in range(j):
                sim.flip_to_next_support_tri()
            # print(sim.get_endpts())

            # for pid in sim.pids:
            #     pid.min_length = min_length
            #     pid.RANGE = range_
            #     pid.tol = tol

            sim.run_w_target_gaits([[1., 1., 1., 1., 1., 1.]])

            for _ in range(1000):
                sim.sim_step(controls=np.zeros((1, 6)))

            # sim.align_prin(np.array([1.0, 0., 0.]).reshape(1, 3, 1), np.array([0., 0., 0.]).reshape(1, 3, 1))
            sim.mjc_data.qvel = np.zeros_like(sim.mjc_data.qvel)

            # all_gaits = 10 * prim_gaits[prim_type]
            all_gaits = [gait for _ in range(traj_length) for gait in prim_gaits[sample(prim_gaits.keys())]]
            all_data, all_extra_data = sim.run_w_target_gaits(all_gaits)

            with Path(out, "processed_data.json").open("w") as fp:
                json.dump(all_data, fp)

            with Path(out, "extra_state_data.json").open("w") as fp:
                json.dump(all_extra_data, fp)

            frames = []
            for d in all_data[::4]:
                pos = np.array(d['pos'], dtype=np.float64).reshape(-1, 3)
                quat = np.array(d['quat'], dtype=np.float64).reshape(-1, 4)
                pose = np.hstack([pos, quat]).flatten()
                sim.mjc_data.qpos = pose
                sim.forward()
                frame = sim.render_frame()

                frames.append(frame)

            sim.save_video(Path(out, 'gt_vid.mp4'), frames)
            del sim


def get_3bar_prims_se2_data(xml):
    # prim_gaits = {
    #     'cw': [[1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 1, 1, 0, 0.8, 0], [1, 1, 1, 1, 1, 1]],
    #     'roll': [[1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]],
    #     'ccw': [[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]],
    #     # 'crawl': [[0.1, 0.0, 0.1, 0.0, 0.0, 0.0], [1.0, 0.1, 1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]]
    # }
    prim_gaits = {
        'ccw': [[1, 1, 1, 0, 1, 1], [1, 0, 1, 0, 1, 1], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]],
        'cw': [[0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 1], [0, 0, 0.5, 0, 1, 1], [1, 1, 1, 1, 1, 1]],
        'roll': [[1, 1, 0.1, 1, 1, 0.1], [0, 1, 1, 0, 1, 0.1], [1, 1, 1, 1, 1, 1]]
        # 'crawl': [[0.1, 0.0, 0.1, 0.0, 0.0, 0.0], [1.0, 0.1, 1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]]
    }

    prims = [
        ('cw', 120, 120),
        ('ccw', 100, 100),
        ('roll', 100, 100),
        ('roll', 100, 120),
        ('roll', 100, 140),
        ('roll', 120, 100),
        ('roll', 120, 120),
        ('roll', 120, 140),
        ('roll', 140, 100),
        ('roll', 140, 120),
        ('roll', 140, 140)
    ]

    for prim_type, left, right in prims:
        if "ccw" == prim_type:
            min_length = 1.0
            range_ = 1.0
            tol = 0.1
        elif "roll" == prim_type:
            min_length = 0.9
            range_ = 1.0
            tol = 0.1
        elif "cw" == prim_type:
            min_length = 0.8
            range_ = 1.1
            tol = 0.1
        else:
            min_length = 1.0
            range_ = 1.0
            tol = 0.1

        sim = ThreeBarTensegrityMuJoCoSimulator(xml, attach_type='real_attach', visualize=False)

        for j, pid in enumerate(sim.pids):
            pid.min_length = min_length
            if j % 2 == 0:
                pid.RANGE = left / 100
            else:
                pid.RANGE = right / 100
            pid.tol = tol

        for _ in range(1000):
            sim.sim_step(controls=np.zeros((1, 6)))

        sim.align_prin(np.array([1.0, 0., 0.]).reshape(1, 3, 1), np.array([0., 0.]).reshape(1, 2, 1))
        sim.mjc_data.qvel = np.zeros_like(sim.mjc_data.qvel)

        init_end_pts = sim.get_endpts()
        init_com = init_end_pts.mean(axis=0)
        init_left, init_right = init_end_pts[::2].mean(axis=0), init_end_pts[1::2].mean(axis=0)
        init_prin = init_right - init_left
        init_prin = init_prin[:2] / np.linalg.norm(init_prin[:2])

        all_gaits = prim_gaits[prim_type]
        all_data, all_extra_data = sim.run_w_target_gaits(all_gaits)

        final_end_pts = sim.get_endpts()
        final_com = final_end_pts.mean(axis=0)
        final_left, final_right = final_end_pts[::2].mean(axis=0), final_end_pts[1::2].mean(axis=0)
        final_prin = final_right - final_left
        final_prin = final_prin[:2] / np.linalg.norm(final_prin[:2])

        dx = final_com[:2] - init_com[:2]
        dtheta = np.arctan2(final_prin[1], final_prin[0])

        print(prim_type, left, right, (dx[0], dx[1], dtheta))

        del sim


def gen_3bar_data_real_dl(real_data_dir, output_dir, xml_path):
    output_dir.mkdir(exist_ok=True)
    shutil.copy(xml_path, output_dir / xml_path.name)

    for p in real_data_dir.iterdir():
        if 'real' not in p.name or not p.is_dir():
            continue
        print(p.name)

        sim = TensegrityMuJoCoSimulator(xml_path, attach_type='real_attach')
        name_split = p.name.split('_')
        out = Path(output_dir, f"mjc_{name_split[1]}_{name_split[2]}")
        out.mkdir(exist_ok=True)

        gt_extra = json.load((p / "raw_extra_state_data.json").open("r"))
        interp_times = np.arange(0, gt_extra[-1]['time'], 0.01)
        raw_enc_counts = np.array([e['encoder_counts'] for e in gt_extra])
        raw_enc_lens = 0.082 * np.pi * raw_enc_counts / 1800
        raw_enc_lens[:, 3] *= -1
        raw_rest_lens = np.array([gt_extra[0]['rest_lengths']]) + raw_enc_lens - raw_enc_lens[:1]

        rest_lengths, controls = [], []
        k, t0, t1 = -1, -1, -1
        for t in interp_times:
            while not t0 <= t <= t1:
                k += 1
                t0, t1 = gt_extra[k]['time'], gt_extra[k + 1]['time']

            controls.append(gt_extra[k]['controls'])
            r0 = raw_rest_lens[k]
            r1 = raw_rest_lens[k + 1]
            w = (t - t0) / (t1 - t0)
            r = (1 - w) * r0 + w * r1
            rest_lengths.append(r)

        sim.forward()
        for i, sites in enumerate(sim.cable_sites):
            if i < 6:
                sim.mjc_model.tendon_lengthspring[i, 0] = gt_extra[0]['rest_lengths'][i]
                sim.mjc_model.tendon_lengthspring[i, 1] = gt_extra[0]['rest_lengths'][i]

        data = []
        extra_data = []
        for m in range(500):
            sim.forward()

            data.append({
                "time": 0.01 * m,
                "end_pts": [
                    sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for s in sim.end_pts
                ],
                "sites": {
                    s: sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for c in sim.cable_sites for s in c
                },
                "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
                "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
            })

            extra_data.append({
                "time": 0.01 * m,
                "rest_lengths": sim.mjc_model.tendon_lengthspring[:6, 0].tolist(),
                "controls": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            })

            sim.sim_step(controls=np.zeros((1, 6)))

        start_time = data[-1]['time']
        for m, r in enumerate(rest_lengths[1:]):
            data.append({
                "time": 0.01 * m + start_time,
                "end_pts": [
                    sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for s in sim.end_pts
                ],
                "sites": {
                    s: sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for c in sim.cable_sites for s in c
                },
                "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
                "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
            })

            extra_data.append({
                "time": 0.01 * m + start_time,
                "rest_lengths": sim.mjc_model.tendon_lengthspring[:6, 0].tolist(),
                "controls": controls[m]
            })

            sim.forward()
            for i, sites in enumerate(sim.cable_sites):
                if i < 6:
                    sim.mjc_model.tendon_lengthspring[i, 0] = r[i]
                    sim.mjc_model.tendon_lengthspring[i, 1] = r[i]
            sim.sim_step(controls=np.zeros((1, 6)))

        start_time = data[-1]['time']
        for m in range(300):
            sim.forward()

            data.append({
                "time": 0.01 * (m + 1) + start_time,
                "end_pts": [
                    sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for s in sim.end_pts
                ],
                "sites": {
                    s: sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for c in sim.cable_sites for s in c
                },
                "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
                "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
            })

            extra_data.append({
                "time": 0.01 * (m + 1) + start_time,
                "rest_lengths": sim.mjc_model.tendon_lengthspring[:6, 0].tolist(),
                "controls": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            })

            sim.sim_step(controls=np.zeros((1, 6)))

        with Path(out, "processed_data.json").open("w") as fp:
            json.dump(data, fp)

        with Path(out, "extra_state_data.json").open("w") as fp:
            json.dump(extra_data, fp)

        frames_front, frames_top = [], []
        for d in data[::4]:
            pos = np.array(d['pos'], dtype=np.float64).reshape(-1, 3)
            quat = np.array(d['quat'], dtype=np.float64).reshape(-1, 4)
            pose = np.hstack([pos, quat]).flatten()
            sim.mjc_data.qpos = pose
            sim.forward()
            frames_front.append(sim.render_frame('front'))
            frames_top.append(sim.render_frame('camera'))

        sim.save_video(Path(out, 'gt_vid_front.mp4'), frames_front)
        sim.save_video(Path(out, 'gt_vid_top.mp4'), frames_top)
        del sim


def regen_6bar_data(data_dirs, output_dir, xml_path):
    for data_dir in data_dirs:
        with (data_dir / 'processed_data.json').open('r') as fp:
            data = json.load(fp)

        with (data_dir / 'extra_state_data.json').open('r') as fp:
            extra_data = json.load(fp)

        init_pos = np.array(data[0]['pos']).reshape(-1, 3)
        init_quat = np.array(data[0]['quat']).reshape(-1, 4)
        init_pose = np.hstack([init_pos, init_quat]).flatten()
        init_rest_lens = extra_data[0]['rest_lengths']
        init_motor_speeds = extra_data[0]['motor_speeds']
        all_controls = [e['controls'] for e in extra_data]

        sim = SixBarTensegrityMuJoCoSimulator(xml_path, 'six_bar_surface', visualize=True)
        sim.mjc_data.qpos = init_pose
        sim.mjc_model.tendon_lengthspring[:, 0] = init_rest_lens
        sim.mjc_model.tendon_lengthspring[:, 1] = init_rest_lens
        for j, motor in enumerate(sim.cable_motors):
            motor.motor_state.omega_t = np.array(init_motor_speeds[j])

        sim.forward()

        if 'mppi' in data_dir.name:
            for _ in range(2000):
                sim.sim_step(np.zeros((1, 24)))

        new_data, new_extra_data = [], []
        for j, ctrls in enumerate(tqdm.tqdm(all_controls)):
            ctrls = np.array(ctrls).reshape(1, -1)
            new_data.append({
                "time": sim.dt * j,
                "end_pts": [
                    sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for s in sim.end_pts
                ],
                "sites": {
                    s: sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                    for c in sim.cable_sites for s in c
                },
                "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
                "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
            })

            new_extra_data.append({
                "time": sim.dt * j,
                "rest_lengths": sim.mjc_model.tendon_lengthspring[:, 0].tolist(),
                "controls": ctrls.flatten().tolist(),
            })

            sim.sim_step(controls=ctrls)

        sim.forward()
        new_data.append({
            "time": new_data[-1]['time'] + sim.dt,
            "end_pts": [
                sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                for s in sim.end_pts
            ],
            "sites": {
                s: sim.mjc_data.sensor(f"pos_{s}").data.tolist()
                for c in sim.cable_sites for s in c
            },
            "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
            "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:].flatten().tolist(),
            "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
            "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist(),
        })

        out = output_dir / data_dir.name
        out.mkdir(parents=True, exist_ok=True)
        with (out / 'processed_data.json').open('w') as fp:
            json.dump(new_data, fp)
        with (out / 'extra_state_data.json').open('w') as fp:
            json.dump(new_extra_data, fp)

        frames = []
        for d in new_data[::4]:
            pos = np.array(d['pos'], dtype=np.float64).reshape(-1, 3)
            quat = np.array(d['quat'], dtype=np.float64).reshape(-1, 4)
            pose = np.hstack([pos, quat]).flatten()
            sim.mjc_data.qpos = pose
            sim.forward()
            frame = sim.render_frame()

            frames.append(frame)

        sim.save_video(Path(out, 'gt_vid.mp4'), frames)
        del sim


def passive_six_bar(sim, output_dir, passive_type, num_steps):
    for _ in range(2000):
        sim.sim_step(np.zeros((1, sim.n_actuators)))

    output_dir.mkdir(exist_ok=True)

    z = 4 * random.random() + 3
    velx = (4 * random.random() + 2) * np.random.choice([-1, 1])
    vely = (4 * random.random() + 2) * np.random.choice([-1, 1])

    end_pts = sim.get_endpts()
    pos = sim.mjc_data.qpos.reshape(-1, 7).copy()
    pos[:, 2] = pos[:, 2] - end_pts[:, 2].min() + 0.175 + z
    sim.mjc_data.qpos = pos.flatten()

    if passive_type == 'throw':
        vel = sim.mjc_data.qvel.reshape(-1, 6).copy()
        vel[:, 0] = velx
        vel[:, 1] = vely
        sim.mjc_data.qvel = vel.flatten()

    extra_data = []
    processed_data = [{
        "time": 0.0,
        "end_pts": sim.get_endpts().tolist(),
        "sites": {s: sim.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                  for sp in sim.cable_sites for s in sp},
        "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
        "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:7].flatten().tolist(),
        "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
        "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist()
    }]

    for n in range(num_steps):
        c = [0.0 for _ in range(sim.n_actuators)]
        e_data = {
            "time": n * sim.dt,
            "dt": sim.dt,
            "rest_lengths": sim.mjc_model.tendon_lengthspring[:sim.n_actuators, 0].flatten().tolist(),
            "motor_speeds": [c.motor_state.omega_t.flatten().item() for c in sim.cable_motors],
            "controls": deepcopy(c)
        }

        sim.sim_step(np.array(c).reshape(1, -1))

        p_data = {
            "time": (n + 1) * sim.dt,
            "end_pts": sim.get_endpts().tolist(),
            "sites": {s: sim.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                      for sp in sim.cable_sites for s in sp},
            "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
            "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:7].flatten().tolist(),
            "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
            "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist()
        }

        processed_data.append(p_data)
        extra_data.append(e_data)

    with (output_dir / f'processed_data.json').open('w') as fp:
        json.dump(processed_data, fp)

    with (output_dir / f'extra_state_data.json').open('w') as fp:
        json.dump(extra_data, fp)

    frames = []
    for d in processed_data:
        pos = np.array(d['pos'], dtype=np.float64).reshape(-1, 3)
        quat = np.array(d['quat'], dtype=np.float64).reshape(-1, 4)
        pose = np.hstack([pos, quat]).flatten()
        frames.append({'time': d['time'], 'pos': pose})

    vis = MuJoCoVisualizer()
    vis.set_xml_path(sim.xml_path)
    vis.set_camera("camera")
    vis.data = frames[::4]
    vis.visualize(Path(output_dir, f"mjc_vid.mp4"), 0.01)

    del vis


def random_ctrls(sim,
                 ctrl_int,
                 num_steps,
                 output_dir,
                 low_lim=0.9,
                 up_lim=2.1):
    assert num_steps % ctrl_int == 0
    output_dir.mkdir(parents=True, exist_ok=True)

    nu = sim.n_actuators
    for _ in range(2000):
        sim.sim_step(np.zeros((1, nu)))

    num_ctrls = num_steps // ctrl_int

    extra_data = []
    processed_data = [{
        "time": 0.0,
        "end_pts": sim.get_endpts().tolist(),
        "sites": {s: sim.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                  for sp in sim.cable_sites for s in sp},
        "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
        "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:7].flatten().tolist(),
        "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
        "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist()
    }]

    for j in range(num_ctrls):
        s, m = sim.cable_motors[0].speed, sim.cable_motors[0].max_omega
        sm_inv = 1. / (s * m)
        r_w = sim.winch_r

        motor_speeds = np.hstack([c.motor_state.omega_t for c in sim.cable_motors])
        rest_lengths = sim.mjc_model.tendon_lengthspring[:nu, 0].reshape(1, -1)

        alpha = 2 * sm_inv / (sim.dt * r_w)
        lower = alpha * (rest_lengths - up_lim) - motor_speeds * sm_inv
        upper = alpha * (rest_lengths - low_lim) - motor_speeds * sm_inv
        lower = np.clip(lower, -1., 1. - 1e-2)
        upper = np.clip(upper, -1. + 1e-2, 1.)

        ctrl = np.random.uniform(lower, upper, (1, nu))

        for i in range(ctrl_int):
            e_data = {
                "time": (j * ctrl_int + i) * 0.01,
                "dt": 0.01,
                "rest_lengths": sim.mjc_model.tendon_lengthspring[:nu, 0].flatten().tolist(),
                "motor_speeds": [c.motor_state.omega_t.flatten().item() for c in sim.cable_motors],
                "controls": ctrl.tolist()
            }

            sim.sim_step(ctrl.copy())

            p_data = {
                "time": (j * ctrl_int + i + 1) * 0.01,
                "end_pts": sim.get_endpts().tolist(),
                "sites": {s: sim.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                          for sp in sim.cable_sites for s in sp},
                "pos": sim.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": sim.mjc_data.qpos.reshape(-1, 7)[:, 3:7].flatten().tolist(),
                "linvel": sim.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": sim.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist()
            }

            processed_data.append(p_data)
            extra_data.append(e_data)

    with (output_dir / f'processed_data.json').open('w') as fp:
        json.dump(processed_data, fp)

    with (output_dir / f'extra_state_data.json').open('w') as fp:
        json.dump(extra_data, fp)

    frames = []
    for d in processed_data:
        pos = np.array(d['pos'], dtype=np.float64).reshape(-1, 3)
        quat = np.array(d['quat'], dtype=np.float64).reshape(-1, 4)
        pose = np.hstack([pos, quat]).flatten()
        frames.append({'time': d['time'], 'pos': pose})

    vis = MuJoCoVisualizer()
    vis.set_xml_path(sim.xml_path)
    vis.set_camera("front")
    vis.data = frames[::4]
    vis.visualize(Path(output_dir, f"mjc_vid.mp4"), 0.01)

    del vis


def rerun(base_base_path, xml_path, save_new_data=False):
    for base_path in base_base_path.iterdir():
        if ('roll' not in base_path.name
                and 'ccw' not in base_path.name
                and 'cw' not in base_path.name
                and 'pdrop' not in base_path.name
                and 'pthrow' not in base_path.name
                and 'random' not in base_path.name
                and 'mppi' not in base_path.name):
            continue
        print()
        print(base_path.name)

        gt_data = json.load(Path(base_path, f"processed_data.json").open('r'))
        gt_extra_data = json.load(Path(base_path, f"extra_state_data.json").open('r'))
        # xml = Path("xml_models/6bar_new_platform.xml")
        # env_copy = SixBarTensegrityMuJoCoSimulator(xml_path, "six_bar_surface", False)
        env_copy = ThreeBarTensegrityMuJoCoSimulator(xml_path, attach_type='real_attach')
        controls = [d['controls'] for d in gt_extra_data[:-1]]

        init_pos_arr = np.array(gt_data[0]['pos'], dtype=np.float64).reshape(-1, 3)
        init_quat_arr = np.array(gt_data[0]['quat'], dtype=np.float64).reshape(-1, 4)
        init_linvel_arr = np.array(gt_data[0]['linvel'], dtype=np.float64).reshape(-1, 3)
        init_angvel_arr = np.array(gt_data[0]['angvel'], dtype=np.float64).reshape(-1, 3)

        end_pts = [np.array(d['end_pts'], dtype=np.float64) for d in gt_data]

        env_copy.mjc_data.qpos = np.hstack([init_pos_arr, init_quat_arr]).flatten()
        env_copy.mjc_data.qvel = np.hstack([init_linvel_arr, init_angvel_arr]).flatten()

        # init_rest = gt_extra_data[w]['rest_lengths']
        # init_mspeeds = gt_extra_data[w]['motor_speeds']

        # for j, cable in enumerate(env_copy.cable_motors):
        #     cable.motor_state.omega_t[:] = deepcopy(gt_extra_data[0]['motor_speeds'][j])
        #
        env_copy.mjc_model.tendon_lengthspring[:env_copy.n_actuators, 0] = deepcopy(gt_extra_data[0]['rest_lengths'])
        env_copy.mjc_model.tendon_lengthspring[:env_copy.n_actuators, 1] = deepcopy(gt_extra_data[0]['rest_lengths'])

        env_copy.forward()

        extra_data = []
        processed_data = [{
            "time": 0.0,
            "end_pts": env_copy.get_endpts().tolist(),
            "sites": {s: env_copy.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                      for sp in env_copy.cable_sites for s in sp},
            "pos": gt_data[0]['pos'],
            "quat": gt_data[0]['quat'],
            "linvel": gt_data[0]['linvel'],
            "angvel": gt_data[0]['angvel']
        }]

        for i in range(len(gt_extra_data) - 1):
            c = gt_extra_data[i]['controls']
            next_rest_lengths = np.array(gt_extra_data[i + 1]['rest_lengths'])
            e_data = {
                "time": i * 0.01,
                "dt": 0.01,
                "rest_lengths": env_copy.mjc_model.tendon_lengthspring[:env_copy.n_actuators, 0].copy().tolist(),
                "motor_speeds": [c.motor_state.omega_t.flatten().item() for c in env_copy.cable_motors],
                "controls": deepcopy(c)
            }

            env_copy.mjc_model.tendon_lengthspring[:env_copy.n_actuators, 0] = next_rest_lengths.copy()
            env_copy.mjc_model.tendon_lengthspring[:env_copy.n_actuators, 1] = next_rest_lengths.copy()

            env_copy.sim_step(np.zeros((1, env_copy.n_actuators)))
            # print(np.abs(env_copy.get_endpts() - np.array(gt_data[i + 1]['end_pts'])).max())
            p_data = {
                "time": (i + 1) * 0.01,
                "end_pts": env_copy.get_endpts().tolist(),
                "sites": {s: env_copy.mjc_data.sensor(f"pos_{s}").data.flatten().tolist()
                          for sp in env_copy.cable_sites for s in sp},
                "pos": env_copy.mjc_data.qpos.reshape(-1, 7)[:, :3].flatten().tolist(),
                "quat": env_copy.mjc_data.qpos.reshape(-1, 7)[:, 3:7].flatten().tolist(),
                "linvel": env_copy.mjc_data.qvel.reshape(-1, 6)[:, :3].flatten().tolist(),
                "angvel": env_copy.mjc_data.qvel.reshape(-1, 6)[:, 3:].flatten().tolist()
            }

            processed_data.append(p_data)
            extra_data.append(e_data)

        endpts_error = max([max([max([abs(a - b) for a, b in zip(p0['end_pts'][k], p1['end_pts'][k])]) for k in range(6)])
                   for p0, p1 in zip(gt_data, processed_data)])
        pos_error = max([max([abs(a - b) for a, b in zip(p0['pos'], p1['pos'])]) for p0, p1 in
                          zip(gt_data, processed_data)])
        quat_error = max([max([abs(a - b) for a, b in zip(p0['quat'], p1['quat'])]) for p0, p1 in
                           zip(gt_data, processed_data)])
        linvel_error = max([max([abs(a - b) for a, b in zip(p0['linvel'], p1['linvel'])]) for p0, p1 in
                             zip(gt_data, processed_data)])
        angvel_error = max([max([abs(a - b) for a, b in zip(p0['angvel'], p1['angvel'])]) for p0, p1 in
                             zip(gt_data, processed_data)])
        rest_lens_error = max([max([abs(a - b) for a, b in zip(p0['rest_lengths'], p1['rest_lengths'])]) for p0, p1 in
                   zip(gt_extra_data, extra_data)])
        ctrls_error = max([max([abs(a - b) for a, b in zip(p0['controls'], p1['controls'])]) for p0, p1 in
                               zip(gt_extra_data, extra_data)])

        max_error = max([endpts_error, pos_error, quat_error, linvel_error, angvel_error, rest_lens_error, ctrls_error])

        print('endpts', endpts_error)
        print('pos', pos_error)
        print('quat', quat_error)
        print('linvel', linvel_error)
        print('angvel', angvel_error)
        print('rest_lengths', rest_lens_error)
        print('controls', ctrls_error)

        if save_new_data:
            with (base_path / f'processed_data.json').open('w') as fp:
                json.dump(processed_data, fp)

            with (base_path / f'extra_state_data.json').open('w') as fp:
                json.dump(extra_data, fp)

    # frames = []
    # for i in range(poses.shape[0]):
    #     p = poses[i].flatten()
    #     gt_p = gt_pose[i].flatten()
    #     frames.append({'time': 0.01 * i, 'pos': np.concatenate([p, gt_p])})
    #
    # vis = MuJoCoVisualizer()
    # vis.set_xml_path(Path("mujoco_physics_engine/xml_models/3prism_real_upscaled_obs_course_w_gt.xml"))
    # vis.set_camera("camera")
    # vis.data = frames[::4]
    # vis.visualize(Path(base_path, f"mjc_vid.mp4"), 0.01)
