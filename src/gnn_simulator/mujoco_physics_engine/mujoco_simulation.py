import json
import random
import threading
import asyncio
import time
from pathlib import Path
from typing import List, Optional

import cv2
import mujoco
import numpy as np
from gnn_simulator.mujoco_visualizer_utils.mujoco_visualizer import RemoteViewer


class AbstractMuJoCoSimulator:
    """
    MuJoCo spring rod simulator
    """

    def __init__(self,
                 xml_path: Path | str,
                 visualize: bool = False,
                 render_size: (int, int) = (1280, 1280),
                 render_fps: int = 100,
                 use_remote_viewer: bool = False,
                 remote_port: int = 8765,
                 remote_max_fps: int = None,
                 overlay_callback=None):
        self.xml_path = Path(xml_path)
        self.visualize = visualize
        self.mjc_model = self._load_model_from_xml(self.xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, render_size[0], render_size[1]) if visualize else None
        self.render_fps = render_fps
        self.states = []
        self.time = 0
        self.dt = self.mjc_model.opt.timestep

        # Remote viewer
        self.remote_viewer = None
        if use_remote_viewer:
            self.remote_viewer = RemoteViewer(
                self.mjc_model, self.mjc_data, port=remote_port,
                overlay_callback=overlay_callback, max_fps=remote_max_fps
            )
            print(f"[RemoteViewer] Open http://localhost:{remote_port} in your browser")

    def reset(self):
        self.mjc_model = self._load_model_from_xml(self.xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)

    def _load_model_from_xml(self, xml_path: Path) -> mujoco.MjModel:
        model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        return model

    def sim_step(self):
        # self.mjc_data.ctrl = [-500, -500]
        # self.mjc_model.tendon_lengthspring = np.maximum(self.mjc_model.tendon_lengthspring - 0.01, 0.0)
        mujoco.mj_step(self.mjc_model, self.mjc_data)
        self.sync_remote_viewer()
        # k=1

    def forward(self):
        mujoco.mj_forward(self.mjc_model, self.mjc_data)

    def render_frame(self, view='camera'):
        self.renderer.update_scene(self.mjc_data, view)
        frame = self.renderer.render().copy()
        return frame

    def render_frames_from_poses(self, poses, view='camera'):
        curr_pose = self.mjc_model.qpos().copy()
        frames = []
        for pose in poses:
            self.mjc_data.qpos = pose
            self.forward()
            frame = self.render_frame(view)
            frames.append(frame)

        self.mjc_data.qpos = curr_pose
        self.forward()
        return frames

    def sync_remote_viewer(self):
        """Sync and render the remote viewer if active."""
        if self.remote_viewer is not None and self.remote_viewer.is_running():
            self.remote_viewer.sync()
            self.remote_viewer.render()

    def run(self,
            end_time: float = None,
            num_steps: int = None,
            save_path: Path = None,
            pos_sensor_names: Optional[List] = None,
            quat_sensor_names: Optional[List] = None,
            linvel_sensor_names: Optional[List] = None,
            angvel_sensor_names: Optional[List] = None):

        if end_time is None and num_steps is None:
            raise Exception("Need to specify one of time params (end_time or num_steps)")
        elif end_time:
            num_steps = np.ceil(end_time / self.dt).astype(int)

        frames = []
        num_steps_per_frame = int(1 / self.render_fps / self.dt)

        poses, end_pts = [], []
        for i in range(num_steps + 1):
            mujoco.mj_forward(self.mjc_model, self.mjc_data)
            # self.mjc_data.ctrl = 0.1
            # if self.visualize and ((i + 1) % num_steps_per_frame == 0 or i == num_steps - 1):
            #     frame = self.render_frame()
            #     frames.append(frame.copy())

            if i % 100:
                print(f"Timestep: {self.mjc_data.time}")

            pos = self.mjc_data.qpos.tolist()
            # end_pt = np.hstack([self.mjc_data.sensor("pos_s0").data.copy(), self.mjc_data.sensor("pos_s1").data.copy()])
            poses.append(pos.copy())
            # end_pts.append(end_pt)
            vel = self.mjc_data.qvel.tolist()
            self.states.append({"time": round(self.mjc_data.time, 5),
                                "pos": pos,
                                "vel": vel,
                                "r01_end_pt1": self.mjc_data.sensor("pos_s0").data.tolist(),
                                "r01_end_pt2": self.mjc_data.sensor("pos_s1").data.tolist(),
            #                     "r23_end_pt1": self.mjc_data.sensor("pos_s2").data.tolist(),
            #                     "r23_end_pt2": self.mjc_data.sensor("pos_s3").data.tolist(),
            #                     "r45_end_pt1": self.mjc_data.sensor("pos_s4").data.tolist(),
            #                     "r45_end_pt2": self.mjc_data.sensor("pos_s5").data.tolist(),
            #                     "s6": self.mjc_data.sensor("pos_s6").data.tolist(),
            #                     "s7": self.mjc_data.sensor("pos_s7").data.tolist(),
            #                     "s8": self.mjc_data.sensor("pos_s8").data.tolist(),
            #                     "s9": self.mjc_data.sensor("pos_s9").data.tolist(),
            #                     "s10": self.mjc_data.sensor("pos_s10").data.tolist(),
            #                     "s11": self.mjc_data.sensor("pos_s11").data.tolist(),
            #                     "s12": self.mjc_data.sensor("pos_s12").data.tolist(),
            #                     "s13": self.mjc_data.sensor("pos_s13").data.tolist(),
            #                     "s14": self.mjc_data.sensor("pos_s14").data.tolist(),
            #                     "s15": self.mjc_data.sensor("pos_s15").data.tolist(),
            #                     "s16": self.mjc_data.sensor("pos_s16").data.tolist(),
            #                     "s17": self.mjc_data.sensor("pos_s17").data.tolist(),
            #                     "s18": self.mjc_data.sensor("pos_s18").data.tolist(),
            #                     "s19": self.mjc_data.sensor("pos_s19").data.tolist(),
            #                     "s20": self.mjc_data.sensor("pos_s20").data.tolist(),
            #                     "s21": self.mjc_data.sensor("pos_s21").data.tolist(),
            #                     "s22": self.mjc_data.sensor("pos_s22").data.tolist(),
            #                     "s23": self.mjc_data.sensor("pos_s23").data.tolist(),
            #                     # "rod_3_end_pt1": self.mjc_data.sensor("pos_s6").data.tolist(),
            #                     # "rod_3_end_pt2": self.mjc_data.sensor("pos_s7").data.tolist(),
            #                     # "rod_4_end_pt1": self.mjc_data.sensor("pos_s8").data.tolist(),
            #                     # "rod_4_end_pt2": self.mjc_data.sensor("pos_s9").data.tolist(),
            #                     # "rod_5_end_pt1": self.mjc_data.sensor("pos_s10").data.tolist(),
            #                     # "rod_5_end_pt2": self.mjc_data.sensor("pos_s11").data.tolist()
                                })
            self.sim_step()

        if save_path:
            save_path = Path(save_path)
            save_path.mkdir(exist_ok=True)

            with Path(save_path, "data.json").open("w") as fp:
                json.dump(self.states, fp)

            if self.visualize:
                self.save_video(save_path / "video.mp4", frames)

        self.states.clear()
        return end_pts, poses

    def save_video(self, save_path: Path, frames: list):
        frame_size = (self.renderer.width, self.renderer.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(save_path.as_posix(), fourcc, self.render_fps, frame_size)

        for i, frame in enumerate(frames):
            im = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # cv2.imwrite(f"tmp/frame{i}.png", frame)
            video_writer.write(im)

        video_writer.release()


if __name__ == '__main__':
    import shutil

    base_path = Path('/Users/nelsonchen/Documents/Rutgers-CS-PhD/Research/tensegrity/data_sets/')
    exp_dir = Path('/Users/nelsonchen/Desktop/tmp_data/')
    xml_path = Path("xml_models/single_rod_spring.xml")
    exp_dir.mkdir(exist_ok=True)

    sim = AbstractMuJoCoSimulator(xml_path, visualize=True)
    sim.run(num_steps=400, save_path=exp_dir)
