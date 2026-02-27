import json
from pathlib import Path
from typing import Dict

import cv2
import mujoco
import numpy
import numpy as np
import tqdm


class MuJoCoVisualizer:

    def __init__(self, render_fps: int = 50, render_size: (int, int) = (640, 640)):
        """

        @param render_fps:
        @param render_size:
        """
        self.mjc_model = None
        self.mjc_data = None
        self.renderer = None
        self.scene = None
        self.data = {}
        self.render_fps = render_fps
        self.render_size = render_size
        self.camera = "fixed"

    def set_camera(self, camera_name: str):
        self.camera = camera_name

    def set_xml_path(self, xml_path: Path):
        self.mjc_model = self._load_model_from_xml(xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, self.render_size[0], self.render_size[1])

        mujoco.mj_resetData(self.mjc_model, self.mjc_data)

    def _load_model_from_xml(self, xml_path: Path) -> mujoco.MjModel:
        model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        return model

    def load_data(self, data_path: Path):
        with data_path.open("r") as fp:
            self.data = json.load(fp)

    def visualize(self, save_video_path: Path, dt: float):
        frames = []
        num_steps_per_frame = int(1 / self.render_fps / dt)
        for i, data_step in tqdm.tqdm(enumerate(self.data)):
            # if True:
            if i % num_steps_per_frame == 0:
                frame = self.take_snap_shot(data_step['time'],
                                            data_step['pos'])

                frames.append(frame)
                # cv2.imwrite(Path(save_video_path, f"{i}.png").as_posix(), frame)

        self.save_video(save_video_path, frames)

    def visualize_from_ext_data(self,
                                xml_path: Path,
                                data_path: Path,
                                dt: float,
                                video_path: Path):
        self.mjc_model = self._load_model_from_xml(xml_path)
        self.mjc_data = mujoco.MjData(self.mjc_model)
        self.renderer = mujoco.Renderer(self.mjc_model, self.render_size[0], self.render_size[1])
        self.load_data(data_path)

        self.visualize(video_path, dt)

    def render_frame(self):
        self.renderer.update_scene(self.mjc_data, self.camera)
        frame = self.renderer.render().copy()
        return frame

    def save_video(self, save_path: Path, frames: list):
        frame_size = (self.renderer.width, self.renderer.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(save_path.as_posix(), fourcc, self.render_fps, frame_size)

        for i, frame in enumerate(frames):
            im = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(im)

        video_writer.release()

    def take_snap_shot(self, t: float = None, pos: np.array = None, camera_view: str = None):
        if t:
            self.mjc_data.time = t
        if pos is not None:
            self.mjc_data.qpos = pos

        mujoco.mj_forward(self.mjc_model, self.mjc_data)
        self.renderer.update_scene(self.mjc_data, camera_view if camera_view else self.camera)
        frame = self.renderer.render().copy()

        return frame

    def add_line_segment(self, scene, point1, point2, radius, rgba):
        """Adds one capsule to an mjvScene."""
        if scene.ngeom >= scene.maxgeom:
            print("Max scene geoms reached")
            return
        scene.ngeom += 1  # increment ngeom
        # initialise a new capsule, add it to the scene using mjv_connector
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                            np.zeros(3), np.zeros(9), rgba.astype(np.float32))
        mujoco.mjv_makeConnector(scene.geoms[scene.ngeom - 1],
                                 mujoco.mjtGeom.mjGEOM_CAPSULE,
                                 radius,
                                 point1[0], point1[1], 0.5,
                                 point2[0], point2[1], 0.5
                                 )

    def add_capsule(self, scene, pt, radius=0.3, rgba=np.ones(4)):
        if scene.ngeom >= scene.maxgeom:
            print("Max scene geoms reached")
            return
        scene.ngeom += 1  # increment ngeom
        # initialise a new capsule, add it to the scene using mjv_connector
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, radius, radius]),
                            pt, np.zeros(9), rgba.astype(np.float32)
                            )

    def add_path_to_scene(self, positions, radius=0.05, rgba=None):
        """Draw position trace, speed modifies width and colors."""
        if rgba is None:
            rgba = np.array([1.0, 0., 0., 1.0])

        for i in range(len(positions) - 1):
            pt1 = positions[i]
            pt2 = positions[i + 1]
            self.add_line_segment(self.renderer.scene, pt1, pt2, radius, rgba)


if __name__ == '__main__':
    for i in range(2, 5):
        xml_path = Path("xml/3prism_real_upscaled_vis.xml")
        base_path = Path("../../../tensegrity/data_sets/tensegrity_real_datasets/models/rss_demo_new_v1/"
                         )
        frames = json.load((base_path / f"patrick{i}_init_frames.json").open('r'))[::2]

        visualizer = MuJoCoVisualizer()
        visualizer.set_xml_path(Path(xml_path))
        visualizer.data = frames
        visualizer.set_camera("front")
        # visualizer.visualize(Path(model_path, f"{base_path.name}_vid_camera.mp4"), dt)
        # visualizer.set_camera("front")
        visualizer.visualize(Path(base_path, f"vid{i}.mp4"), 0.01)
