import os
import json
import open3d as o3d
import numpy as np
import sys
import cv2


def vis_pcd(pcd, cam_pose=None, coord_frame_size=0.2):
    if not isinstance(pcd, list):
        pcd = [pcd]
    pcd_frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=coord_frame_size)
    if cam_pose is not None:
        cam_frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=coord_frame_size)
        cam_frame.transform(cam_pose)
        o3d.visualization.draw_geometries([*pcd, pcd_frame, cam_frame])
    else:
        o3d.visualization.draw_geometries([*pcd, pcd_frame])


def create_pcd(depth_im: np.ndarray,
               cam_intr: np.ndarray,
               color_im: np.ndarray = None,
               cam_extr: np.ndarray = np.eye(4)):
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic()
    intrinsic_o3d.intrinsic_matrix = cam_intr
    depth_im_o3d = o3d.geometry.Image(depth_im)
    if color_im is not None:
        color_im_o3d = o3d.geometry.Image(color_im)
        rgbd = o3d.geometry.RGBDImage().create_from_color_and_depth(color_im_o3d, depth_im_o3d, depth_scale=1,
                                                                    convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud().create_from_rgbd_image(rgbd, intrinsic_o3d, extrinsic=cam_extr)
    else:
        pcd = o3d.geometry.PointCloud().create_from_depth_image(depth_im_o3d, intrinsic_o3d, extrinsic=cam_extr,
                                                                depth_scale=1)
    return pcd


def main():
    dataset = "/home/willjohnson/catkin_ws/src/tensegrity/data/" + sys.argv[1]
    calibration_path = "/home/willjohnson/catkin_ws/src/tensegrity_perception/configs/"
    with open(os.path.join(calibration_path, 'data_cfg.json'), 'r') as f:
        cam_info = json.load(f)
    cam_intr = cam_info['cam_intr']
    cam_extr = cam_info['cam_extr']
    depth_scale = cam_info['depth_scale']

    prefix = 0
    color_path = os.path.join(dataset, 'color', f'{prefix:04d}.png')
    depth_path = os.path.join(dataset, 'depth', f'{prefix:04d}.png')

    color_im = cv2.cvtColor(cv2.imread(color_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth_im = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32) / depth_scale

    print(f"visualize the camera frame")
    pcd = create_pcd(depth_im, cam_intr, color_im)
    vis_pcd(pcd)

    print(f"visualize the world frame")
    pcd.transform(cam_extr)
    vis_pcd(pcd)


if __name__ == "__main__":
    main()
