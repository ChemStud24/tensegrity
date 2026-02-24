import os
import json
from pathlib import Path
from time import perf_counter

import numpy as np
import cv2
import open3d as o3d

from perception_utils import vis_pcd, create_pcd, plane_detection_o3d


def pick_points(pcd):
    print("1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


def height_1d_ransac(pts, max_iterations=100, inlier_thresh=0.005):
    num_pts = pts.shape[0]
    heights = pts[:, 2]
    max_num_inliers = 0
    height = 0
    for _ in range(max_iterations):
        selected_index = np.random.choice(num_pts, size=1, replace=False)
        selected_height = heights[selected_index].item()
        num_inliers = np.sum(np.abs(heights - selected_height) < inlier_thresh)
        if num_inliers > max_num_inliers:
            max_num_inliers = num_inliers
            height = selected_height
    max_inlier_ratio = max_num_inliers / num_pts
    return height, max_inlier_ratio


def main():
    dataset = Path("/home/willjohnson/catkin_ws/src/tensegrity/src/tensegrity_perception/dataset/2023-08-08_18-05-47").expanduser()
    with open(os.path.join(dataset, 'config.json'), 'r') as f:
        cam_info = json.load(f)
    cam_intr = np.array(cam_info['cam_intr'])
    cam_extr = cam_info['cam_extr']
    depth_scale = cam_info['depth_scale']
    depth_trunc = cam_info['depth_trunc']

    prefix = 0#361
    color_path = dataset / 'color' / f'{prefix:04d}.png'
    depth_path = dataset / 'depth' / f'{prefix:04d}.png'

    color_im = cv2.cvtColor(cv2.imread(str(color_path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth_im = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED).astype(np.float32) / depth_scale
    pcd = create_pcd(depth_im, cam_intr, color_im, depth_trunc=depth_trunc)
    vis_pcd(pcd)
    # cam_extr, _ = plane_detection_o3d(pcd, inlier_thresh=0.003, max_iterations=1000, visualize=True)
    cam_extr = json.load(open('../configs/data_cfg.json','r'))['cam_extr']

    pcd.transform(np.linalg.inv(cam_extr))
    vis_pcd(pcd)

    # test hsv values of the bar
    # pt_indices = pick_points(pcd)
    # normalized_rgb_values = np.asarray(pcd.colors)[pt_indices]
    # rgb_values = (normalized_rgb_values * 255).astype(np.uint8)
    # hsv_values = cv2.cvtColor(rgb_values[None], cv2.COLOR_RGB2HSV)
    # print(f'{hsv_values = }')  # (23, 180, 200)

    pcd_colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
    hsv_values = cv2.cvtColor(pcd_colors[None], cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_values, (20, 100, 100), (25, 255, 255))[0]
    valid_indices = np.where(mask == 255)[0]
    bar_pcd = pcd.select_by_index(valid_indices)
    vis_pcd(bar_pcd)

    tic = perf_counter()
    height, inlier_ratio = height_1d_ransac(np.asarray(bar_pcd.points), max_iterations=300)
    toc = perf_counter()
    print(f'{height = }, {inlier_ratio = }')
    print(f'Computing bar height takes {toc - tic}s.')

    # visualization
    plane = o3d.geometry.AxisAlignedBoundingBox()
    plane.max_bound = (0.2, 0.2, 0.005)
    plane.min_bound = (-0.2, -0.2, -0.005)
    translation = np.array([0, 0, height], dtype=np.float32)
    plane.translate(translation)
    plane.color = (1, 0, 0)
    vis_pcd([pcd, plane])


if __name__ == "__main__":
    main()
