import sys
import os
import open3d as o3d
# pip install open3d==0.9.0.0 if using python2
import numpy as np
import cv2
from tqdm import tqdm  # pip install tqdm
from tqdm.contrib import tenumerate
# use opencv-contrib-python: 4.2.0.32(python2)/4.2.0.34(python3)  (These versions do not have Qthread issues.)
# pip uninstall opencv-python; pip install opencv-contrib-python==4.2.0.32

# ---------------------- change this------------------------
# dataset = os.path.expanduser('~/catkin_ws/src/tensegrity/data')
dataset = os.path.expanduser('~/catkin_ws/src/tensegrity/data')
video_folder = sys.argv[1]
# ---------------------- change this------------------------

prefixes = sorted([i.split('.')[0] for i in os.listdir(os.path.join(dataset, video_folder, 'color'))])
# im_w, im_h = 640, 480
# fx = 612.162
# fy = 612.253
# cx = 324.662
# cy = 241.931
# intr_o3d = o3d.camera.PinholeCameraIntrinsic(im_w, im_h, fx, fy, cx, cy)
# depth_scale = 1000
im_w, im_h = 1280, 720
fx = 902.514
fy = 902.589
cx = 648.303
cy = 372.909
intr_o3d = o3d.camera.PinholeCameraIntrinsic(im_w, im_h, fx, fy, cx, cy)
depth_scale = 4000

cv2.namedWindow('video')
visualizer = o3d.visualization.Visualizer()
# visualizer.create_window(width=640, height=480)
visualizer.create_window(width=1280, height=720)

for i, prefix in tenumerate(prefixes):
    color_path = os.path.join(dataset, video_folder, 'color', str(prefix) + '.png')
    depth_path = os.path.join(dataset, video_folder, 'depth', str(prefix) + '.png')

    color_im_bgr = cv2.imread(color_path)
    depth_im = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_im, alpha=0.3), cv2.COLORMAP_JET)

    # visualize point cloud for the first frame
    depth_im_o3d = o3d.geometry.Image(depth_im)
    color_im_o3d = o3d.geometry.Image(cv2.cvtColor(color_im_bgr, cv2.COLOR_BGR2RGB))
    rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_im_o3d, depth_im_o3d,
        depth_scale=depth_scale, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, intr_o3d)

    if i == 0:
        o3d.visualization.draw_geometries([pcd])

    # ================================= comment this part if you want to uncomment the line above==================
    visualizer.clear_geometries()
    visualizer.add_geometry(pcd)

    # Due to a potential bug in Open3D, cx, cy can only be w / 2 - 0.5, h / 2 - 0.5
    # https://github.com/intel-isl/Open3D/issues/1164
    cam_intr_vis = o3d.camera.PinholeCameraIntrinsic()
    cam_intr = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    cx_viz, cy_viz = im_w / 2 - 0.5, im_h / 2 - 0.5
    cam_intr_vis.set_intrinsics(im_w, im_h, fx, fy, cx_viz, cy_viz)
    cam_params = o3d.camera.PinholeCameraParameters()
    cam_params.intrinsic = cam_intr_vis
    cam_params.extrinsic = np.eye(4)

    visualizer.get_view_control().convert_from_pinhole_camera_parameters(cam_params)
    visualizer.poll_events()
    visualizer.update_renderer()
    # ==============================================================================================================

    combined_im = np.hstack([color_im_bgr, depth_colormap])
    cv2.putText(combined_im, prefix, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow('video', combined_im)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyWindow('video')
