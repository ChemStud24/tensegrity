#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
import datetime
import json
import copy
import pprint
from collections import defaultdict
from typing import Dict

import numpy as np
import numpy.linalg as la
import cv2
import networkx as nx
import open3d as o3d
from scipy.optimize import minimize
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import trimesh
from trimesh.sample import sample_surface_even
import pyrender
from pyrender.constants import RenderFlags
from easydict import EasyDict

from perception_utils import create_pcd, plane_detection_ransac, plane_detection_o3d

# ROS library
import rospy
import rospkg
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
# =============================== Untested ==================================
from tensegrity.msg import SensorsStamped, TensegrityStamped, PoseStateStamped
# ===========================================================================
from tensegrity.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity.srv import GetPose, GetPoseRequest, GetPoseResponse
from tensegrity.srv import GetBarHeight, GetBarHeightRequest, GetBarHeightResponse

from numba import njit, prange


class Tracker:
    ColorDict = {
        "red": [255, 0, 0],
        "green": [0, 255, 0],
        "blue": [0, 0, 255]
    }

    def __init__(self, cfg, data_cfg, rod_pcd=None, rod_mesh=None, endcap_meshes=None):
        self.cfg = cfg
        self.data_cfg = data_cfg
        pprint.pprint(self.data_cfg)

        self.endcap_meshes = endcap_meshes
        self.rod_pcd = copy.deepcopy(rod_pcd)
        self.rod_mesh = rod_mesh
        self.rod_length = self.data_cfg['rod_length']

        H, W = self.data_cfg['im_h'], self.data_cfg['im_w']
        self.obs_vis = np.empty((self.data_cfg['im_h'], self.data_cfg['im_w'], 3), dtype=np.uint8)

        # set up pyrender scene
        self.renderer = pyrender.OffscreenRenderer(W // self.cfg.render_scale, H // self.cfg.render_scale)
        cam_intr = np.asarray(self.data_cfg['cam_intr'])
        fx = cam_intr[0, 0] / self.cfg.render_scale
        fy = cam_intr[1, 1] / self.cfg.render_scale
        cx = cam_intr[0, 2] / self.cfg.render_scale
        cy = cam_intr[1, 2] / self.cfg.render_scale
        camera = pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=cx, cy=cy)
        light = pyrender.SpotLight(color=np.ones(3), intensity=3.0, innerConeAngle=np.pi / 16.0)
        camera_node = pyrender.Node(name='cam', camera=camera, matrix=np.eye(4))
        light_node = pyrender.Node(name='light', light=light, matrix=np.eye(4))
        self.render_scene = pyrender.Scene(nodes=[camera_node, light_node])

        # initialize graph
        self.G = nx.Graph()
        self.G.graph['rods'] = [(u, v) for (u, v) in self.data_cfg['color_to_rod'].values()]
        self.G.graph['tendons'] = [(u, v) for (u, v) in self.data_cfg['sensor_to_tendon'].values()]

        # add end cap (marker) node to graph
        for node, color in enumerate(self.data_cfg['node_to_color']):
            self.G.add_node(node)
            self.G.nodes[node]['color'] = color
            self.G.nodes[node]['pos_list'] = []

        # add tendon edge to graph
        for sensor_id, (u, v) in self.data_cfg['sensor_to_tendon'].items():
            self.G.add_edge(u, v)
            self.G.edges[u, v]['sensor_id'] = sensor_id
            self.G.edges[u, v]['type'] = 'tendon'

        # add rod edge to graph
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            self.G.add_edge(u, v)
            self.G.edges[u, v]['type'] = 'rod'
            self.G.edges[u, v]['length'] = self.data_cfg['rod_length']
            self.G.edges[u, v]['color'] = color
            self.G.edges[u, v]['pose_list'] = []

        # ros service
        self.init_tracker_srv_name = 'init_tracker'
        self.get_pose_srv_name = 'get_pose'
        self.get_bar_height_srv_name = 'get_bar_height'
        self.init_tracker_srv = rospy.Service(self.init_tracker_srv_name, InitTracker, self.initialize_tracker)
        self.get_pose_srv = rospy.Service(self.get_pose_srv_name, GetPose, self.get_current_state)
        self.get_bar_height_srv = rospy.Service(self.get_bar_height_srv_name, GetBarHeight, self.get_bar_height)

        # ros subscriber
        # =============================== Untested ==================================
        self.rgb_topic = "/rgb_images"
        self.depth_topic = "/depth_images"
        self.strain_topic = "/control_msg"
        color_im_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_im_sub = message_filters.Subscriber(self.depth_topic, Image)
        strain_sub = message_filters.Subscriber(self.strain_topic, TensegrityStamped)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_im_sub, depth_im_sub, strain_sub], queue_size=10, slop=0.1)
        self.time_synchronizer.registerCallback(self.tracking_callback)
        self.count = 0
        self.prev_timestamp = 0
        # ros publisher
        # self.perception_topic = "/tracking_images"
        self.trajectory_topic = "/trajectory_images"
        # self.perception_pub = rospy.Publisher(self.perception_topic, Image, queue_size=10)
        self.trajectory_pub = rospy.Publisher(self.trajectory_topic, Image, queue_size=10)
        self.pose_topic = '/pose_msg'
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStateStamped, queue_size=10)
        self.latest_bar_height = 0

        # saving data
        data_path = os.path.join(rospkg.RosPack().get_path('tensegrity'), 'data/')
        self.output_dir = data_path + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.color_dir = os.path.join(self.output_dir, 'color')
        self.depth_dir = os.path.join(self.output_dir, 'depth')
        self.track_dir = os.path.join(self.output_dir, 'tracking')
        self.data_dir = os.path.join(self.output_dir, 'data')
        os.mkdir(self.output_dir)
        os.mkdir(self.color_dir)
        os.mkdir(self.depth_dir)
        os.mkdir(self.track_dir)
        os.mkdir(self.data_dir)
        # ===========================================================================

        self.bridge = CvBridge()
        self.current_state = None
        self.rgb_im = None
        self.depth_im = None
        self.initialized = False

    def get_current_state(self, req: GetPoseRequest):
        # may have data racing issue, not sure how to solve
        rospy.loginfo(f"Received '{self.get_pose_srv_name}' service request.")
        response = GetPoseResponse()
        try:
            for color in self.data_cfg['end_cap_colors']:
                u, v = self.data_cfg['color_to_rod'][color]
                T = self.G.edges[u, v]['pose_list'][-1].copy()
                T = np.matmul(self.data_cfg['cam_extr'],T)
                R = T[:3, :3]
                t = T[:3, 3]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = t
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
                    Rotation.from_matrix(R).as_quat()  # x,y,z,w
                response.poses.append(pose)
        except IndexError:
            response.success = False
            return response
        response.success = True

        # # visualize tracking
        # cv2.imshow('Tracking Result',self.get_2d_vis_fast())
        # cv2.waitKey(500)

        return response

    @staticmethod
    def height_1d_ransac(pts, max_iterations=100, inlier_thresh=0.005):
        num_pts = pts.shape[0]
        heights = pts[:, 2]
        max_num_inliers = 0
        height = 0
        for _ in range(max_iterations):
            selected_index = np.random.choice(num_pts, size=1, replace=False)
            selected_height = heights[selected_index]
            num_inliers = np.sum(np.abs(heights - selected_height) < inlier_thresh)
            if num_inliers > max_num_inliers:
                max_num_inliers = num_inliers
                height = selected_height
        max_inlier_ratio = max_num_inliers / num_pts
        return height, max_inlier_ratio

    def get_bar_height(self, req: GetBarHeightRequest):
        rospy.loginfo(f"Received '{self.get_bar_height_srv_name}' service request.")
        response = GetBarHeightResponse()
        try:
            response.success = True
            cam_intr = np.asarray(self.data_cfg['cam_intr'])
            pcd = create_pcd(self.depth_im, cam_intr, self.rgb_im, depth_trunc=self.data_cfg['depth_trunc'])
            # pcd.transform(np.linalg.inv(self.data_cfg['cam_extr']))  # cam frame to world frame
            pcd.transform(np.linalg.inv(self.data_cfg['extr_for_height']))  # cam frame to world frame

            # test hsv values of the bar
            # pt_indices = self.pick_points(pcd)
            # normalized_rgb_values = np.asarray(pcd.colors)[pt_indices]
            # rgb_values = (normalized_rgb_values * 255).astype(np.uint8)
            # hsv_values = cv2.cvtColor(rgb_values[None], cv2.COLOR_RGB2HSV)
            # print(f'{hsv_values = }')  # [(23, 180, 200), (23, 200, 200), ...]

            pcd_colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
            hsv_values = cv2.cvtColor(pcd_colors[None], cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_values, (20, 100, 100), (25, 255, 255))[0]
            valid_indices = np.where(mask == 255)[0]
            bar_pcd = pcd.select_by_index(valid_indices)

            tic = time.perf_counter()
            height, inlier_ratio = self.height_1d_ransac(np.asarray(bar_pcd.points), 300, 0.005)
            toc = time.perf_counter()
            print(f'{height = }, {inlier_ratio = }')
            print(f'Computing bar height takes {toc - tic}s.')
            response.height = height
            self.latest_bar_height = height
            """
            # get height of highest node
            endcaps = []
            for color in self.data_cfg['end_cap_colors']:
                u, v = self.data_cfg['color_to_rod'][color]
                T = self.G.edges[u, v]['pose_list'][-1].copy()
                T = np.matmul(self.data_cfg['extr_for_height'],T)
                R = T[:3, :3]
                t = T[:3, 3]
                unit_vector = R[:,2]
                endcaps.append(t + self.data_cfg['rod_length']/2*unit_vector)
                endcaps.append(t - self.data_cfg['rod_length']/2*unit_vector)
            endcap_heights = [endcap[2] for endcap in endcaps]
            response.highest_node = max(endcap_heights)
            """
        except IndexError:
            response.success = False
            return response
        return response

    def initialize_tracker(self, req: InitTrackerRequest):
        rospy.loginfo(f"Received '{self.init_tracker_srv_name}' service request.")
        rgb_im = self.bridge.imgmsg_to_cv2(req.rgb_im, 'rgb8')
        depth_im = self.bridge.imgmsg_to_cv2(req.depth_im, 'mono16').astype(np.float32) / self.data_cfg['depth_scale']
        hsv_im = cv2.cvtColor(rgb_im, cv2.COLOR_RGB2HSV)

        self.rgb_im = rgb_im
        self.depth_im = depth_im

        info = defaultdict(dict)
        for i, length in enumerate(req.cable_lengths.data):
            info['sensors'][str(i)] = {'length': length}

        cam_intr = np.asarray(self.data_cfg['cam_intr'])
        fx, fy, cx, cy = cam_intr[0, 0], cam_intr[1, 1], cam_intr[0, 2], cam_intr[1, 2]

        scene_pcd = create_pcd(depth_im, cam_intr, rgb_im, depth_trunc=self.data_cfg['depth_trunc'])
        if 'cam_extr' not in self.data_cfg:
            plane_frame, _ = plane_detection_ransac(scene_pcd, inlier_thresh=0.003, visualize=self.cfg.visualize)
            self.data_cfg['cam_extr'] = np.round(plane_frame, decimals=3)

        self.data_cfg['extr_for_height'], _ = plane_detection_o3d(scene_pcd, inlier_thresh=0.003, max_iterations=1000, visualize=False)
        print(self.data_cfg['extr_for_height'])

        if 'init_endcap_pos' not in self.data_cfg:
            self.data_cfg['init_endcap_pos'] = dict()
            print(f"\nPick one point for each endcap. Need {len(self.G.nodes)} in total.")
            pt_indices = self.pick_points(scene_pcd)
            pts = np.asarray(scene_pcd.points)[pt_indices]
            normalized_rgb_values = np.asarray(scene_pcd.colors)[pt_indices]
            for endcap_id, (pos, rgb) in enumerate(zip(pts, normalized_rgb_values)):
                self.data_cfg['init_endcap_pos'][str(endcap_id)] = pos.tolist()

                # check whether this point is in the HSV range of the endcap color
                endcap_color = self.G.nodes[endcap_id]['color']
                lower_bound, upper_bound = self.data_cfg['hsv_ranges'][endcap_color]
                x = int(np.round(pos[0] * fx / pos[2] + cx))
                y = int(np.round(pos[1] * fy / pos[2] + cy))
                hsv = hsv_im[y, x]
                if np.any(hsv < np.asarray(lower_bound)) or np.any(hsv > np.asarray(upper_bound)):
                    self.G.nodes[endcap_id]['confidence'] = 0
                    print(f"The HSV value {hsv} of {endcap_id = } is not within the HSV threshold. "
                          f"HSV {lower_bound = }, {upper_bound = }"
                          f"Either a random point is picked or the HSV threshold has to be updated.")
                else:
                    self.G.nodes[endcap_id]['confidence'] = 1

        # -------------------- optimize once --------------------
        for _, (u, v) in self.data_cfg['color_to_rod'].items():
            self.G.nodes[u]['pos_list'].append(self.data_cfg['init_endcap_pos'][str(u)])
            self.G.nodes[v]['pos_list'].append(self.data_cfg['init_endcap_pos'][str(v)])

        add_physical_constraints = self.cfg.add_physical_constraints
        self.cfg.add_physical_constraints = False
        self.constrained_optimization(info, sanity_check=False)
        self.cfg.add_physical_constraints = add_physical_constraints
        # -------------------------------------------------------

        for _, (u, v) in self.data_cfg['color_to_rod'].items():
            self.data_cfg['init_endcap_pos'][str(u)] = self.G.nodes[u]['pos_list'][0].tolist()
            self.data_cfg['init_endcap_pos'][str(v)] = self.G.nodes[v]['pos_list'][0].tolist()

        # --- DEBUG: print robot location in world frame after initialization ---
        H = np.array(self.data_cfg['cam_extr'])  # camera → world
        rod_centers_world = []
        print("\n[DEBUG] Robot pose after initialization (world frame, meters):")
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            p_u_cam = np.array(self.G.nodes[u]['pos_list'][0])
            p_v_cam = np.array(self.G.nodes[v]['pos_list'][0])
            p_u_world = (H @ np.append(p_u_cam, 1.0))[:3]
            p_v_world = (H @ np.append(p_v_cam, 1.0))[:3]
            rod_center_world = (p_u_world + p_v_world) / 2.0
            rod_centers_world.append(rod_center_world)
            print(f"  Rod '{color}':  endcap[{u}] = ({p_u_world[0]*1000:.1f}, {p_u_world[1]*1000:.1f}, {p_u_world[2]*1000:.1f}) mm"
                  f"  |  endcap[{v}] = ({p_v_world[0]*1000:.1f}, {p_v_world[1]*1000:.1f}, {p_v_world[2]*1000:.1f}) mm"
                  f"  |  center = ({rod_center_world[0]*1000:.1f}, {rod_center_world[1]*1000:.1f}, {rod_center_world[2]*1000:.1f}) mm")
        com_world = np.mean(rod_centers_world, axis=0)
        print(f"  Overall COM   = ({com_world[0]*1000:.1f}, {com_world[1]*1000:.1f}, {com_world[2]*1000:.1f}) mm")
        print(f"  Overall COM (x, y only) = ({com_world[0]*1000:.1f}, {com_world[1]*1000:.1f}) mm\n")
        # ----------------------------------------------------------------------

        if self.cfg.visualize:
            pcd = self.get_3d_vis()
            frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.1)
            o3d.visualization.draw_geometries([pcd, frame])
            # o3d.visualization.draw_geometries_with_editing([pcd])

        # save trajectory information
        self.trajectory = []
        H = np.array(self.data_cfg['cam_extr'])
        E = np.zeros((4,4))
        E[:3,:3] = H[:3,:3].T
        E[:3,3:] = -np.matmul(H[:3,:3].T,H[:3,3:])
        E[3,3] = 1
        cam_intr = np.array(self.data_cfg['cam_intr'])
        fx, fy = cam_intr[0, 0], cam_intr[1, 1]
        cx, cy = cam_intr[0, 2], cam_intr[1, 2]
        for i in range(len(req.trajectory_x.data)):
            this_point = np.array([[req.trajectory_x.data[i]],[req.trajectory_y.data[i]],[0],[1]])
            XYZ = np.matmul(E,this_point)
            x = int(np.round((XYZ[0] * fx / XYZ[2]) + cx))
            y = int(np.round((XYZ[1] * fy / XYZ[2]) + cy))
            self.trajectory.append([x,y])

        self.initialized = True
        response = InitTrackerResponse()
        response.success = True
        return response
    
    @staticmethod
    def pick_points(pcd):
        print("1) Please pick at least three correspondences using [shift + left click]")
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        # # visualize the world frame
        # world_frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.2)
        # world_frame.transform(self.data_cfg['cam_extr'])
        # o3d.visualization.draw_geometries([pcd, world_frame])
        # #####
        vis.run()  # user picks points
        vis.destroy_window()
        return vis.get_picked_points()

    def compute_obs_pts(self, depth_im, mask):
        fx, fy = self.data_cfg['cam_intr'][0][0], self.data_cfg['cam_intr'][1][1]
        cx, cy = self.data_cfg['cam_intr'][0][2], self.data_cfg['cam_intr'][1][2]

        ys, xs = np.nonzero((depth_im > 0) & mask)
        pts = np.empty([len(ys), 3], dtype=np.float32)
        pts[:, 2] = depth_im[ys, xs]
        pts[:, 0] = (xs - cx) / fx * pts[:, 2]
        pts[:, 1] = (ys - cy) / fy * pts[:, 2]
        return pts

    def filter_obs_pts(self, obs_pts, color, radius=0.1, thresh=0.01):
        u, v = self.data_cfg['color_to_rod'][color]
        u_pos = self.G.nodes[u]['pos_list'][-1]
        u_dist_sqr = np.sum((obs_pts - u_pos) ** 2, axis=1)
        v_pos = self.G.nodes[v]['pos_list'][-1]
        v_dist_sqr = np.sum((obs_pts - v_pos) ** 2, axis=1)

        u_mask = u_dist_sqr < v_dist_sqr
        u_pts = obs_pts[u_mask & (u_dist_sqr < radius ** 2)]
        v_pts = obs_pts[(~u_mask) & (v_dist_sqr < radius ** 2)]

        # filter points based on depth
        if u_pts.shape[0] > 0:
            zs = u_pts[:, 2]
            z_min = np.percentile(zs, q=10)
            u_pts = u_pts[zs < z_min + thresh]

        if v_pts.shape[0] > 0:
            zs = v_pts[:, 2]
            z_min = np.percentile(zs, q=10)
            v_pts = v_pts[zs < z_min + thresh]

        return u_pts, v_pts

    def tracking_callback(self, rgb_msg, depth_msg, strain_msg):
        # print("I entered a new callback")
        # self.count += 1
        
        # this_timestamp = rgb_msg.header.stamp.to_sec()
        # # skip frames to run real-time at 10 Hz
        # if this_timestamp - self.prev_timestamp < 0.1:
        #     print("Ignoring tracking frame " + str(self.count))
        #     return
        # self.prev_timestamp = this_timestamp
    
        print("Processing tracking frame " + str(self.count))

        if not self.initialized:
            print('Tracking service not initialized yet')
            return

        # print("Unpacking data...")
        rgb_im = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        depth_im = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16').astype(np.float32) / self.data_cfg['depth_scale']
        info = defaultdict(dict)
        for sensor in strain_msg.sensors:
            info['sensors'][str(sensor.id)] = {'length': sensor.length, 'capacitance': sensor.capacitance}

        self.rgb_im = rgb_im
        self.depth_im = depth_im

        # print("Getting observed points...")
        # get observed 3D points for both endcaps of each rod
        obs_pts_dict = dict()
        model_pcd_dict = dict()
        self.obs_vis = np.zeros_like(self.obs_vis)
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            color_im_hsv = cv2.cvtColor(rgb_im, cv2.COLOR_RGB2HSV)
            hsv_mask = cv2.inRange(
                color_im_hsv,
                lowerb=tuple(self.data_cfg['hsv_ranges'][color][0]),
                upperb=tuple(self.data_cfg['hsv_ranges'][color][1])
            ).astype(np.bool8)
            # if color == 'red':
            #     hsv_mask |= cv2.inRange(color_im_hsv, lowerb=(0, 100, 100), upperb=(3, 255, 255)).astype(np.bool8)
            obs_pts = self.compute_obs_pts(depth_im, hsv_mask)
            obs_pts_dict[color] = obs_pts

            prev_pose = np.copy(self.G.edges[u, v]['pose_list'][-1])
            for i, node in enumerate([u, v]):
                model_pcd = copy.deepcopy(self.endcap_meshes[i])
                model_pcd.transform(prev_pose)
                _, pt_map = model_pcd.hidden_point_removal([0, 0, 0], radius=0.001)
                model_pcd = model_pcd.select_by_index(pt_map, invert=True)
                model_pcd_dict[node] = model_pcd

            self.G.edges[u, v]['pose_list'].append(prev_pose)
            self.G.nodes[u]['pos_list'].append(prev_pose[:3, 3] + prev_pose[:3, 2] * self.rod_length / 2)
            self.G.nodes[v]['pos_list'].append(prev_pose[:3, 3] - prev_pose[:3, 2] * self.rod_length / 2)

            if color == 'red':
                self.obs_vis[hsv_mask, 0] = 255
            elif color == 'green':
                self.obs_vis[hsv_mask, 1] = 255
            elif color == 'blue':
                self.obs_vis[hsv_mask, 2] = 255

        # print("Beginning iterative optimization...")
        for iteration, max_distance in enumerate(self.cfg.max_correspondence_distances):
            # print('Iteration ' + str(iteration))
            prev_poses = {}
            for color, (u, v) in self.data_cfg['color_to_rod'].items():
                prev_poses[color] = np.copy(self.G.edges[u, v]['pose_list'][-1])

            # ================================== prediction step ==================================
            tic = time.time()
            for color, (u, v) in self.data_cfg['color_to_rod'].items():
                u_rendered_pts = np.asarray(model_pcd_dict[u].points)
                v_rendered_pts = np.asarray(model_pcd_dict[v].points)

                obs_pts = obs_pts_dict[color]

                # filter observed points based on z coordinates
                if self.cfg.filter_observed_pts:
                    u_obs_pts, v_obs_pts = self.filter_obs_pts(obs_pts, color, radius=max_distance, thresh=0.02)
                else:
                    u_obs_pts, v_obs_pts = self.filter_obs_pts(obs_pts, color, radius=max_distance, thresh=np.inf)

                # add dummy points at the previous endcap position
                if self.cfg.num_dummy_points > 0:
                    u_dummy = self.G.nodes[u]['pos_list'][-1]
                    v_dummy = self.G.nodes[v]['pos_list'][-1]

                    obs_pts = np.vstack([u_obs_pts, v_obs_pts, u_dummy, v_dummy])
                    obs_w = np.ones(obs_pts.shape[0])
                    obs_w[-2:] = self.cfg.dummy_weights * self.cfg.num_dummy_points

                    rendered_pts = np.vstack([u_rendered_pts, v_rendered_pts, u_dummy, v_dummy])
                    rendered_w = np.ones(rendered_pts.shape[0])
                    rendered_w[-2:] = self.cfg.dummy_weights * self.cfg.num_dummy_points

                delta_T = self.register(obs_pts, rendered_pts, obs_w, rendered_w, max_distance)
                rod_pose = delta_T @ prev_poses[color]

                self.G.edges[u, v]['pose_list'][-1] = rod_pose
                self.G.nodes[u]['pos_list'][-1] = rod_pose[:3, 3] + rod_pose[:3, 2] * self.rod_length / 2
                self.G.nodes[v]['pos_list'][-1] = rod_pose[:3, 3] - rod_pose[:3, 2] * self.rod_length / 2

                if self.cfg.use_adaptive_weights:
                    self.G.nodes[u]['confidence'] = self.compute_confidence(u_obs_pts, u_rendered_pts,
                                                                            inlier_thresh=max_distance)
                    self.G.nodes[v]['confidence'] = self.compute_confidence(v_obs_pts, v_rendered_pts,
                                                                            inlier_thresh=max_distance)
                else:
                    self.G.nodes[u]['confidence'] = 1
                    self.G.nodes[v]['confidence'] = 1

            # print(f"point registration takes {time.time() - tic}s")

            # ================================== correction step ==================================
            if self.cfg.optimize_every_n_iters != 0 and iteration % self.cfg.optimize_every_n_iters == 0:
                tic = time.time()
                self.constrained_optimization(info, sanity_check=True)
                # print(f"optimization takes {time.time() - tic}s")

            for color, (u, v) in self.data_cfg['color_to_rod'].items():
                prev_pose = prev_poses[color]
                curr_pose = self.G.edges[u, v]['pose_list'][-1]
                delta_T = curr_pose @ la.inv(prev_pose)
                model_pcd_dict[u].transform(delta_T)
                model_pcd_dict[v].transform(delta_T)

        # This could be slow if your GPU is not powerful since it requires image rendering
        # vis_im = self.get_2d_vis()
        # vis_im_bgr = cv2.cvtColor(vis_im, cv2.COLOR_RGB2BGR)
        # cv2.imshow("estimation", vis_im_bgr)
        # key = cv2.waitKey(1)
        # if key == ord('q'):
        #     exit(0)

        # visualize tracking
        tracking_result_im = self.get_2d_vis_fast()
        # tracking_msg = self.bridge.cv2_to_imgmsg(tracking_result_im,'rgb8')
        # tracking_msg.header.stamp = rgb_msg.header.stamp
        # self.perception_pub.publish(tracking_msg)

        # visualize the trajectory on the camera's image
        # traj_im = self.rgb_im
        traj_im = tracking_result_im
        
        H = np.array(self.data_cfg['cam_extr'])
        E = np.zeros((4,4))
        E[:3,:3] = H[:3,:3].T
        E[:3,3:] = -np.matmul(H[:3,:3].T,H[:3,3:])
        E[3,3] = 1
        cam_intr = np.array(self.data_cfg['cam_intr'])
        fx, fy = cam_intr[0, 0], cam_intr[1, 1]
        cx, cy = cam_intr[0, 2], cam_intr[1, 2]
        """
        for point in strain_msg.trajectory.trajectory:
            this_point = np.array([[point.x],[point.y],[0],[1]])
            XYZ = np.matmul(E,this_point)
            # UVW = np.matmul(cam_intr,XYZ[:3,:])
            # for i in range(XYZ.shape[0]):
            x = int(np.round((XYZ[0] * fx / XYZ[2]) + cx))
            y = int(np.round((XYZ[1] * fy / XYZ[2]) + cy))
            # x = np.round(UVW[0]/UVW[2])
            # y = np.round(UVW[1]/UVW[2])
            # print(x,y)
        """

        for x,y in self.trajectory:
            traj_im = cv2.circle(traj_im, (x,y), radius=2, color=(0, 255, 255), thickness=-1)
            # print(x,y,z)

        prev_x,prev_y = None,None
        for point in strain_msg.trajectory.COMs:
            this_point = np.array([[point.x],[point.y],[0],[1]])
            XYZ = np.matmul(E,this_point)
            x = int(np.round((XYZ[0] * fx / XYZ[2]) + cx))
            y = int(np.round((XYZ[1] * fy / XYZ[2]) + cy))
            traj_im = cv2.circle(traj_im, (x,y), radius=5, color=(0,0,0), thickness=-1)
            if not prev_x is None:
                traj_im = cv2.line(traj_im, (prev_x,prev_y), (x,y), color=(0,0,0), thickness=1)
            prev_x, prev_y = x,y

        # for i in range(len(strain_msg.trajectory.PAs)):
        #     x,y = strain_msg.trajectory.COMs[i].x,strain_msg.trajectory.COMs[i].y
        #     com_point = np.array([[x],[y],[0],[1]])
        #     tip_point = np.array([[x + strain_msg.trajectory.PAs[i].x*0.15],[y + strain_msg.trajectory.PAs[i].y*0.15],[0],[1]])
        #     XYZcom = np.matmul(E,com_point)
        #     XYZtip = np.matmul(E,tip_point)
        #     x = int(np.round((XYZcom[0] * fx / XYZcom[2]) + cx))
        #     y = int(np.round((XYZcom[1] * fy / XYZcom[2]) + cy))
        #     a = int(np.round((XYZtip[0] * fx / XYZtip[2]) + cx))
        #     b = int(np.round((XYZtip[1] * fy / XYZtip[2]) + cy))
        #     traj_im = cv2.line(traj_im, (x,y), (a,b), color=(230,56+70*i,224), thickness=3)

        trajectory_im_msg = self.bridge.cv2_to_imgmsg(traj_im,'rgb8')
        trajectory_im_msg.header.stamp = rgb_msg.header.stamp
        self.trajectory_pub.publish(trajectory_im_msg)

        # publish PoseStateStamped message
        pose_state_msg = PoseStateStamped()
        pose_state_msg.header.stamp = rgb_msg.header.stamp
        pose_state_msg.header.frame_id = rgb_msg.header.frame_id if hasattr(rgb_msg.header, 'frame_id') else ''
        
        # Extract poses for each rod (similar to get_current_state)
        try:
            endcaps = []
            for color in self.data_cfg['end_cap_colors']:
                u, v = self.data_cfg['color_to_rod'][color]
                T = self.G.edges[u, v]['pose_list'][-1].copy()
                T = np.matmul(self.data_cfg['cam_extr'], T)
                R = T[:3, :3]
                t = T[:3, 3]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = t
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
                    Rotation.from_matrix(R).as_quat()  # x,y,z,w
                pose_state_msg.poses.append(pose)

                unit_vector = R[:,2]
                endcaps.append(t + self.data_cfg['rod_length']/2*unit_vector)
                endcaps.append(t - self.data_cfg['rod_length']/2*unit_vector)
        except (IndexError, KeyError) as e:
            rospy.logwarn(f"Failed to extract poses for PoseStateStamped: {e}")
            # Continue without poses if extraction fails
        
        # Extract encoder_counts and motor_speeds from strain_msg
        encoder_lengths = []
        motor_speeds = []
        for motor in strain_msg.motors:
            encoder_lengths.append(float(motor.encoder_length))
            motor_speeds.append(float(motor.speed))
        
        pose_state_msg.encoder_lengths = encoder_lengths
        pose_state_msg.motor_speeds = motor_speeds
        
        self.pose_pub.publish(pose_state_msg)


        # # get height of highest node
        # endcaps = []
        # for color in self.data_cfg['end_cap_colors']:
        #     u, v = self.data_cfg['color_to_rod'][color]
        #     T = self.G.edges[u, v]['pose_list'][-1].copy()
        #     T = np.matmul(self.data_cfg['extr_for_height'],T)
        #     R = T[:3, :3]
        #     t = T[:3, 3]
        #     unit_vector = R[:,2]
        #     endcaps.append(t + self.data_cfg['rod_length']/2*unit_vector)
        #     endcaps.append(t - self.data_cfg['rod_length']/2*unit_vector)
        # endcap_heights = [endcap[2] for endcap in endcaps]
        # highest_node = max(endcap_heights)
        # print(highest_node)

        # save the data
        cv2.imwrite(os.path.join(self.color_dir, str(self.count).zfill(4) + ".png"), cv2.cvtColor(self.rgb_im,cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(self.depth_dir, str(self.count).zfill(4) + ".png"), self.bridge.imgmsg_to_cv2(depth_msg, 'mono16'))
        cv2.imwrite(os.path.join(self.track_dir, str(self.count).zfill(4) + ".png"), cv2.cvtColor(traj_im,cv2.COLOR_RGB2BGR))
        data = {}
        data['header'] = {'seq':strain_msg.header.seq,'secs':strain_msg.header.stamp.to_sec()}
        data['info'] = {'min_length':strain_msg.info.min_length,'RANGE':strain_msg.info.RANGE,'RANGE024':strain_msg.info.RANGE024,'RANGE135':strain_msg.info.RANGE135,'MAX_RANGE':strain_msg.info.MAX_RANGE,'MIN_RANGE':strain_msg.info.MIN_RANGE,'max_speed':strain_msg.info.max_speed,'tol':strain_msg.info.tol,'low_tol':strain_msg.info.low_tol,'P':strain_msg.info.P,'I':strain_msg.info.I,'D':strain_msg.info.D,'dist_weight':strain_msg.info.dist_weight,'ang_weight':strain_msg.info.ang_weight,'prog_weight':strain_msg.info.prog_weight}
        data['motors'] = {}
        for motor in strain_msg.motors:
            data['motors'][motor.id] = {'target':motor.target,'position':motor.position,'speed':motor.speed,'done':motor.done,'error':motor.error,'d_error':motor.d_error,'cum_error':motor.cum_error,'encoder_counts':motor.encoder_counts,'encoder_length':motor.encoder_length, 'absolute_encoder_length':motor.absolute_encoder_length}
        data['sensors'] = {}
        for sensor in strain_msg.sensors:
            data['sensors'][sensor.id] = {'length':sensor.length,'capacitance':sensor.capacitance}
        data['imu'] = {}
        for imu in strain_msg.imus:
            # data['imu'][imu.id] = {'ax':imu.ax,'ay':imu.ay,'az':imu.az,'gx':imu.gx,'gy':imu.gy,'gz':imu.gz,'mx':imu.mx,'my':imu.my,'mz':imu.mz}
            data['imu'][imu.id] = {'acceleromter':{'x':imu.ax,'y':imu.ay,'z':imu.az},'gyroscope':{'x':imu.gx,'y':imu.gy,'z':imu.gz}}#{'x':imu.x,'y':imu.y,'z':imu.z}#,'q1':imu.q1,'q2':imu.q2,'q3':imu.q3,'q4':imu.q4}#,'yaw':imu_msg.yaw, 'pitch':imu_msg.pitch,'roll':imu_msg.roll}
        data['trajectory'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.trajectory)}
        data['COM'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.COMs)}
        data['PA'] = {i:{'x':point.x,'y':point.y} for i,point in enumerate(strain_msg.trajectory.PAs)}
        data['action'] = {str(i):act for i,act in enumerate(strain_msg.actions)}
        data['endcaps'] = {i:{'x':end[0],'y':end[1],'z':end[2]} for i,end in enumerate(endcaps)}
        data['segment'] = strain_msg.trajectory.trajectory_segment
        # data['robot-height'] = highest_node
        # data['bar-height'] = float(self.latest_bar_height)
        json.dump(data,open(os.path.join(self.data_dir, str(self.count).zfill(4) + ".json"),'w'))
        self.count += 1

        # cv2.imshow('Tracking Result',tracking_result_bgr)
        # print('showing result')
        # cv2.waitKey(1)
        # print('I reached the end of the callback')
        return

    @staticmethod
    def compute_confidence(obs_pts, rendered_pts, inlier_thresh=0.01, min_node_confidence=0.1):
        if len(obs_pts) == 0 or len(rendered_pts) == 0:
            return min_node_confidence

        o_tree = KDTree(obs_pts)
        r_tree = KDTree(rendered_pts)
        r2o_dist, _ = o_tree.query(rendered_pts, k=1, distance_upper_bound=inlier_thresh)
        o2r_dist, _ = r_tree.query(obs_pts, k=1, distance_upper_bound=inlier_thresh)

        numerator = np.sum(o2r_dist < inlier_thresh) + np.sum(r2o_dist < inlier_thresh)
        confidence = numerator / (len(o2r_dist) + len(r2o_dist))  # f1 score
        return max(confidence, min_node_confidence)

    def constrained_optimization(self, info, sanity_check=True):
        rod_length = self.data_cfg['rod_length']
        num_endcaps = 2 * self.data_cfg['num_rods']  # a rod has two end caps

        obj_func, jac_func = self.objective_function_generator(info)

        init_values = np.zeros(3 * num_endcaps)
        for i in range(num_endcaps):
            init_values[(3 * i):(3 * i + 3)] = self.G.nodes[i]['pos_list'][-1]

        # be VERY CAREFUL when generating a lambda function in a for loop
        # https://stackoverflow.com/questions/45491376/
        # https://docs.python-guide.org/writing/gotchas/#late-binding-closures
        constraints = []
        for _, (u, v) in self.data_cfg['color_to_rod'].items():
            constraint = dict()
            constraint['type'] = 'eq'
            constraint['fun'], constraint['jac'] = self.endcap_constraint_generator(u, v, rod_length)
            constraints.append(constraint)

        if self.cfg.add_ground_constraints:
            for u, color in enumerate(self.data_cfg['node_to_color']):
                # if color == 'blue':
                #     continue
                constraint = dict()
                constraint['type'] = 'ineq'
                constraint['fun'], constraint['jac'] = self.ground_constraint_generator(u)
                constraints.append(constraint)

        if self.cfg.add_physical_constraints:
            rods = list(self.data_cfg['color_to_rod'].values())
            for i, (u, v) in enumerate(rods):
                p1 = self.G.nodes[u]['pos_list'][-1]
                p2 = self.G.nodes[v]['pos_list'][-1]
                for p, q in rods[i + 1:]:
                    p3 = self.G.nodes[p]['pos_list'][-1]
                    p4 = self.G.nodes[q]['pos_list'][-1]

                    alpha1, alpha2 = self.compute_closest_pair_of_points(p1, p2, p3, p4)
                    if alpha1 < 0 or alpha1 > 1 or alpha2 < 0 or alpha2 > 1:
                        continue

                    constraint = dict()
                    constraint['type'] = 'ineq'
                    constraint['fun'], constraint['jac'] = self.rod_constraint_generator(u, v, p, q, alpha1, alpha2)
                    constraints.append(constraint)

        # noinspection PyTypeChecker
        res = minimize(obj_func, init_values, jac=jac_func, method='SLSQP', constraints=constraints)
        if not res.success:
            print("Joint optimization fail! Something must be wrong.")

        for i in range(num_endcaps):
            if sanity_check and la.norm(res.x[(3 * i):(3 * i + 3)] - self.G.nodes[i]['pos_list'][-1]) > 0.1:
                print("Endcap position sanity check failed.")
            else:
                self.G.nodes[i]['pos_list'][-1] = res.x[(3 * i):(3 * i + 3)].copy()

        for u, v in self.data_cfg['color_to_rod'].values():
            if self.G.edges[u, v]['pose_list']:
                prev_rod_pose = self.G.edges[u, v]['pose_list'][-1]
            else:
                prev_rod_pose = None
            u_pos = self.G.nodes[u]['pos_list'][-1]
            v_pos = self.G.nodes[v]['pos_list'][-1]
            curr_endcap_centers = np.vstack([u_pos, v_pos])
            optimized_pose = self.estimate_rod_pose_from_endcap_centers(curr_endcap_centers, prev_rod_pose)
            if self.G.edges[u, v]['pose_list']:
                self.G.edges[u, v]['pose_list'][-1] = optimized_pose
            else:
                self.G.edges[u, v]['pose_list'].append(optimized_pose)

    def ground_constraint_generator(self, u):
        cam_to_world = la.inv(self.data_cfg['cam_extr'])
        R = cam_to_world[:3, :3]
        t = cam_to_world[:3, 3]

        def constraint_function(X):
            u_pos = X[3 * u: 3 * u + 3]
            u_world_pos = R @ u_pos + t
            u_z = u_world_pos[2]
            return u_z - self.data_cfg['rod_diameter'] / 2 + 0.01

        def jacobian_function(X):
            result = np.zeros_like(X)
            result[3 * u: 3 * u + 3] = R[2]
            return result

        return constraint_function, jacobian_function

    def objective_function_generator(self, info):
        sensor_measurement = info['sensors']

        def objective_function(X):
            unary_loss = 0
            for i in range(len(self.data_cfg['node_to_color'])):
                pos_est = X[(3 * i):(3 * i + 3)]
                pos_old = self.G.nodes[i]['pos_list'][-1]
                confidence = self.G.nodes[i].get('confidence', 1)
                unary_loss += confidence * np.sum((pos_est - pos_old) ** 2)

            binary_loss = 0
            for sensor_id, (u, v) in self.data_cfg['sensor_to_tendon'].items():
                sensor_id = str(sensor_id)
                # print(list(sensor_measurement.keys()))
                # print(type(list(sensor_measurement.keys())[0]))
                # print(list(self.data_cfg["sensor_status"].keys()))
                # print(type(list(self.data_cfg["sensor_status"].keys())[0]))
                if not self.data_cfg["sensor_status"][sensor_id] or sensor_measurement[sensor_id]['length'] <= 0:
                    continue

                factor = 0.2
                c_u = self.G.nodes[u].get('confidence', 1)
                c_v = self.G.nodes[v].get('confidence', 1)
                if c_u > 0.5 and c_v > 0.5:
                    factor = 0
                elif c_u > 0.2 and c_v > 0.2:
                    factor *= (1 - 0.5 * (c_u + c_v))

                u_pos = X[(3 * u):(3 * u + 3)]
                v_pos = X[(3 * v):(3 * v + 3)]
                estimated_length = la.norm(u_pos - v_pos)
                measured_length = sensor_measurement[sensor_id]['length'] / self.data_cfg['cable_length_scale']

                # sanity check
                if np.abs(estimated_length - measured_length) > 0.1:
                    factor = 0

                binary_loss += factor * (estimated_length - measured_length) ** 2

            return unary_loss + binary_loss

        def jacobian_function(X):
            result = np.zeros_like(X)

            for i in range(len(self.data_cfg['node_to_color'])):
                pos_est = X[3 * i: 3 * i + 3]
                pos_old = self.G.nodes[i]['pos_list'][-1]
                confidence = self.G.nodes[i].get('confidence', 1)
                result[3 * i: 3 * i + 3] = 2 * confidence * (pos_est - pos_old)

            for sensor_id, (u, v) in self.data_cfg['sensor_to_tendon'].items():
                sensor_id = str(sensor_id)
                if not self.data_cfg['sensor_status'][sensor_id] or sensor_measurement[sensor_id]['length'] <= 0:
                    continue

                factor = 0.2
                c_u = self.G.nodes[u].get('confidence', 1)
                c_v = self.G.nodes[v].get('confidence', 1)
                if c_u > 0.5 and c_v > 0.5:
                    factor = 0
                elif c_u > 0.2 and c_v > 0.2:
                    factor *= (1 - 0.5 * (c_u + c_v))

                u_pos = X[(3 * u):(3 * u + 3)]
                v_pos = X[(3 * v):(3 * v + 3)]
                l_est = la.norm(u_pos - v_pos)
                l_gt = sensor_measurement[sensor_id]['length'] / self.data_cfg['cable_length_scale']

                # sanity check
                if np.abs(l_est - l_gt) > 0.1:
                    factor = 0

                result[3 * u: 3 * u + 3] += 2 * factor * (l_est - l_gt) * (u_pos - v_pos) / (l_est + 1e-12)
                result[3 * v: 3 * v + 3] += 2 * factor * (l_est - l_gt) * (v_pos - u_pos) / (l_est + 1e-12)

            return result

        return objective_function, jacobian_function

    @staticmethod
    def endcap_constraint_generator(u, v, rod_length):

        def constraint_function(X):
            u_pos = X[3 * u: 3 * u + 3]
            v_pos = X[3 * v: 3 * v + 3]
            return la.norm(u_pos - v_pos) - rod_length

        def jacobian_function(X):
            u_pos = X[3 * u: 3 * u + 3]
            v_pos = X[3 * v: 3 * v + 3]
            C = la.norm(u_pos - v_pos) + 1e-12
            result = np.zeros_like(X)
            result[3 * u: 3 * u + 3] = (u_pos - v_pos) / C
            result[3 * v: 3 * v + 3] = (v_pos - u_pos) / C
            return result

        return constraint_function, jacobian_function

    def rod_constraint_generator(self, u, v, p, q, alpha1, alpha2):
        try:
            p1 = self.G.nodes[u]['pos_list'][-2]
            p2 = self.G.nodes[v]['pos_list'][-2]
            p3 = self.G.nodes[p]['pos_list'][-2]
            p4 = self.G.nodes[q]['pos_list'][-2]
        except IndexError:
            p1 = self.G.nodes[u]['pos_list'][-1]
            p2 = self.G.nodes[v]['pos_list'][-1]
            p3 = self.G.nodes[p]['pos_list'][-1]
            p4 = self.G.nodes[q]['pos_list'][-1]
        p5 = alpha1 * p1 + (1 - alpha1) * p2  # closest point on (u, v)
        p6 = alpha2 * p3 + (1 - alpha2) * p4  # closest point on (p, q)
        v1 = p5 - p6
        v1 /= (la.norm(v1) + 1e-12)

        def constraint_function(X):
            q1 = X[3 * u: 3 * u + 3]
            q2 = X[3 * v: 3 * v + 3]
            q3 = X[3 * p: 3 * p + 3]
            q4 = X[3 * q: 3 * q + 3]
            q5 = alpha1 * q1 + (1 - alpha1) * q2  # closest point on (u, v)
            q6 = alpha2 * q3 + (1 - alpha2) * q4  # closest point on (p, q)
            v2 = q5 - q6
            return v1 @ v2 - self.data_cfg['rod_diameter']

        def jacobian_function(X):
            result = np.zeros_like(X)
            result[3 * u: 3 * u + 3] = alpha1 * v1
            result[3 * v: 3 * v + 3] = (1 - alpha1) * v1
            result[3 * p: 3 * p + 3] = -alpha2 * v1
            result[3 * q: 3 * q + 3] = -(1 - alpha2) * v1
            return result

        return constraint_function, jacobian_function

    @staticmethod
    def estimate_rod_pose_from_endcap_centers(curr_endcap_centers, prev_rod_pose=None):
        curr_rod_pos = (curr_endcap_centers[0] + curr_endcap_centers[1]) / 2
        curr_z_dir = curr_endcap_centers[0] - curr_endcap_centers[1]
        curr_z_dir /= la.norm(curr_z_dir)

        if prev_rod_pose is None:
            prev_rod_pose = np.eye(4)

        prev_rot = prev_rod_pose[:3, :3]
        prev_z_dir = prev_rot[:, 2]

        delta_rot = np.eye(3)
        cos_dist = prev_z_dir @ curr_z_dir
        if not np.allclose(cos_dist, 1):
            axis = np.cross(prev_z_dir, curr_z_dir)
            axis = axis / la.norm(axis)
            angle = np.arccos(cos_dist)
            delta_rot = Rotation.from_rotvec(angle * axis).as_matrix()

        curr_rod_pose = np.eye(4)
        curr_rod_pose[:3, :3] = delta_rot @ prev_rot
        curr_rod_pose[:3, 3] = curr_rod_pos
        return curr_rod_pose

    @staticmethod
    def register(obs_pts, rendered_pts, obs_w=None, rendered_w=None, max_distance=0.02):
        if obs_w is None:
            obs_w = np.ones(obs_pts.shape[0])
        if rendered_w is None:
            rendered_w = np.ones(rendered_pts.shape[0])

        # ICP ref: https://www.youtube.com/watch?v=djnd502836w&t=781s
        r_tree = KDTree(rendered_pts)
        closest_distance, closest_indices = r_tree.query(obs_pts, k=1, distance_upper_bound=max_distance)
        mask = (closest_distance != np.inf)
        Q1 = obs_pts[mask]
        P1 = rendered_pts[closest_indices[mask]]
        W1 = (1 - (closest_distance[mask] / max_distance) ** 2) ** 2 * obs_w[mask] * rendered_w[closest_indices[mask]]

        o_tree = KDTree(obs_pts)
        closest_distance, closest_indices = o_tree.query(rendered_pts, k=1, distance_upper_bound=max_distance)
        mask = (closest_distance != np.inf)
        Q2 = obs_pts[closest_indices[mask]]
        P2 = rendered_pts[mask]
        W2 = (1 - (closest_distance[mask] / max_distance) ** 2) ** 2 * obs_w[closest_indices[mask]] * rendered_w[mask]

        Q = np.vstack([Q1, Q2])
        P = np.vstack([P1, P2])
        W = np.hstack([W1, W2])

        P_mean = P.mean(0)
        Q_mean = Q.mean(0)

        # https://github.com/scipy/scipy/blob/main/scipy/spatial/transform/_rotation.pyx#L2367
        H = (W[:, None] * (Q - Q_mean)).T @ (P - P_mean)
        U, _, V_t = la.svd(H)

        # ensure that R is in the right-hand coordinate system, very important!!!
        # https://en.wikipedia.org/wiki/Kabsch_algorithm
        d = np.sign(la.det(U @ V_t))
        U[:, -1] = d * U[:, -1]
        R = U @ V_t
        t = Q_mean - R @ P_mean
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    # https://math.stackexchange.com/questions/1993953/closest-points-between-two-lines
    @staticmethod
    def compute_closest_pair_of_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray):
        """compute the closest point pairs on two rods (p1, p2) and (p3, p4)

        Args:
            p1 (np.ndarray): start point of rod 1
            p2 (np.ndarray): end point of rod 1
            p3 (np.ndarray): start point of rod 2
            p4 (np.ndarray): end point of rod 2
        """
        v1 = p2 - p1
        l1 = la.norm(v1)
        v1 /= l1

        v2 = p4 - p3
        l2 = la.norm(v2)
        v2 /= l2

        v3 = np.cross(v2, v1)
        rhs = p3 - p1
        lhs = np.array([v1, -v2, v3]).T
        t1, t2, _ = la.solve(lhs, rhs)
        alpha1 = 1 - t1 / l1
        alpha2 = 1 - t2 / l2
        return alpha1, alpha2

    @staticmethod
    def create_scene_node(name, mesh, pose):
        m = np.array([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ])
        return pyrender.Node(name=name, mesh=mesh, matrix=m @ pose)

    def render_nodes(self, seg_node_map: Dict[pyrender.Node, int], depth_only: bool = False):
        for node in seg_node_map:
            self.render_scene.add_node(node)

        scale = self.cfg.render_scale
        if depth_only:
            rendered_depth = self.renderer.render(self.render_scene, flags=RenderFlags.DEPTH_ONLY)
            for node in seg_node_map:
                self.render_scene.remove_node(node)
            if scale != 1:
                H, W = rendered_depth.shape
                rendered_depth = cv2.resize(rendered_depth, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
            return rendered_depth

        rendered_seg, rendered_depth = self.renderer.render(self.render_scene, flags=RenderFlags.SEG,
                                                            seg_node_map=seg_node_map)
        rendered_seg = np.copy(rendered_seg[:, :, 0])
        for node in seg_node_map:
            self.render_scene.remove_node(node)
        if scale != 1:
            H, W = rendered_depth.shape
            rendered_depth = cv2.resize(rendered_depth, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
            rendered_seg = cv2.resize(rendered_seg, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
        return rendered_seg, rendered_depth

    def render_current_state(self):
        mapping = {'red': 1, 'green': 2, 'blue': 3}
        seg_node_map = dict()
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            pose = self.G.edges[u, v]['pose_list'][-1]
            mesh_node = self.create_scene_node(f"{color}-mash", self.rod_mesh, pose=pose)
            seg_node_map[mesh_node] = mapping[color]
        return self.render_nodes(seg_node_map)

    def get_2d_vis(self):
        rendered_seg, rendered_depth = self.render_current_state()
        H, W = self.data_cfg['im_h'], self.data_cfg['im_w']
        vis_im1 = np.copy(self.rgb_im)
        vis_im2 = np.zeros_like(self.rgb_im)
        for i, color in enumerate(['red', 'green', 'blue']):
            mask = rendered_depth.copy()
            mask[rendered_seg != i + 1] = 0
            vis_im1[self.depth_im < mask] = Tracker.ColorDict[color]
            vis_im2[mask > 0] = Tracker.ColorDict[color]
        vis_im = np.empty((H * 2, W * 2, 3), dtype=np.uint8)
        vis_im[:H, :W] = self.rgb_im
        vis_im[H:, :W] = self.obs_vis
        vis_im[:H, W:] = vis_im1
        vis_im[H:, W:] = vis_im2
        return vis_im

    def get_3d_vis(self):
        robot_pcd = o3d.geometry.PointCloud()
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            rod_pcd = copy.deepcopy(self.rod_pcd)
            rod_pcd = rod_pcd.paint_uniform_color(np.array(Tracker.ColorDict[color]) / 255)
            rod_pose = np.copy(self.G.edges[u, v]['pose_list'][-1])
            rod_pcd.transform(rod_pose)
            robot_pcd += rod_pcd

        scene_pcd = create_pcd(self.depth_im, self.data_cfg['cam_intr'], self.rgb_im,
                               depth_trunc=self.data_cfg['depth_trunc'])
        robot_pts = np.asarray(robot_pcd.points)
        rospy.loginfo(f"[get_3d_vis] robot_pcd z-range: {robot_pts[:,2].min():.3f} to {robot_pts[:,2].max():.3f} m, "
                      f"extent: {robot_pts.max(0) - robot_pts.min(0)}")
        scene_pts = np.asarray(scene_pcd.points)
        if len(scene_pts) > 0:
            rospy.loginfo(f"[get_3d_vis] scene_pcd z-range: {scene_pts[:,2].min():.3f} to {scene_pts[:,2].max():.3f} m, "
                          f"num_pts: {len(scene_pts)}")
        else:
            rospy.logwarn("[get_3d_vis] scene_pcd is EMPTY after depth_trunc crop")
        bbox = o3d.geometry.AxisAlignedBoundingBox()
        bbox.min_bound = np.min(robot_pts, axis=0) - 0.05
        bbox.max_bound = np.max(robot_pts, axis=0) + 0.05
        scene_pcd = scene_pcd.crop(bbox)
        return robot_pcd + scene_pcd

    @staticmethod
    def run():
        rospy.loginfo("Tracking service started but not initialized yet.")
        rospy.loginfo("Waiting for requests to initialize.")
        rospy.spin()

    @njit
    def project_xyz_to_color(color_im, depth_im, xyz, cam_intr, color):
        im_h, im_w = depth_im.shape
        fx, fy = cam_intr[0, 0], cam_intr[1, 1]
        cx, cy = cam_intr[0, 2], cam_intr[1, 2]

        for i in prange(xyz.shape[0]):
            x = int(np.round((xyz[i, 0] * fx / xyz[i, 2]) + cx))
            y = int(np.round((xyz[i, 1] * fy / xyz[i, 2]) + cy))
            if x < 0 or x >= im_w or y < 0 or y >= im_h:
                continue
            if (depth_im[y, x] == 0) or (depth_im[y, x] > xyz[i, 2]):
                color_im[y, x] = color

    def render_current_state_fast(self):
        vis_im_overlapping = np.copy(self.rgb_im)
        vis_im_endcap_only = np.zeros_like(self.rgb_im)
        cam_intr = np.asarray(self.data_cfg['cam_intr'])
        mapping = {'red': np.array([255, 0, 0]), 'green': np.array([0, 255, 0]), 'blue': np.array([0, 0, 255])}
        for color, (u, v) in self.data_cfg['color_to_rod'].items():
            pose = self.G.edges[u, v]['pose_list'][-1]
            R = pose[:3, :3]
            t = pose[:3, 3:]
            pts = R @ np.asarray(self.rod_pcd.points).T + t  # 3, N
            pts = pts.T  # N, 3
            Tracker.project_xyz_to_color(vis_im_overlapping, self.depth_im, pts, cam_intr, mapping[color])
            Tracker.project_xyz_to_color(vis_im_endcap_only, np.zeros_like(self.depth_im), pts, cam_intr, mapping[color])
        return vis_im_overlapping, vis_im_endcap_only

    def get_2d_vis_fast(self):
        H, W = self.data_cfg['im_h'], self.data_cfg['im_w']
        vis_im_overlapping, vis_im_endcap_only = self.render_current_state_fast()
        # vis_im = np.empty((H*2, W*2, 3), dtype=np.uint8)
        # vis_im[:H, :W] = self.rgb_im
        # vis_im[H:, :W] = self.obs_vis
        # vis_im[:H, W:] = vis_im_overlapping
        # vis_im[H:, W:] = vis_im_endcap_only
        # return vis_im
        return vis_im_overlapping


def main():
    rospy.init_node("tracking_service")

    model_params = [params for params in rospy.get_param_names() if params.startswith("/tracking_service")]
    model_cfg = EasyDict({param.split('/')[-1]: rospy.get_param(param) for param in model_params})

    # read data config
    with open(rospy.get_param("data_cfg_file"), 'r') as fp:
        data_cfg = json.load(fp)

    rod_mesh_file = rospy.get_param("rod_mesh_file")
    top_endcap_mesh_file = rospy.get_param("top_endcap_mesh_file")
    bottom_endcap_mesh_file = rospy.get_param("bottom_endcap_mesh_file")

    assert os.path.exists(rod_mesh_file), f"{rod_mesh_file} doesn't exist!"
    assert os.path.exists(top_endcap_mesh_file), f"{top_endcap_mesh_file} doesn't exist!"
    assert os.path.exists(bottom_endcap_mesh_file), f"{bottom_endcap_mesh_file} doesn't exist!"

    # read rod point cloud
    rod_mesh_o3d = o3d.io.read_triangle_mesh(rod_mesh_file)
    rod_pcd = rod_mesh_o3d.sample_points_poisson_disk(1000)

    # read rod and end cap trimesh for rendering
    rod_mesh = pyrender.Mesh.from_trimesh(trimesh.load_mesh(rod_mesh_file))
    top_endcap_mesh = trimesh.load_mesh(top_endcap_mesh_file)
    top_endcap_pts, _ = sample_surface_even(top_endcap_mesh, 300)
    top_endcap_pcd = o3d.geometry.PointCloud()
    top_endcap_pcd.points = o3d.utility.Vector3dVector(top_endcap_pts)
    bottom_endcap_mesh = trimesh.load_mesh(bottom_endcap_mesh_file)
    bottom_endcap_pts, _ = sample_surface_even(bottom_endcap_mesh, 300)
    bottom_endcap_pcd = o3d.geometry.PointCloud()
    bottom_endcap_pcd.points = o3d.utility.Vector3dVector(bottom_endcap_pts)
    # o3d.visualization.draw_geometries([top_endcap_pcd, bottom_endcap_pcd])

    # initialize tracker
    tracker = Tracker(model_cfg, data_cfg, rod_pcd, rod_mesh, [top_endcap_pcd, bottom_endcap_pcd])
    tracker.run()


if __name__ == '__main__':
    main()
