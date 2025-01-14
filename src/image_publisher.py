#!/usr/bin/env python3

from __future__ import print_function

import os
import pyrealsense2 as rs
import numpy as np
import cv2
import json

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ==========================================================
# Write something similar in your control script

# from time_sync_demo.msg import StampedIndex
# ==========================================================

# from run_tensegrity_tracking import get_pose

class RealSenseCamera:

    def __init__(self, config=None):
        if config is not None:
            self.config = config
        else:
            self.config = rs.config()

            # for L515
            # self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
            # self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            # self.config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
            # self.config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)

            # for D435
            # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            self.bridge = CvBridge()

            # ros publisher
            self.color_im_pub = rospy.Publisher('rgb_images', Image, queue_size=6)
            self.depth_im_pub = rospy.Publisher('depth_images', Image, queue_size=6)

            # self.cam_ext = np.array([[-9.98945777e-01, -4.16785258e-02,  1.92415053e-02,  7.09414767e-01],
            #                          [-4.16669873e-02,  9.99131053e-01,  1.00035678e-03,  6.28713551e-01],
            #                          [-1.92664788e-02,  1.97566621e-04, -9.99814365e-01,  1.88932353e+00],
            #                          [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

            # self.inv_cam_ext = np.array([[-9.98945777e-01 -4.16669873e-02 -1.92664788e-02  7.71264097e-01]
            #                              [-4.16785258e-02  9.99131053e-01  1.97566621e-04 -5.98973138e-01]
            #                              [ 1.92415053e-02  1.00035678e-03 -9.99814365e-01  1.87469366e+00]
            #                              [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]])

            # self.cam_intr = np.array([[ 601.67626953125, 0.0, 325.53509521484375 ],
            #                           [ 0.0, 601.7260131835938, 248.60618591308594 ],
            #                           [ 0.0, 0.0, 1.0 ]])


    def run(self, rate, output_dir=None):

        if output_dir is not None:
            if not os.path.exists(output_dir):
                os.mkdir(output_dir)
            color_folder = os.path.join(output_dir, 'color')
            depth_folder = os.path.join(output_dir, 'depth')
            if not os.path.exists(color_folder):
                os.mkdir(color_folder)
            if not os.path.exists(depth_folder):
                os.mkdir(depth_folder)

        pipeline = rs.pipeline()
        profile = pipeline.start(self.config)

        # depth align to color
        align = rs.align(rs.stream.color)
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        print("camera intrinsics")
        print(color_intrinsics)

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = int(round(1.0 / depth_sensor.get_depth_scale()))
        print("Depth Scale:", depth_scale)

        # save camera information to a json file
        self.cam_info = dict()
        self.cam_info['id'] = os.path.basename(output_dir)
        self.cam_info['im_w'] = color_intrinsics.width
        self.cam_info['im_h'] = color_intrinsics.height
        self.cam_info['depth_scale'] = depth_scale
        fx, fy = color_intrinsics.fx, color_intrinsics.fy
        cx, cy = color_intrinsics.ppx, color_intrinsics.ppy
        self.cam_info['cam_intr'] = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        cam_config_path = os.path.join(output_dir, 'config.json')
        with open(cam_config_path, 'w') as f:
            print("Camera info has been saved to:", cam_config_path)
            json.dump(self.cam_info, f, indent=4)
        save = False
        count = 0
        try:
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            while not rospy.is_shutdown():
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                timestamp = rospy.get_rostime()
                color_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB), 'rgb8')
                # color_msg = self.bridge.cv2_to_imgmsg(color_image, 'rgb8')
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, 'mono16')
                color_msg.header.stamp = timestamp
                depth_msg.header.stamp = timestamp
                self.color_im_pub.publish(color_msg)
                self.depth_im_pub.publish(depth_msg)
                """
                # save data
                color_im = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8') # double check this
                depth_im = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
                cv2.imwrite(os.path.join('../data/camera_only/color', str(count).zfill(4) + ".png"), color_im)
                cv2.imwrite(os.path.join('../data/camera_only/depth', str(count).zfill(4) + ".png"), depth_im)
                data = {}
                data['header'] = {'seq':color_msg.header.seq,'secs':color_msg.header.stamp.to_sec()}
                json.dump(data,open(os.path.join('../data/camera_only/data', str(count).zfill(4) + ".json"),'w'))
                count += 1
                print(count)
                """
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # # add current COM and principal axis to image
                # if '/tracking_service' in rosnode.get_node_names():
                #     COM, principal_axis = get_pose()

                #     cam_COM = np.zeros((4,1))
                #     cam_COM[0:3,:] = COM
                #     cam_COM[3,0] = 1

                #     cam_PA = 
                #     COM = np.matmul(inv_cam_ext,cam_COM)

                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))

                # Show images
                cv2.imshow('RealSense', images)
                key = cv2.waitKey(1)
                if (key & 0xFF) == ord('q'):
                    break
                if (key & 0xFF) == ord('s'):
                    save = True
                    print("saving images to: ", output_dir)
                if save and output_dir is not None:
                    cv2.imwrite(os.path.join(output_dir, "color", str(count).zfill(4) + ".png"), color_image)
                    cv2.imwrite(os.path.join(output_dir, "depth", str(count).zfill(4) + ".png"), depth_image)
                    count += 1
                    if count % 50 == 0:
                        print(str(count), "frames have been saved.")
                
                rate.sleep()
        finally:
            pipeline.stop()

if __name__ == '__main__':
    rospy.init_node("realsense")
    rate = rospy.Rate(30)
    camera = RealSenseCamera()
    camera.run(rate, 'images')
