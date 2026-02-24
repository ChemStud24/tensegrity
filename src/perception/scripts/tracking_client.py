#!/home/willjohnson/miniconda3/envs/tensegrity/bin/python

import os
import json

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

import rospy
import rosgraph
import cv_bridge
from std_msgs.msg import Float64MultiArray
from tensegrity_perception.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse


def init_tracker(rgb_im: np.ndarray, depth_im: np.ndarray, cable_lengths: np.ndarray):
    bridge = cv_bridge.CvBridge()
    rgb_msg = bridge.cv2_to_imgmsg(rgb_im, encoding="rgb8")
    depth_msg = bridge.cv2_to_imgmsg(depth_im, encoding="mono16")
    cable_length_msg = Float64MultiArray()
    cable_length_msg.data = cable_lengths.tolist()

    request = InitTrackerRequest()
    request.rgb_im = rgb_msg
    request.depth_im = depth_msg
    request.cable_lengths = cable_length_msg

    service_name = "init_tracker"
    rospy.loginfo(f"Waiting for {service_name} service...")
    rospy.wait_for_service(service_name)
    rospy.loginfo(f"Found {service_name} service.")
    try:
        init_tracker_srv = rospy.ServiceProxy(service_name, InitTracker)
        rospy.loginfo("Request sent. Waiting for response...")
        response: InitTrackerResponse = init_tracker_srv(request)
        rospy.loginfo(f"Got response. Request success: {response.success}")
        return response.success
    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
    return False


def get_pose():
    service_name = "get_pose"
    rospy.loginfo(f"Waiting for {service_name} service...")
    rospy.wait_for_service(service_name)
    rospy.loginfo(f"Found {service_name} service.")
    poses = []
    try:
        request = GetPoseRequest()
        get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
        rospy.loginfo("Request sent. Waiting for response...")
        response: GetPoseResponse = get_pose_srv(request)
        rospy.loginfo(f"Got response. Request success: {response.success}")
        if response.success:
            for pose in response.poses:
                T = np.eye(4)
                T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
                T[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                pose.orientation.w]).as_matrix()
                poses.append(T)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
    return poses


def main():
    if not rosgraph.is_master_online():
        print("roscore is not running! Run roscore first!")
        exit(0)

    rospy.init_node("tracking_client", anonymous=True)

    dataset ="/home/willjohnson/catkin_ws/src/tensegrity/src/tensegrity_perception/dataset"
    video = 'pebbles9'
    video_path = os.path.join(dataset, video)

    color_path = os.path.join(video_path, 'color', '0000.png')
    depth_path = os.path.join(video_path, 'depth', '0000.png')
    info_path = os.path.join(video_path, 'data', '0000.json')

    rgb_im = cv2.cvtColor(cv2.imread(color_path), cv2.COLOR_BGR2RGB)
    depth_im = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    with open(info_path, 'r') as f:
        info = json.load(f)
    cable_lengths = np.zeros(len(info['sensors']))
    for key, sensor_data in info['sensors'].items():
        cable_lengths[int(key)] = sensor_data['length']

    init_tracker(rgb_im, depth_im, cable_lengths)
    poses = get_pose()
    print(poses)


if __name__ == "__main__":
    main()
