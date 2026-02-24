#!/home/willjohnson/miniconda3/envs/tensegrity/bin/python
from dt_apriltags import Detector
import numpy as np
import json
import sys
import os
import pyrealsense2 as rs
import rospkg

if len(sys.argv) != 2:
        print("Usage: python hsv_filter_tuner.py <trialname>")
        sys.exit(1)

visualization = True
try:
    import cv2
except:
    raise Exception('You need cv2 in order to run this script.')

try:
    from cv2 import imshow
except:
    print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
    print("Visualization will be disabled.")
    visualization = False

package_path = rospkg.RosPack().get_path('tensegrity_perception')
data_cfg_path = os.path.join(package_path,'configs/data_cfg.json')
test_images_path = os.path.join(package_path,'../../data/',sys.argv[1],'color')

tag_size = 0.172

#### GET CAMERA PARAMETERS ####

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline = rs.pipeline()
profile = pipeline.start(config)

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
with open(data_cfg_path) as f:
    cam_info = json.load(f)
cam_info['im_w'] = color_intrinsics.width
cam_info['im_h'] = color_intrinsics.height
cam_info['depth_scale'] = depth_scale
fx, fy = color_intrinsics.fx, color_intrinsics.fy
cx, cy = color_intrinsics.ppx, color_intrinsics.ppy
cam_info['cam_intr'] = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

with open(data_cfg_path, 'w') as f:
    print("Camera info has been saved to ", data_cfg_path)
    json.dump(cam_info, f, indent=4)

#### GET EXTRINSIC MATRIX ####

# try:
#     import yaml
# except:
#     raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# with open('/home/willjohnson/catkin_ws/src/tensegrity/calibration/camera_calibration_info.yaml', 'r') as stream:
#     parameters = yaml.safe_load(stream)

#### test WITH THE SAMPLE IMAGE ####

print("\n\nTESTING WITH A SAMPLE IMAGE")

img = cv2.imread(test_images_path+'/0000.png', cv2.IMREAD_GRAYSCALE)
cameraMatrix = np.array(cam_info.get('cam_intr')).reshape((3,3))
camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

if visualization:
    cv2.imshow('Original image',img)

tags = at_detector.detect(img, True, camera_params, tag_size)
tag = tags[0]
print(tag)

for TAG in tags:
    print(TAG)

# update camera extrinsic matrix
R = tag.pose_R
t = tag.pose_t
R = R.T
t = -np.matmul(R,t)
cam_extr = np.zeros((4,4))
cam_extr[:3,:3] = R
cam_extr[:3,3:] = t
cam_extr[3,3] = 1
R180x = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
cam_extr = np.matmul(R180x,cam_extr)

print(cam_extr)

cam_info['cam_extr'] = cam_extr.tolist()
with open(data_cfg_path, 'w') as f:
    json.dump(cam_info, f, indent=4)
print('Camera extrinsic matrix saved to ', data_cfg_path)

color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

for tag in tags:
    for idx in range(len(tag.corners)):
        cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

    cv2.putText(color_img, str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))

if visualization:
    cv2.imshow('Detected tags', color_img)

    k = cv2.waitKey(0)
    # if k == 27:         # wait for ESC key to exit
    #     cv2.destroyAllWindows()