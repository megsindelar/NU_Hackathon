## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
import modern_robotics as mr
# First import the library
from ctypes.wintypes import HHOOK
from re import U
from turtle import color, up
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import time

robot = InterbotixManipulatorXS("px100", "arm", "gripper")

global lh
lh = 112
global uh
hh = 130
uh = 167
max_H = 250
window_detection_name = 'Object Detection'

calibrated = 0

def low_H_trackbar(val):
    global lh
    global uh 
    lh = val
    lh = min(uh-1, lh)
    cv2.setTrackbarPos("Low H",window_detection_name,lh)

def high_H_trackbar(val):
    global lh
    global uh 
    uh = val
    uh = max(uh, lh+1)
    cv2.setTrackbarPos("High H",window_detection_name,uh)

def convert_deg_to_rad(deg):
    rad = deg*(np.pi/180)
    return rad

"""Create trackbar"""
cv2.namedWindow(window_detection_name)
cv2.createTrackbar("Low H", window_detection_name, lh, max_H, low_H_trackbar)
cv2.createTrackbar("High H", window_detection_name, lh, max_H, high_H_trackbar)

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

global coord
coord = []
coord_prev = [1,0,0]
global count
count = 0

global x,y,z
x = 0
y = 0
z = 0

# Streaming loop
#try:

# while count < 100:

depth = 0

joints = robot.arm.get_joint_commands()
T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
[R,p] = mr.TransToRp(T)
p = [[p[0]], [p[1]], [p[2]]]

robot.arm.go_to_home_pose()

while True:
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue
    #   return 

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # """Get depth xyz"""
    # rs.rs2_deproject_pixel_to_point(rs.intrinsics, [px, py], depth_image)
    
    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    """Convert BGR to HSV"""
    hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)

    """Define range of colors in HSV"""
    #lower_hue = np.array([lh,50,50])
    #upper_hue = np.array([uh,255,255])
    lower_hue = np.array([125,50,50])   #112
    upper_hue = np.array([187,255,255]) #167

    """Threshold the HSV image"""
    mask = cv2.inRange(hsv, lower_hue, upper_hue)

    """Bitwise-AND mask and original image"""
    res = cv2.bitwise_and(bg_removed,bg_removed, mask = mask)

    cv2.imshow('frame',bg_removed)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    """Draw Contours"""
    ret, thresh = cv2.threshold(mask, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(mask, contours, -1, (50,50,50), 3)
    cv2.drawContours(bg_removed, contours, -1, (0, 255, 0), 3)

    """Finding Centroid"""
    if len(contours)>1:

        areas = []
        areas = [cv2.contourArea(c) for c in contours]
        max_ind = np.argmax(areas)
        main_body = contours[max_ind]

        M = cv2.moments(main_body)
        if M['m00']>0:
            cx = int(M['m10']/M['m00'])     #centroid in x
            cy = int(M['m01']/M['m00'])     #centroid in y

            cv2.circle(bg_removed, (cx, cy), 5, (255, 0, 0), -1)

            cfg = profile.get_stream(rs.stream.color)
            intr = cfg.as_video_stream_profile().get_intrinsics()
            dpt_frame = aligned_depth_frame.as_depth_frame()
            depth = dpt_frame.get_distance(cx, cy) 
            #print(depth)
            depth = depth#* 0.0010000000474974513
            #print(f"depth: {depth}")
            coord = rs.rs2_deproject_pixel_to_point(intr, [cx,cy], depth)
            #print(coord)
            if coord[0]!=0 and coord[1]!=0 and coord[2]!=0:
                x_pos = coord[0]
                y_pos = coord[1]
                z_pos = coord[2]
                print(coord)
                
                if calibrated == 1:
                    pr_y = oc_y - z_pos
                    pr_x = oc_x - x_pos
                    pr_z = oc_z - y_pos

                    waist = np.arctan(pr_y/pr_x)
                    print(f"waist: {waist}")
                    robot.arm.set_single_joint_position('waist', waist)

                    #dp_x = (pr_x - pr_x_past) + 0.02
                    dp_x = (pr_x - pr_x_past) + 0.02
                    print(dp_x)
                    robot.arm.set_ee_cartesian_trajectory(dp_x)
                    pr_x_past = pr_x

                    robot.gripper.grasp()

                    time.sleep(2)
                    robot.gripper.release()
                    time.sleep(5)
                    

    # Render images:
    #   depth align to color on left
    #   depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    images = np.hstack((bg_removed, depth_colormap))

    cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    cv2.imshow('Align Example', images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

    """Recording"""
    #config.enable_record_to_file("realsense_record")
    # if count > 30:
    #     x += coord[0]
    #     y += coord[1]
    #     z += coord[2]


    count +=1
    if count == 100:
        joints = robot.arm.get_joint_commands()
        T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
        [R,p] = mr.TransToRp(T)
        p_rx = p[0]
        p_ry = p[1]
        p_rz = p[2]
        oc_y = p_ry + z_pos 
        oc_x = p_rx + x_pos
        oc_z = p_rz + y_pos
        calibrated = 1
        pr_x_past = p_rx
        print("calibrated!")
        

# x = x/70
# y = y/70
# z = z/70
# coord = [x,y,z]

# finally:
#     pipeline.stop()