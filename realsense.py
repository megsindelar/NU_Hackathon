## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
from ctypes.wintypes import HHOOK
from re import U
from turtle import up
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

lh = 110
uh = 130
max_H = 360//2
window_detection_name = 'Object Detection'

def low_H_trackbar(val):
    global lh
    global hh 
    lh = val
    lh = min(hh-1, lh)
    cv2.setTrackbarPos("Low H",window_detection_name,lh)

def high_H_trackbar(val):
    global lh
    global hh 
    hh = val
    hh = max(hh, lh+1)
    cv2.setTrackbarPos("High H",window_detection_name,hh)

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

# Streaming loop
try:
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

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        """Convert BGR to HSV"""
        hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)

        """Define range of colors in HSV"""
        lower_hue = np.array([lh,50,50])
        upper_hue = np.array([uh,255,255])

        """Threshold the HSV image"""
        mask = cv2.inRange(hsv, lower_hue, upper_hue)

        """Bitwise-AND mask and original image"""
        res = cv2.bitwise_and(bg_removed,bg_removed, mask = mask)

        cv2.imshow('frame',bg_removed)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)

        #img = cv2.imread(bg_removed)
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask, contours, -1, (50,50,50), 3)
        cv2.drawContours(bg_removed, contours, -1, (0, 255, 0), 3)

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
        
finally:
    pipeline.stop()