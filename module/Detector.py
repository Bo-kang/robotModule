import cv2
import numpy as np
import threading

import sys
sys.path.append('usr/local/lib')
import pyrealsense2 as rs
import rospy
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
ROI_X = 0.4

spatial = rs.spatial_filter()
spatial.set_option(rs.option.holes_fill, 3)
hole_filling = rs.hole_filling_filter()
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

def disparity(depth_colormap):
    def func(x, a, b):
        return a * x ** 2 + b
    h, w = CAMERA_HEIGHT, CAMERA_WIDTH
    result_image = np.zeros((h, w)).astype('uint8')

def depth_filter(depth_data):
    depth_data = depth_to_disparity.process(depth_data)
    depth_data = spatial.process(depth_data)
    depth_data = disparity_to_depth.process(depth_data)
    depth_data = hole_filling.process(depth_data)
    return depth_data


class Detector(threading.Thread):
    detect = False

    def __init__(self, detect):
        pipeline = rs.pipeline()  # create pipeline
        config = rs.config()  # create realsense configuration
        config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16,
                             30)  # set resolution and depth type
        config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, 30)

        profile = pipeline.start(config)  # start pipeline
        for x in range(5):
            pipeline.wait_for_frames()  # skip 5 frames(skip error & depth_sensor improvement)

        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))

        depth_sensor = profile.get_device().first_depth_sensor()  # (after 5 frame skip) get depth sensor
        depth_scale = depth_sensor.get_depth_scale()

        threading.Thread.__init__(self)
        self.detect = detect