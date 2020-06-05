import cv2
import numpy as np
import threading

import sys
sys.path.append('usr/local/lib')
import pyrealsense2 as rs
import rospy
import math
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
    fit_log = [2.16836735e-04, 3.78571428e+00]

    for i in range(h//7*3):
        result_image[h - 1 - i] = int(func(i, fit_log[0], fit_log[1]))

    for j in range(h//7*3, h + 1):
        result_image[h - 1 - j] = depth_colormap[h - 1 - j]

    return result_image

# def depth_filter(depth_data):
#     depth_data = depth_to_disparity.process(depth_data)
#     depth_data = spatial.process(depth_data)
#     depth_data = disparity_to_depth.process(depth_data)
#     depth_data = hole_filling.process(depth_data)
#     return depth_data
#
# def pixel_to_3d(pixel_x, pixel_y, depth_image, depth_intrinsics):  # Convert pixel (x, y), depth to (x, y, z) coord
#     coord = [pixel_y, pixel_x]
#     z, x, y = rs.rs2_deproject_pixel_to_point(depth_intrinsics, coord, depth_image[pixel_y][pixel_x] * depth_scale)
#     z = -z
#     return [x, y, z]

def calculate_distance(list1, list2):  # Take two 2D points and calculate distance
    dist = float(math.sqrt((list1[0] - list2[0]) ** 2 + (list1[1] - list2[1]) ** 2))
    return dist  # [m]


medium_goal = None
detect = False
def detecting():
    global medium_goal
    global detect
    # Init realsense
    pipeline = rs.pipeline()  # create pipeline
    config = rs.config()  # create realsense configuration
    config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, 30)  # set resolution and depth type
    config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, 30)
    profile = pipeline.start(config)  # start pipeline
    for x in range(5):
        pipeline.wait_for_frames()  # skip 5 frames(skip error & depth_sensor improvement)

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))

    depth_sensor = profile.get_device().first_depth_sensor()  # (after 5 frame skip) get depth sensor
    depth_scale = depth_sensor.get_depth_scale()

    align_to = rs.stream.color
    align = rs.align(align_to)
    final_map = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH)).astype('uint8')

    def pixel_to_3d(pixel_x, pixel_y, depth_image, depth_intrinsics):  # Convert pixel (x, y), depth to (x, y, z) coord
        coord = [pixel_y, pixel_x]
        z, x, y = rs.rs2_deproject_pixel_to_point(depth_intrinsics, coord, depth_image[pixel_y][pixel_x] * depth_scale)
        z = -z
        return [x, y, z]

    def depth_filter(depth_data):
        depth_data = depth_to_disparity.process(depth_data)
        depth_data = spatial.process(depth_data)
        depth_data = disparity_to_depth.process(depth_data)
        depth_data = hole_filling.process(depth_data)
        return depth_data

    while True:

        # Get frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        depth_frame = depth_filter(depth_frame)

        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        # Validate that both frames are valid
        if not depth_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())

        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.005), cv2.COLORMAP_BONE)[:, :, 0]  # TODO:
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_BONE)[:, :, 1]  # TODO:
        #depth_colormap = np.dstack((depth_colormap, depth_colormap, depth_colormap))

        disparity_map = disparity(depth_colormap)


        bg_removed_mask = np.where(((5 < depth_colormap - disparity_map) & (depth_colormap - disparity_map < 20)), 0,
                           255).astype('uint8')  # remove floor
        bg_removed = cv2.bitwise_and(depth_colormap, depth_colormap, mask=bg_removed_mask)

        bg_removed2_mask = np.where(depth_colormap > 20, 0, 255).astype('uint8')  # remove far
        bg_removed2 = cv2.bitwise_and(bg_removed, bg_removed, mask=bg_removed2_mask)


        test = np.dstack((bg_removed2, bg_removed2, bg_removed2))

        test_yuv = cv2.cvtColor(test, cv2.COLOR_BGR2YUV)

        test_yuv[:, :, 0] = cv2.equalizeHist(test_yuv[:, :, 0])

        # 1. Histogram Equalization
        test_Hist = cv2.cvtColor(test_yuv, cv2.COLOR_YUV2RGB)

        test_gray = cv2.cvtColor(test_Hist, cv2.COLOR_RGB2GRAY)

        # 2. Binarization
        test_Bi = cv2.threshold(test_gray, 1, 255, cv2.THRESH_BINARY)[1]

        # 3. Noise Reduction (Median Filtering)
        floor_removed_bw = cv2.medianBlur(test_Bi, 5)  # Kunel Size 5

        if (floor_removed_bw == 255).any():
            # gray = cv2.cvtColor(floor_removed, cv2.COLOR_RGB2GRAY)  # TODO:
            # threshold = cv2.threshold(gray, 1, 255, 0)[1]

            # 2) detect obstacles
            contours = cv2.findContours(floor_removed_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

            floor_removed = cv2.cvtColor(floor_removed_bw, cv2.COLOR_GRAY2RGB)

            # 3) calculate medium goal
            medium_points = np.empty(shape=[0, 3])  # (x, y, r)
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)  # find obstacle in camera view point
                if (
                        80 < h or 120 < w) and 10 < w and 10 < h:  # here!!!!!!!!!!!!!!!!!!!!!!!! h < 200 is to ignore the wall

                    cv2.drawContours(floor_removed, [cnt], 0, (0, 255, 0), 2)

                    cv2.rectangle(floor_removed, (x, y), (x + w, y + h), (0, 0, 255), 2)

                    # cv2.putText(test_color, "w = " + str(w) + ", h = " + str(h), (300, 300), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (255, 127, 0), 1)

                    cx = int(x + w / 2)
                    cy = int(y + h / 2)

                    cv2.circle(floor_removed, (cx, cy), 5, (255, 0, 255), -1)

                    # convert camera view point obstacle (x, y) to bird view (x, y) coordinate
                    points = np.empty(shape=[0, 2])
                    for i in range(w // 10):
                        for j in range(h // 20):
                            points = np.empty(shape=[0, 2])  # (x, y)
                            # obstacle (x,y) in the same z of depth camera
                            point = pixel_to_3d(x + 10 * i, y + 20 * j, depth_image, depth_intrinsics)[0:2]
                            # if calculate_distance(point, [0, 0]) < DIST_TO_OBSTACLE:  # unit : meter      1.5
                            if floor_removed_bw[y + 20 * j][x + 10 * i] == 255:  # unit : meter      1.5
                                # (camera coord) offset tuning
                                point[0] -= 0.1
                                point[1] += 0.1

                                if (abs(point[0] > 0.5)) and (abs(point[1] ) > 0.5):  # 50cm far from XY_COORD_M
                                    points = np.append(points, [point], 0)

                    cv2.putText(floor_removed, str(points.shape), (cx, cy + 20), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1,(255, 255, 0), 2)

                    # calculate medium points
                    if points.any():
                        if (abs(points[:, 0]) < ROI_X).any():  # ROI_X : 0.4
                            cv2.putText(floor_removed,
                                        str(round(calculate_distance(np.mean(points, 0), [0, 0]), 2)) + 'm',
                                        (cx, cy - 20), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, (255, 255, 0), 2)

                            if np.mean(points, 0)[0] > 0:
                                print("obs in right")
                                medium_point = np.min(points, 0)
                                # medium_point[0] -= 0.65 # offset
                                # medium_point[1] = 0.5
                            else:
                                print("obs in left")
                                medium_point = np.max(points, 0)
                                # medium_point[0] += 0.65 # offset
                                # medium_point[1] = 0.5

                            medium_point[0] = (np.min(points, 0)[0] + np.max(points, 0)[0]) / 2
                            medium_point[1] = (np.min(points, 0)[1] + np.max(points, 0)[1]) / 2
                            medium_point = np.append(medium_point, calculate_distance([0, 0], medium_point))  # append r : distacne from (0, 0) to obstacle
                            medium_points = np.append(medium_points, [medium_point], 0)
                if medium_points.any():
                    detect = True
                    medium_goal = medium_points[np.argmin(medium_points[:, 2])][:2]  # (x, y) about minimum of r




