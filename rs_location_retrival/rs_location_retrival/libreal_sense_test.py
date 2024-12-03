## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
import os
import pyrealsense2 as rs
import numpy as np
import cv2
from marker_reader import marker_finder, marker_pos_locator
import cv2.aruco as aruco



# Configure depth and color streams
pipeline = rs.pipeline()
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
marker_locator = marker_pos_locator(42,aruco.DICT_6X6_250)

corners_finder = marker_finder(42, aruco.DICT_6X6_250)

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)


profile = cfg.get_stream(rs.stream.color)  # Fetch stream profile for depth stream
intr = profile.as_video_stream_profile().get_intrinsics()  # Downcast to video_stream_profile and fetch intrinsics

print(intr)
# align = rs.align(rs.stream.color)
# [ 1280x720  p[642.224 369.929]  f[909.705 908.924]  Inverse Brown Conrady [0 0 0 0 0] ]
# depth
# intrin = cfg.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
# color
intrin = cfg.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
# extr = cfg.get_stream(rs.stream.depth).get_extrinsics_to(cfg.get_stream(rs.stream.color))
# rot = np.array(extr.rotation).reshape(3, 3)
# translation = np.array(extr.translation)
k = np.array([[intr.fx, 0, intr.ppx],
              [0, intr.fy, intr.ppy],
              [0, 0, 1]])
# Example pixel coordinates
u, v = 1280, 720

# Calculate world coordinates
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # aligned_frames = align.process(frames)
        # color_frame = aligned_frames.first(rs.stream.color)
        # depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        corner = corners_finder.finder(frame=color_image)
        # base_corners, base_ids = marker_locator.base_finder(color_image)
        target_corner, target_id = marker_locator.finder(color_image, True)


        if corner is not None:
            # pixel_3D  = marker_locator.locating(color_image, k, depth_frame.get_distance(int(target_corner[0][0][0]),
            #                                                                 (int(target_corner[0][0][1]))))
            # pixel_3D = rs.rs2_deproject_pixel_to_point(intr,(int(target_corner[0][0][0]),int(target_corner[0][0][1])), depth_frame.get_distance(int(target_corner[0][0][0]),int(target_corner[0][0][1])))
            # target_corner[0][0][0], target_corner[0][0][1] = 640, 360
            # if not len(base_ids) == 0:
            #     color_image = aruco.drawDetectedMarkers(color_image, np.array(base_corners), np.array(base_ids))
            if target_corner is not None:
                # color_image = aruco.drawDetectedMarkers(color_image, np.array([target_corner]),
                #                                             np.array([target_id]))
                cv2.circle(color_image, (int(target_corner[0][1][0]), (int(target_corner[0][1][1]))), 1, (255, 0, 0), 5)

            depth = depth_frame.get_distance(int(target_corner[0][1][0]), (int(target_corner[0][1][1])))
            data = rs.rs2_deproject_pixel_to_point(intrin, [target_corner[0][1][0], target_corner[0][1][1]], depth)
            # data_2 = rs.rs2_deproject_pixel_to_point(intrin, [target_corner[0][2][0], target_corner[0][2][1]], depth)
            # point_test = rs.rs2_project_point_to_pixel(intr, data)
            # cv2.circle(color_image, (int(target_corner[0][1][0]), (int(target_corner[0][1][1]))), 1, (255, 0, 0), 2)
            # cv2.circle(color_image, (int(target_corner[0][2][0]), (int(target_corner[0][2][1]))), 1, (255, 255, 0), 2)

            print(f'deprojection results is equal to ({data[0]}, {data[1]}) and depth is {depth}')
            test_point = [-0.045, -0.076, 0.9]
            prj_point = rs.rs2_project_point_to_pixel(intrin, data)
            cv2.circle(color_image, (int(prj_point[0]), (int(prj_point[1]))), 1, (255, 0, 255), 2)

            image = cv2.putText(color_image, f'{int(data[0] * 100)}, {int(data[1] * 100)}, {int(depth * 100)}',
                                (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 0, 255), 2, cv2.LINE_AA)

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
