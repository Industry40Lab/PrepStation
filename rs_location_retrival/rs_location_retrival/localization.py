#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rs_location_retrival.marker_reader import marker_finder, marker_pos_locator
import cv2.aruco as aruco
import pyrealsense2 as rs
import tf2_ros
from panda_info.msg import PandaWsMsg  # Import the custom message
from geometry_msgs.msg import PointStamped, Pose
from colorama import Fore, Back, Style

from scipy.spatial.transform import Rotation as R
# from moveit_interface import robotplanninginterface

marker_locator = marker_pos_locator(42, aruco.DICT_6X6_250)
#change base on Calibration info of camera location acording to robot base link
 
translation = [-0.20449, 0.55968, 0.899054] 
quaternion = [-0.243909, 0.969083, -0.00334553, 0.0370794]

def transform_point(point):

    rotation = R.from_quat(quaternion)  # Make sure quaternion is [x, y, z, w]# Rotate the point using the quaternion    
    rotated_point = rotation.apply(point)
    transformed_point = rotated_point + translation    
    return transformed_point


def box_identifier(box_points, image, depth, cam_inter):
    heat_box_point_mag_h = [3, 4.5, 6, 7.6, 8.8]
    heat_box_point_mag_w = [1.5, 1.3, 1.2, 1.2, 1.2]
    new_3d_point = []
    # Calculate width and height of the original bounding box
    original_width = np.linalg.norm(box_points[1] - box_points[0])
    original_height = np.linalg.norm(box_points[3] - box_points[0])

    # Calculate the unit vectors for width and height directions
    width_unit_vector = (box_points[1] - box_points[0]) / original_width
    height_unit_vector = (box_points[3] - box_points[0]) / original_height

    # Update the points
    new_box_points = np.zeros_like(box_points)
    new_box_points[0] = box_points[0]  # Base point remains the same

    for i in range(len(heat_box_point_mag_h)):

        new_width = original_width * heat_box_point_mag_w[i]
        new_height = original_height * heat_box_point_mag_h[i]
        new_3d_point.append(new_box_points[0] + width_unit_vector * new_width + height_unit_vector * new_height)


    return new_3d_point, image


def convert_to_realsense_intrinsics(camera_info, shape):
    """
    Convert ROS CameraInfo to pyrealsense2.intrinsics
    """
    intrinsics = rs.intrinsics()
    intrinsics.width = shape[1]
    intrinsics.height = shape[0]
    intrinsics.fx = camera_info['fx']  # fx
    intrinsics.fy = camera_info['fy']  # fy
    intrinsics.ppx = camera_info['cx']  # cx
    intrinsics.ppy = camera_info['cy']  # cy
    intrinsics.model = rs.distortion.inverse_brown_conrady
    intrinsics.coeffs = [0, 0, 0, 0, 0]
    return intrinsics

# retrieving the 3D point from the frame base by detecting the aurco marker
def get_point_function(color_frame, depth_frame, color_intrinsics, color_intrinsics_converted):
    target_corner, target_id = marker_locator.finder(color_frame, True)
    if target_corner is not None:
        color_frame = cv2.putText(color_frame, 'Heatsink Detected',
                           (int(target_corner[0][1][0]) - 50, int(target_corner[0][1][1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 5/10, (255, 0, 0), 2, cv2.LINE_AA)
        # cv2.circle(color_frame, (int(target_corner[0][1][0]), int(target_corner[0][1][1])), 1, (255, 0, 0), 5)
        target_corner = target_corner.reshape(-1,2)
        color_intrinsics_converted = convert_to_realsense_intrinsics(color_intrinsics, color_frame.shape)
        data = []
        for corner in target_corner:
            data.append(rs.rs2_deproject_pixel_to_point(color_intrinsics_converted, corner, 0.85))

        return data, color_frame
    return None, color_frame


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('localization')
        self.is_no_mission = True
        # Define the color and depth image subscribers
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.panda_info_sub = self.create_subscription(PandaWsMsg, '/PCBTransferInfo', self.panda_info_callback, 10)
        self.agv_info = self.create_subscription(String, '/AGVInfo', self.agv_info_callback, 10)
        self.came_info_converted = None
        self.publisher_ = self.create_publisher(Pose, 'objectpose/fromcamera', 10)




        # Publisher for the point cloud
        self.points_pub = self.create_publisher(Float32MultiArray, '/location_topic', 10)
        
        # ROS image converter
        self.bridge = CvBridge()

        # Variables to hold frames and intrinsics
        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None
        self.panda_info = None


    def agv_info_callback(self, msg):
            self.get_logger().info(Fore.GREEN + f'[{msg.data}] following message recieved PCB Pickup in progress .... ')

    def panda_info_callback(self, data):
        self.panda_info = data

    def camera_info_callback(self, data):
        """ Callback to get the camera intrinsics from CameraInfo message """
        self.intrinsics = {
            'fx': data.k[0],  # Focal length x
            'fy': data.k[4],  # Focal length y
            'cx': data.k[2],  # Principal point x
            'cy': data.k[5],  # Principal point y
        }


    def color_callback(self, data):
        """ Callback to process the color image """
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting color image: {e}")


    def depth_callback(self, data):
        """ Callback to process the depth image """
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting depth image: {e}")


        # Once we have both color and depth frames, process them
        if self.color_frame is not None and self.intrinsics is not None:
            if self.panda_info is not None: 
                print('The following data is communicated from Loading Workstation:')
                self.get_logger().info(Fore.GREEN + f'\n \tid={self.panda_info.id},\n \t departured={self.panda_info.departured},\n \t width={self.panda_info.width},\n \t height={self.panda_info.height}, \n \t defected={self.panda_info.defected},\n \t material_info={self.panda_info.material_info},\n \t heatsinc_number={self.panda_info.heatsink_number},\n \t defect loc x={self.panda_info.defect_loc_x},\n \t defect loc y={self.panda_info.defect_loc_y}')
                # print(self.panda_info)
                self.panda_info = None
                self.is_no_mission = False
            image = self.process_frames()
            if image is not None:
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', image)
                cv2.waitKey(1)
            else:
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', self.color_frame)
                # cv2.imshow('RealSense', image)
                cv2.waitKey(1)




    def process_frames(self):
        """ Function to process color and depth frames and publish points """
        data, self.color_frame = get_point_function(self.color_frame, self.depth_frame, self.intrinsics, self.came_info_converted)

        if data is not None:
            sample_data = [data[1][:-1], data[0][:-1], data[3][:-1], data[2][:-1]]
            # print(self.came_info_converted)
            new_point, color_frame = box_identifier(np.array(sample_data), self.color_frame, 0.85, self.came_info_converted)
            now = rclpy.time.Time()

            # pose_point = choose()
            # TODO

            # transformed_points = transform_point(point=np.append(new_point[0],[0.85]))
            transformed_points = transform_point(point=data[1])

            # Create a Pose message
            pose = Pose()

            # Set position (x, y, z)
            pose.position.x = transformed_points[0]+ 0.03
            pose.position.y = transformed_points[1]+ 0.08
            pose.position.z = transformed_points[2]+ 0.2

            # Set orientation (quaternion: x, y, z, w)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            # checi if is on mission
            if not self.is_no_mission:
                # Publish the message
                self.publisher_.publish(pose)            
                self.is_no_mission = True
                print('Following point is sent to the Universal Robot:')
                print(transformed_points)
                print(f'is on mission {self.is_no_mission}')

        
            # Display points on the image
            image = cv2.putText(self.color_frame, f'Heatsink Location: {int((transformed_points[0]+ 0.03) * 100)}, {int((transformed_points[1]+ 0.06) * 100)}, {int(transformed_points[2] * 100)}',
                                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            return image
            # return self.color_frame


def main(args=None):
    print('start working ...')
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
