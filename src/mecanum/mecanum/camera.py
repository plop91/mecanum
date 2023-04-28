import pyrealsense2 as rs
import numpy as np
import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node

# opencv bridge
from cv_bridge import CvBridge, CvBridgeError
# image message
from sensor_msgs.msg import Image
# depth message
from sensor_msgs.msg import CompressedImage
# IMU message
from sensor_msgs.msg import Imu


class Camera(Node):

    def __init__(self):
        super().__init__('camera')

        # cv2 bridge 
        self.bridge = CvBridge()

        # configure the streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # enable the camera streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # enable the imu streams
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        
        # start the pipeline
        self.pipeline.start(self.config)

        # Ros2 publishers
        self.rgb_publisher = self.create_publisher(Image, 'topic', 10)
        self.depth_publisher = self.create_publisher(String, 'topic', 10)
        self.imu_publisher = self.create_publisher(Imu, 'topic', 10)

        # timer
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # get the frames
        color_image, depth_image, motion_frame = self.get_frame()

        # convert the frames to ros2 messages
        color_message = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        depth_message = self.bridge.cv2_to_imgmsg(depth_image, "passthrough")

        #convert the motion frame to ros2 message
        imu_message = Imu()
        imu_message.linear_acceleration.x = motion_frame.acceleration.x
        imu_message.linear_acceleration.y = motion_frame.acceleration.y
        imu_message.linear_acceleration.z = motion_frame.acceleration.z

        imu_message.angular_velocity.x = motion_frame.gyro.x
        imu_message.angular_velocity.y = motion_frame.gyro.y
        imu_message.angular_velocity.z = motion_frame.gyro.z

        # publish the frames
        self.rgb_publisher.publish(color_message)
        self.depth_publisher.publish(depth_message)
        self.imu_publisher.publish(imu_message)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        motion_frame = frames.as_motion_frame().get_motion_data()
        

        if not depth_frame or not color_frame:
            return None, None, motion_frame

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image, motion_frame


def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)

    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()