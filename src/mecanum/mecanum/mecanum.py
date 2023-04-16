"""
Title: Mecanum
Author: Ian Sodersjerna
Date: 4/10/2023
Description: This file contains the mecanum class which is used to control the mecanum robot, calculate wheel speeds, and perform forward kinematics.
"""
# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from mecanum_interfaces.msg import RawEncoder, RawEncoderStamped, WheelSpeeds, WheelSpeedsStamped
from nav_msgs.msg import Odometry


class QuadMecanum(Node):

    def __init__(self):
        super().__init__("mecanum")

        # Wheel geometry
        self.wheel_separation_width = rclpy.get_param("/wheel/separation/horizontal")
        self.wheel_separation_length = rclpy.get_param("/wheel/separation/vertical")
        self.wheel_geometry = (self.wheel_separation_width + self.wheel_separation_length) / 2
        self.wheel_radius = rclpy.get_param("/wheel/diameter") / 2

        # Wheel speeds
        self.wheel_speeds = WheelSpeeds()

        # Odometry
        self.odom = Odometry()
        self.odom.header.frame_id = "motors"
        self.odom.child_frame_id = "base_link"

        # Subscribers
        self.cmd_vel_subscriber = self.add_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10) # what are being told to do
        self.raw_encoder_subscriber = self.add_subscription(RawEncoder, "/motors/raw_encoders", self.raw_encoder_callback, 10) #what we know is happening

        # Publishers
        self.wheel_speeds_publisher = self.add_publisher(WheelSpeeds, "/motors/wheel_speeds", 10) # what we want to happen
        self.odom_publisher = self.add_publisher(Odometry, "/motors/odom", 10) # where we think we are

    def cmd_vel_callback(self, twist):
        self.calculate_wheel_speeds(twist)

    def raw_encoder_callback(self, raw_encoder):
        self.forward_kinematics(raw_encoder)

    
    def inverse_kinematics(self, twist: Twist):
        """
        The inverse kinematic equations allow us to compute the wheel velocities when given the robot's base velocity
        """

        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z

        # Calculate the wheel speeds
        front_left  = (x - y - z * self.wheel_geometry) / self.wheel_radius
        front_right  = (x + y + z * self.wheel_geometry) / self.wheel_radius
        back_left  = (x + y - z * self.wheel_geometry) / self.wheel_radius
        back_right  = (x - y + z * self.wheel_geometry) / self.wheel_radius

        # publish the wheel speeds
        wheel_speeds = WheelSpeeds()
        wheel_speeds.fl = front_left
        wheel_speeds.fr = front_right
        wheel_speeds.bl = back_left
        wheel_speeds.br = back_right
        self.wheel_speeds_pub.publish(wheel_speeds)


    
    def forward_kinematics(self, wheel_velocities: RawEncoder):
        """
        The forward kinematic equations allow us to compute the robot's base velocity when given the individual wheel velocities
        """
        
        temp = self.wheel_radius / 4

        v_x = temp * (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3])
        v_y = temp * (wheel_velocities[0] - wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3])
        v_theta = temp * (wheel_velocities[0] - wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]) / self.wheel_geometry

        self.odom.header.stamp = rclpy.Time.now()

        self.odom.twist.twist.linear.x = v_x
        self.odom.twist.twist.linear.y = v_y
        self.odom.twist.twist.angular.z = v_theta

        self.odom_pub.publish(self.odom)


def main(args=None):
    rclpy.init(args=args)

    qm = QuadMecanum()

    rclpy.spin(qm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()