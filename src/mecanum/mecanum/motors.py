# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from mecanum_interfaces.msg import RawEncoder, WheelSpeeds
from std_msgs.msg import String

import serial

class Motors(Node):

    def __init__(self):
        super().__init__("motors")

        self.status_publisher = self.add_publisher(String, "/motors/status", 10)

        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.port = "/dev/ttyUSB0" # TODO: make this a parameter
        
        if not self.ser.is_open:
            self.status_publisher.publish("Not Connected")
            while not self.ser.is_open:
                self.ser.open()

        self.status_publisher.publish("Connected")
            

        self.raw_encoder_publisher = self.add_publisher(RawEncoder, "/motors/raw_encoders", 10)
        self.wheel_speeds_publisher = self.add_publisher(WheelSpeeds, "/motors/wheel_speeds", 10)

        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        try:
            if not self.ser.is_open:
                self.status_publisher.publish("Not Connected")
                while not self.ser.is_open:
                    self.ser.open()
                self.status_publisher.publish("Connected")
            self.encoder_callback()
            self.wheel_speed_callback()
        except Exception as e:
            self.status_publisher.publish("exception: " + str(e))
            self.timer.cancel()

    def encoder_callback(self):
        self.ser.write(b"e")
        encoder_data = self.ser.readline()
        encoder_data = encoder_data.decode("utf-8")
        encoder_data = encoder_data.split(" ")

        raw_encoder = RawEncoder()
        raw_encoder.front_left = int(encoder_data[0])
        raw_encoder.front_right = int(encoder_data[1])
        raw_encoder.back_left = int(encoder_data[2])
        raw_encoder.back_right = int(encoder_data[3])

        self.raw_encoder_publisher.publish(raw_encoder)

    def wheel_speed_callback(self): 
            self.ser.write(b"s")
            wheel_speed_data = self.ser.readline()
            wheel_speed_data = wheel_speed_data.decode("utf-8")
            wheel_speed_data = wheel_speed_data.split(" ")
    
            wheel_speeds = WheelSpeeds()
            wheel_speeds.front_left = int(wheel_speed_data[0])
            wheel_speeds.front_right = int(wheel_speed_data[1])
            wheel_speeds.back_left = int(wheel_speed_data[2])
            wheel_speeds.back_right = int(wheel_speed_data[3])
    
            self.wheel_speeds_publisher.publish(wheel_speeds)


    

