import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np

class Dummy_Des_Publisher(Node):
    def __init__(self):
        super().__init__('dummy_des_publisher')

        namespace_OS = '/OS'

        self.dt = 0.1

        self.OS_des_heading_pub = self.create_publisher(Float64, namespace_OS + '/des_heading', 1)
        self.OS_des_spd_pub = self.create_publisher(Float64, namespace_OS + '/des_spd', 1)
        self.OS_heading_pub = self.create_publisher(Float64, namespace_OS + '/heading', 1)
        self.OS_spd_pub = self.create_publisher(Float64, namespace_OS + '/spd', 1)

        self.timer = self.create_timer(self.dt, self.pub_des)

    def pub_des(self):
        OS_des_heading = Float64()
        OS_des_heading.data = 30.0
        self.OS_des_heading_pub.publish(OS_des_heading)
        OS_des_spd = Float64()
        OS_des_spd.data = 1.0
        self.OS_des_spd_pub.publish(OS_des_spd)
        OS_heading = Float64()
        OS_heading.data = 0.0
        self.OS_heading_pub.publish(OS_heading)
        OS_spd = Float64()
        OS_spd.data = 0.8
        self.OS_spd_pub.publish(OS_spd)

def main(args=None):
    rclpy.init(args=args)
    dummy_des_pub = Dummy_Des_Publisher()
    rclpy.spin(dummy_des_pub)
    dummy_des_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()