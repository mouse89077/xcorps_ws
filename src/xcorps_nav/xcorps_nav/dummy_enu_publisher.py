import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np

class Dummy_ENU_Publisher(Node):
    def __init__(self):
        super().__init__('dummy_enu_publisher')

        namespace_OS = '/OS'
        namespace_TS = '/TS'

        self.dt = 0.1
        self.OS_enu_pos = [0.0, 0.0]
        self.TS_enu_pos = [100.0, 100.0]

        self.OS_enu_pos_pub = self.create_publisher(Point, namespace_OS + '/enu_pos', 1)
        self.TS_enu_pos_pub = self.create_publisher(Point, namespace_TS + '/enu_pos', 1)

        self.timer = self.create_timer(self.dt, self.pub_enu_pos)

    def pub_enu_pos(self):
        OS_enu_pos_p = Point()
        TS_enu_pos_p = Point()

        OS_enu_pos_p.x = self.OS_enu_pos[0]
        OS_enu_pos_p.y = self.OS_enu_pos[1]
        TS_enu_pos_p.x = self.TS_enu_pos[0]
        TS_enu_pos_p.y = self.TS_enu_pos[1]

        self.OS_enu_pos_pub.publish(OS_enu_pos_p)
        self.TS_enu_pos_pub.publish(TS_enu_pos_p)

        self.OS_enu_pos[0] += 0.1
        self.TS_enu_pos[1] += -0.1

def main(args=None):
    rclpy.init(args=args)
    dummy_enu_pub = Dummy_ENU_Publisher()
    rclpy.spin(dummy_enu_pub)
    dummy_enu_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        