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

        self.OS_enu_pos = np.zeros((10, 2))
        self.TS_enu_pos = np.zeros((10, 2))

        self.OS_enu_pos[-1, :] = [0.0, 0.0]
        self.TS_enu_pos[-1, :] = [100.0, 100.0]

        self.OS_enu_pos_pub_ = self.create_publisher(Point, namespace_OS + '/enu_pos', 1)
        self.TS_enu_pos_pub_ = self.create_publisher(Point, namespace_TS + '/enu_pos', 1)
        self.OS_heading_pub_ = self.create_publisher(Float64, namespace_OS + '/heading', 1)
        self.TS_heading_pub_ = self.create_publisher(Float64, namespace_TS + '/heading', 1)
        self.OS_spd_pub_ = self.create_publisher(Float64, namespace_OS + '/spd', 1)
        self.TS_spd_pub_ = self.create_publisher(Float64, namespace_TS + '/spd', 1)

        self.timer = self.create_timer(self.dt, self.pub_enu_pos)

    def pub_enu_pos(self):
        OS_enu_pos_p = Point()
        TS_enu_pos_p = Point()
        OS_heading_p = Float64()
        TS_heading_p = Float64()
        OS_spd_p = Float64()
        TS_spd_p = Float64()

        OS_enu_pos_p.x = self.OS_enu_pos[-1, 0]
        OS_enu_pos_p.y = self.OS_enu_pos[-1, 1]
        TS_enu_pos_p.x = self.TS_enu_pos[-1, 0]
        TS_enu_pos_p.y = self.TS_enu_pos[-1, 1]

        OS_heading = np.arctan2(self.OS_enu_pos[-1, 1] - self.OS_enu_pos[-2, 1], self.OS_enu_pos[-1, 0] - self.OS_enu_pos[-2, 0])
        TS_heading = np.arctan2(self.TS_enu_pos[-1, 1] - self.TS_enu_pos[-2, 1], self.TS_enu_pos[-1, 0] - self.TS_enu_pos[-2, 0])
        OS_heading = np.degrees(OS_heading)
        TS_heading = np.degrees(TS_heading)

        OS_spd = np.linalg.norm(self.OS_enu_pos[-1, :] - self.OS_enu_pos[-2, :]) / 0.1
        TS_spd = np.linalg.norm(self.TS_enu_pos[-1, :] - self.TS_enu_pos[-2, :]) / 0.1

        OS_heading_p.data = OS_heading
        TS_heading_p.data = TS_heading
        OS_spd_p.data = OS_spd
        TS_spd_p.data = TS_spd

        self.OS_enu_pos_pub_.publish(OS_enu_pos_p)
        self.TS_enu_pos_pub_.publish(TS_enu_pos_p)
        self.OS_heading_pub_.publish(OS_heading_p)
        self.TS_heading_pub_.publish(TS_heading_p)
        self.OS_spd_pub_.publish(OS_spd_p)
        self.TS_spd_pub_.publish(TS_spd_p)

        new_OS_enu_pos = [self.OS_enu_pos[-1, 0] + 0.1, self.OS_enu_pos[-1, 1] + 0.0] 
        self.OS_enu_pos = np.append(self.OS_enu_pos, [new_OS_enu_pos], axis = 0)
        self.OS_enu_pos = self.OS_enu_pos[1:]
        new_TS_enu_pos = [self.TS_enu_pos[-1, 0] - 0.1, self.TS_enu_pos[-1, 1] + 0.0] 
        self.TS_enu_pos = np.append(self.TS_enu_pos, [new_TS_enu_pos], axis = 0)
        self.TS_enu_pos = self.TS_enu_pos[1:]

def main(args=None):
    rclpy.init(args=args)
    dummy_enu_pub = Dummy_ENU_Publisher()
    rclpy.spin(dummy_enu_pub)
    dummy_enu_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        