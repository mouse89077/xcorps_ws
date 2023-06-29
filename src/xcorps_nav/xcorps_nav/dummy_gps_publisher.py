# X-CORPS
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String
import numpy as np

class Dummy_GPS_Publisher(Node):
    def __init__(self):
        super().__init__('dummy_gps_publisher')

        namespace_OS = '/OS'
        namespace_TS = '/TS'

        default_params = {
            'dt' : 0.1,
            'Left_Bottom' : [37.455842, 126.951699],
            'Right_Bottom' : [37.455934, 126.951688],
            'Left_Top' : [37.455834, 126.951573],
            'Right_Top' : [37.455925, 126.951562],
        }
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Left_Bottom = self.declare_parameter("Left_Bottom", default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter("Right_Bottom", default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter("Left_Top", default_params['Left_Top']).value
        self.Right_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value

        # self.OS_gps_lat = str((self.Left_Bottom[0] + self.Right_Bottom[0]) / 2)
        # self.OS_gps_lon = str((self.Left_Bottom[1] + self.Right_Bottom[1]) / 2)
        # self.TS_gps_lat = str((self.Left_Bottom[0] + self.Left_Top[0]) / 2)
        # self.TS_gps_lon = str((self.Left_Bottom[1] + self.Left_Top[1]) / 2)
        self.OS_gps_lat = str(self.Left_Bottom[0])
        self.OS_gps_lon = str(self.Left_Bottom[1])
        self.TS_gps_lat = str(self.Right_Top[0])
        self.TS_gps_lon = str(self.Right_Top[1])

        self.OS_gps_lat_pub = self.create_publisher(String, namespace_OS + '/gps/lat', 1)
        self.OS_gps_lon_pub = self.create_publisher(String, namespace_OS + '/gps/lon', 1)
        self.TS_gps_lat_pub = self.create_publisher(String, namespace_TS + '/gps/lat', 1)
        self.TS_gps_lon_pub = self.create_publisher(String, namespace_TS + '/gps/lon', 1)

        self.timer = self.create_timer(self.dt, self.pub_gps)

    def pub_gps(self):
        OS_gps_lat = String()
        OS_gps_lat.data = self.OS_gps_lat
        self.OS_gps_lat_pub.publish(OS_gps_lat)

        OS_gps_lon = String()
        OS_gps_lon.data = self.OS_gps_lon
        self.OS_gps_lon_pub.publish(OS_gps_lon)

        TS_gps_lat = String()
        TS_gps_lat.data = self.TS_gps_lat
        self.TS_gps_lat_pub.publish(TS_gps_lat)

        TS_gps_lon = String()
        TS_gps_lon.data = self.TS_gps_lon
        self.TS_gps_lon_pub.publish(TS_gps_lon)

def main(args=None):
    rclpy.init(args=args)
    dummy_gps_pub = Dummy_GPS_Publisher()
    rclpy.spin(dummy_gps_pub)
    dummy_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
