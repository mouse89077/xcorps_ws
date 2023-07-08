import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import pymap3d as pm

class GNSSConverter(Node):
    def __init__(self):
        super().__init__('gnss_converter')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'gnss_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.origin = self.get_parameter('origin').value
        default_params = {
            'Left_Bottom' : [37.4557578333, 126.9517325], # to be modified
            'Right_Bottom' : [37.4558135, 126.9517261667],
            'Left_Top' : [37.4558071667, 126.9516436667], # to be modified
            'Right_Top' : [37.4557513333, 126.9516513333], # to be modified
            'origin' : [37.455778333, 126.9517325], # to be modified, same as Left_Bottom
        }
        self.Left_Bottom = self.declare_parameter("Left_Bottom", default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter("Right_Bottom", default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter("Left_Top", default_params['Left_Top']).value
        self.Left_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value

        self.origin = self.declare_parameter("origin", default_params['origin']).value

        self.OS = [0, 0, 0]
        self.TS = [0, 0, 0]

        namespace_OS = '/OS'
        namespace_TS = '/TS3'
        self.OS_gps_lon_sub = self.create_subscription(String, namespace_OS + '/gps/lon', self.OS_gps_lon_callback, 1)
        self.OS_gps_lat_sub = self.create_subscription(String, namespace_OS + '/gps/lat', self.OS_gps_lat_callback, 1)
        self.TS_gps_lon_sub = self.create_subscription(String, namespace_TS + '/gps/lon', self.TS_gps_lon_callback, 1)
        self.TS_gps_lat_sub = self.create_subscription(String, namespace_TS + '/gps/lat', self.TS_gps_lat_callback, 1)
        
        self.OS_enu_pos_pub = self.create_publisher(Point, namespace_OS + '/enu_pos', 10)
        self.TS_enu_pos_pub = self.create_publisher(Point, namespace_TS + '/enu_pos', 10)
        self.OS_timer = self.create_timer(0.1, self.OS_pub_enu_pos)
        self.TS_timer = self.create_timer(0.1, self.TS_pub_enu_pos)

        self.OS_gps_lon_received = False
        self.OS_gps_lat_received = False
        self.TS_gps_lon_received = False
        self.TS_gps_lat_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.OS_gps_lat_received:
            self.get_logger().info('No topic OS_gps_lat_received')
        if not self.OS_gps_lon_received:
            self.get_logger().info('No topic OS_gps_lon_received')
        if self.OS_gps_lat_received and self.OS_gps_lon_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    def OS_gps_lon_callback(self, msg):
        self.OS_gps_lon_received = True
        str_gps_lon = msg.data
        self.OS_gps_lon = float(str_gps_lon)

    def OS_gps_lat_callback(self, msg):
        self.OS_gps_lat_received = True
        str_gps_lat = msg.data
        self.OS_gps_lat = float(str_gps_lat)
        
    def TS_gps_lon_callback(self, msg):
        self.TS_gps_lon_received = True
        str_gps_lon = msg.data
        self.TS_gps_lon = float(str_gps_lon)

    def TS_gps_lat_callback(self, msg):
        self.TS_gps_lat_received = True
        str_gps_lat = msg.data
        self.TS_gps_lat = float(str_gps_lat)

    def enu_convert(self, gnss):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], 0, self.origin[0], self.origin[1], 0)
        return e, n, u
    
    def OS_pub_enu_pos(self):
        self.OS[0], self.OS[1], self.OS[2] = self.enu_convert([self.OS_gps_lat, self.OS_gps_lon, 0])
        enu_pos = Point()
        enu_pos.x = self.OS[0]
        enu_pos.y = self.OS[1]
        self.OS_enu_pos_pub.publish(enu_pos)
        
    def TS_pub_enu_pos(self):
        self.TS[0], self.TS[1], self.TS[2] = self.enu_convert([self.TS_gps_lat, self.TS_gps_lon, 0])
        enu_pos = Point()
        enu_pos.x = self.TS[0]
        enu_pos.y = self.TS[1]
        self.TS_enu_pos_pub.publish(enu_pos)

def main(args=None):
    rclpy.init(args=args)
    gnss_converter = GNSSConverter()
    gnss_converter.wait_for_topics()
    rclpy.spin(gnss_converter)
    gnss_converter.destroy_node()
    rclpy.shutodown()

if __name__ == '__main__':
    main()
 
