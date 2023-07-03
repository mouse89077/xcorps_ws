# X-CORPS
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.path as mpath
import math
import wvo

class Collision_Avoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        default_params = {
            # to be modified
            'dt' : 0.1,
            'Left_Bottom' : [37.455842, 126.951699],
            'Right_Bottom' : [37.455934, 126.951688],
            'Left_Top' : [37.455834, 126.951573],
            'Right_Top' : [37.455925, 126.951562],
            'origin' : [37.455842, 126.951699], # Left_Bottom
            'waypoint' : [37.455925, 126.951562], # Right_Top
            'tmin' : 60.0,
            'dmax' : 100.0,
            'TS_dim' : [0.432, 0.152],
            'OS_dim' : [0.914, 0.322],
            'alpha' : 0.1,
            'spd_lim' : [0.9, 1.1],
            'heading_lim' : [-85.0, 85.0],
            'ref_spd' : 1.0,
        }
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Left_Bottom = self.declare_parameter("Left_Bottom", default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter("Right_Bottom", default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter("Left_Top", default_params['Left_Top']).value
        self.Right_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value
        self.origin = self.declare_parameter("origin", default_params['origin']).value
        self.waypoint = self.declare_parameter("waypoint", default_params['waypoint']).value
        self.tmin = self.declare_parameter("tmin", default_params['tmin']).value
        self.dmax = self.declare_parameter("dmax", default_params['dmax']).value
        self.TS_dim = self.declare_parameter("TS_dim", default_params['TS_dim']).value
        self.OS_dim = self.declare_parameter("OS_dim", default_params['OS_dim']).value
        self.alpha = self.declare_parameter("alpha", default_params['alpha']).value
        self.spd_lim = self.declare_parameter("spd_lim", default_params['spd_lim']).value
        self.heading_lim = self.declare_parameter("heading_lim", default_params['heading_lim']).value
        self.ref_spd = self.declare_parameter("ref_spd", default_params['ref_spd']).value
            
        namespace_OS = '/OS'
        namespace_TS = '/TS'
        
        self.TS_enu_pos = np.zeros((10, 2))
        self.OS_enu_pos = np.zeros((10, 2))

        self.TS_spd = np.zeros(10)
        self.TS_heading = np.zeros(10)

        self.OS_spd = np.zeros(10)
        self.OS_heading = np.zeros(10)

        self.des_heading = np.zeros(10)
        self.des_spd = np.zeros(10)
        
        self.TS_tCPA = np.zeros(10)
        self.TS_dCPA = np.zeros(10)
        self.TS_pre_col_ch = np.zeros(10)
        
        # subscriber
        self.OS_enu_pos_sub = self.create_subscription(Point, namespace_OS + '/enu_pos', self.OS_enu_pos_callback, 1)
        self.TS_enu_pos_sub = self.create_subscription(Point, namespace_TS + '/enu_pos', self.TS_enu_pos_callback, 1)
        self.OS_spd_sub = self.create_subscription(Float64, namespace_OS + '/spd', self.OS_spd_callback, 1)
        self.TS_spd_sub = self.create_subscription(Float64, namespace_TS + '/spd', self.TS_spd_callback, 1)
        self.OS_heading_sub = self.create_subscription(Float64, namespace_OS + '/heading', self.OS_heading_callback, 1)
        self.TS_heading_sub = self.create_subscription(Float64, namespace_TS + '/heading', self.TS_heading_callback, 1)
        # publisher
        self.tCPA_pub = self.create_publisher(Float64, namespace_TS + '/tCPA', 1)
        self.dCPA_pub = self.create_publisher(Float64, namespace_TS + '/dCPA', 1)
        self.pre_col_ch_pub = self.create_publisher(Bool, namespace_TS + '/pre_col_ch', 1)
        self.VO_pub = self.create_publisher(Float64MultiArray, namespace_TS + '/VO', 1)
        self.WVO_pub = self.create_publisher(Float64MultiArray, namespace_TS + '/WVO', 1)
        self.RV_pub = self.create_publisher(Float64MultiArray, namespace_OS + '/RV', 1)
        self.RAV_pub = self.create_publisher(Float64MultiArray, namespace_OS + '/RAV', 1)
        self.RAP_pub = self.create_publisher(Float64MultiArray, namespace_OS + '/RAP', 1)
        self.des_heading_pub = self.create_publisher(Float64, namespace_OS + '/des_heading', 1)
        self.des_spd_pub = self.create_publisher(Float64, namespace_OS + '/des_spd', 1)
        # timer
        self.timer = self.create_timer(self.dt, self.pub_Collision)

        self.OS_enu_pos_received = False
        self.TS_enu_pos_received = False
        self.OS_spd_received = False
        self.TS_spd_received = False
        self.OS_heading_received = False
        self.TS_heaidng_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not (self.OS_enu_pos_received and self.TS_enu_pos_received):
            self.get_logger().info('No topic enu_pos_received')
        if not (self.OS_spd_received and self.TS_spd_received):
            self.get_logger().info('No topic spd_received')
        if not (self.OS_spd_received and self.TS_spd_received):
            self.get_logger().info('No topic heading received')
        if (self.OS_enu_pos_received and self.TS_enu_pos_received) and (self.OS_spd_received and self.TS_spd_received) and (self.OS_spd_received and self.TS_spd_received):
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    # callback
    def OS_enu_pos_callback(self, msg):
        self.OS_enu_pos_received = True
        self.OS_enu_pos = np.append(self.OS_enu_pos, [[msg.x, msg.y]], axis=0)
        self.OS_enu_pos = self.OS_enu_pos[1:]
        
    def TS_enu_pos_callback(self, msg):
        self.TS_enu_pos_received = True
        self.TS_enu_pos = np.append(self.TS_enu_pos, [[msg.x, msg.y]], axis=0)
        self.TS_enu_pos = self.TS_enu_pos[1:]
        
    def OS_spd_callback(self, msg):
        self.OS_spd_received = True
        self.OS_spd = np.append(self.OS_spd, msg.data)
        self.OS_spd = self.OS_spd[1:]
        
    def TS_spd_callback(self, msg):
        self.TS_spd_received = True
        self.TS_spd = np.append(self.TS_spd, msg.data)
        self.TS_spd = self.TS_spd[1:]
    
    def OS_heading_callback(self, msg):
        self.OS_heading_received = True
        self.OS_heading = np.append(self.OS_heading, msg.data)
        self.OS_heading = self.OS_heading[1:]
        
    def TS_heading_callback(self, msg):
        self.TS_heading_received = True
        self.TS_heading = np.append(self.TS_heading, msg.data)
        self.TS_heading = self.TS_heading[1:]
    # pub
    def pub_Collision(self):
        # pre_col_ch
        tCPA, dCPA = cal_CPA(self.OS_enu_pos[-1, :], self.TS_enu_pos[-1, :], self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1])
        pre_col_ch = cal_pre_col_ch(tCPA, dCPA, self.tmin. self.tmax)
        
        self.TS_tCPA = np.append(self.TS_tCPA, tCPA)
        self.TS_dCPA = np.append(self.TS_dCPA, dCPA)
        self.TS_pre_col_ch = np.append(self.TS_pre_col_ch, pre_col_ch)
        
        self.TS_tCPA = self.TS_tCPA[1:]
        self.TS_dCPA = self.TS_dCPA[1:]
        self.TS_pre_col_ch = self.TS_pre_col_ch[1:]
        
        TS_tCPA_ = Float64()
        TS_tCPA_.data = tCPA
        self.tCPA_pub.publish(TS_dCPA_)
        
        TS_dCPA_ = Float64()
        TS_dCPA_.data = dCPA
        self.dCPA_pub.publish(TS_tCPA_)
        
        pre_col_ch_ = Bool()
        pre_col_ch_.data = pre_col_ch
        self.pre_col_ch_pub.publish(pre_col_ch_)
        
        # RV, RP
        RV = cal_RV(self.OS_heading[-1], self.spd_lim, self.heading_lim)
        # print(np.shape(RV))
        RVpublish = Float64MultiArray()
        # RVpublish.data = RV
        RVpublish.data = [value for sublist in RV for value in sublist]
        self.RV_pub.publish(RVpublish)
        RAV = RV # initialize
        RP = cal_RP(self.OS_enu_pos[-1, :], RV)
        RAP = RP # initialize

        if pre_col_ch == True:
            self.get_logger().info("Collision avoidance mode: ON")
            # VO
            OS_shape = cal_shape(self.OS_enu_pos[-1, :], self.OS_dim, self.OS_heading[-1])
            TS_shape = cal_shape(self.TS_enu_pos[-1, :], self.TS_dim, self.TS_heading[-1])
            ms = mink_sum(-OS_shape, TS_shape)
            TS_conv_hull = conv_hull(ms)
            ColCone = cal_CC(self.OS_enu_pos[-1, :], TS_conv_hull)
            VO = cal_VO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1], ColCone)
            # WVO
            WVO = cal_WVO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], VO, self.alpha, self.tmin)
            # RAV
            RAV = cal_RAV(RAV, RAP, WVO)
            # RAP
            RAP = cal_RP(self.OS_enu_pos[-1, :], RAV)
            
        else: 
            self.get_logger().info("Collision avoidance mode : OFF")
            VO = []
            WVO = []
            RAV = []
            RAP = []

        VOpublish = Float64MultiArray()
        VOpublish.data = VO
        self.VO_pub.publish(VOpublish)
        WVOpublish = Float64MultiArray()
        WVOpublish.data = WVO
        self.WVO_pub.publish(WVOpublish)
        RAVpublish = Float64MultiArray()
        RAVpublish.data = RAV
        self.RAV_pub.publish(RAVpublish)
        RAPpublish = Float64MultiArray()
        RAPpublish.data = RAP
        self.RAP_pub.publish(RAPpublish)

        # des_heading, des_spd
        des_spd, des_heading = cal_cmd_vel(self.OS_enu_pos[-1, :], RAV, self.ref_spd, self.waypoint, self.des_spd[-1], self.des_heading[-1])

        self.des_heading = np.append(self.des_heading, des_heading)
        self.des_heading = self.des_heading[1:]
        dhdg_pub = Float64()
        dhdg_pub.data = des_heading
        self.des_heading_pub.publish(dhdg_pub)

        self.des_spd = np.append(self.des_spd, des_spd)
        self.des_spd = self.des_spd[1:]
        dspd_pub = Float64()
        dspd_pub.data = des_spd
        self.des_spd_pub.publish(dspd_pub)

    
def main(args=None):
    rclpy.init(args=args)
    collision_avoidance = Collision_Avoidance()
    collision_avoidance.wait_for_topics()
    rclpy.spin(collision_avoidance)
    collision_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()