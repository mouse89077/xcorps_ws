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
        
        # subscriber
        self.OS_enu_pos_sub = self.create_subscription(Point, namespace_OS + '/enu_pos', self.OS_enu_pos_callback, 1)
        self.TS_enu_pos_sub = self.create_subscription(Point, namespace_TS + '/enu_pos', self.TS_enu_pos_callback, 1)
        self.OS_spd_sub = self.create_subscription(Float64, namespace_OS + '/spd', self.OS_spd_callback, 1)
        self.TS_spd_sub = self.create_subscription(Float64, namespace_TS + '/spd', self.TS_spd_callback, 1)
        self.OS_heading_sub = self.create_subscription(Float64, namespace_OS + '/heading', self.OS_heading_callback, 1)
        self.TS_heading_sub = self.create_subscription(Float64, namespace_TS + '/heading', self.TS_heading_callback, 1)
        # publisher
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
        tCPA = self.cal_tCPA(self.OS_enu_pos[-1, :], self.TS_enu_pos[-1, :], self.OS_heading[-1], self.TS_heading[-1])
        dCPA = self.cal_dCPA(self.OS_enu_pos[-1, :], self.TS_enu_pos[-1, :], self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1], \
                            tCPA)
        pre_col_ch = self.cal_pre_col_ch(tCPA, dCPA)
        temp = Bool()
        temp.data = pre_col_ch
        self.pre_col_ch_pub.publish(temp)
        # RV, RP
        RV = self.cal_RV(self.OS_heading[-1])
        # print(np.shape(RV))
        RVpublish = Float64MultiArray()
        # RVpublish.data = RV
        RVpublish.data = [value for sublist in RV for value in sublist]
        self.RV_pub.publish(RVpublish)
        RAV = RV # initialize
        RP = self.cal_RP(self.OS_enu_pos[-1, :], RV)
        RAP = RP # initialize

        if pre_col_ch == True:
            # VO
            OS_shape = self.cal_shape(self.OS_enu_pos[-1, :], self.OS_dim, self.OS_heading[-1])
            TS_shape = self.cal_shape(self.TS_enu_pos[-1, :], self.TS_dim, self.TS_heading[-1])
            ms = self.mink_sum(-OS_shape, TS_shape)
            TS_conv_hull = self.conv_hull(ms)
            ColCone = self.cal_CC(self.OS_pos[-1, :], self.TS_pos[-1, :], OS_shape, TS_conv_hull)
            VO = self.cal_VO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1], ColCone)
            # WVO
            WVO = self.cal_WVO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], VO)
            # RAV
            RAP = self.cal_RP(self.OS_enu_pos[-1, :], RAV)
            RAV = self.cal_RAV(RAV, RAP, WVO)
            # RAP
            RAP = self.cal_RP(self.OS_enu_pos[-1, :], RAV)
            
        else: 
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
        cmd_vel = self.cal_cmd_vel(RAV)

        self.des_heading = np.append(self.des_heading, cmd_vel[1])
        self.des_heading = self.des_heading[1:]
        dhdg_pub = Float64()
        dhdg_pub.data = cmd_vel[1]
        self.des_heading_pub.publish(dhdg_pub)

        self.des_spd = np.append(self.des_spd, cmd_vel[0])
        self.des_spd = self.des_spd[1:]
        dspd_pub = Float64()
        dspd_pub.data = cmd_vel[0]
        self.des_spd_pub.publish(dspd_pub)

    # functions which return something
    def cal_tCPA(self, OS_pos, TS_pos, OS_heading, TS_heading):
        # Convert heading values to radians
        OS_heading_rad = np.radians(OS_heading)
        TS_heading_rad = np.radians(TS_heading)
        # Calculate relative position of two vessels
        rel_pos = np.array(OS_pos) - np.array(TS_pos)
        # Calculate relative velocity of two vessels
        OS_enu_vel = [self.OS_spd[-1] * np.cos(OS_heading_rad), self.OS_spd[-1] * np.sin(OS_heading_rad)]
        TS_enu_vel = [self.TS_spd[-1] * np.cos(TS_heading_rad), self.TS_spd[-1] * np.sin(TS_heading_rad)]
        rel_vel = np.array(OS_enu_vel) - np.array(TS_enu_vel)
        
        # Calculate tCPA
        norm_rel_vel = np.linalg.norm(rel_vel)
        if norm_rel_vel != 0:
            tCPA = np.dot(rel_pos, rel_vel) / (norm_rel_vel ** 2)
        else:
            tCPA = 0.0

        return tCPA

    def cal_dCPA(self, OS_pos, TS_pos, OS_spd, TS_spd, OS_heading, TS_heading, tCPA):
        # Convert heading values to radians
        OS_heading_rad = np.radians(OS_heading)
        TS_heading_rad = np.radians(TS_heading)
        OS_enu_vel = [OS_spd * np.cos(OS_heading_rad), OS_spd * np.sin(OS_heading_rad)]
        TS_enu_vel = [TS_spd * np.cos(TS_heading_rad), TS_spd * np.sin(TS_heading_rad)]
        # calculate enu position at CPA
        OS_pos_CPA = OS_pos + [value * tCPA for value in OS_enu_vel]
        TS_pos_CPA = TS_pos + [value * tCPA for value in TS_enu_vel]

        dCPA = np.linalg.norm(OS_pos_CPA - TS_pos_CPA)
        return dCPA
    
    def cal_pre_col_ch(self, tCPA, dCPA):
        if tCPA < self.tmin and dCPA < self.dmax:
            pre_col_ch = True 
        else:
            pre_col_ch = False

        return pre_col_ch

    def rotate_mat(self, heading):
        heading_rad = np.radians(heading)
        rot_mat = np.array([[np.cos(heading_rad), -np.sin(heading_rad)], [np.sin(heading_rad), np.cos(heading_rad)]])
        return rot_mat

    def cal_shape(self, pos, dim, heading):
        heading_rad = np.radians(heading)
        temp_x = np.transpose([2/3, 1/3, -1/3, -1/3, 1/3])
        temp_x = [value * dim[0] for value in temp_x]
        temp_y = np.transpose([0, 1/2, 1/2, -1/2, -1/2])
        temp_y = [value * dim[1] for value in temp_y]
        temp = np.hstack((temp_x, temp_y))
        Rot = self.rotate_mat(heading_rad)
        r_temp = [value * Rot for value in temp]
        rt_temp = [value + pos for value in r_temp]
        shape = np.array(rt_temp)
        return shape

    def mink_sum(self, A, B):
        # A = - OS_shape
        # B = TS_shape
        # sA = size(A)
        # sB = size(B)
        # AB = reshape(A, [1, sA]) + reshape(B, [sB(1), 1, sB(2)])
        # AB = reshape(AB, [], 2)
        # mink_sum = AB + OS_pos
        ms = []
        for point1 in A:
            for point2 in B:
                sum_point = [point1[0] + point2[0], point1[1] + point2[1]]
                ms.append(sum_point)

        return ms

    def conv_hull(self, mink_sum):
        hull = ConvexHull(mink_sum)
        hull_points = mink_sum[hull.vertices]
        return hull_points

    def cal_CC(self, OS_pos, TS_pos, OS_shape, TS_conv_hull):
        # OS_shape = n x 2 array of [x, y]
        # TS_shape = m x 2 array of [x, y]
        # OS_pos = [x, y]
        # TS_pos = [x, y]
        # to be filled...
        grad = np.zeros(len(TS_conv_hull[:, 0]))
        for i in range(len(TS_conv_hull[:, 0])):
            grad[i] = np.arctan2(TS_conv_hull[i, 1]-OS_pos[1], TS_conv_hull[i, 0]-OS_pos[0])
        
        max_idx = np.argmax(grad)
        min_idx = np.argmin(grad)

        CC = np.zeros((3, 2))
        CC[0, :] = TS_conv_hull[min_idx, :]
        CC[1, :] = OS_pos
        CC[2, :] = TS_conv_hull[max_idx, :]
        return CC

    def cal_VO(self, OS_spd, TS_spd, OS_heading, TS_heading, CC):
        # Convert heading values to radians
        OS_heading_rad = np.radians(OS_heading)
        TS_heading_rad = np.radians(TS_heading)
        # Calculate relative velocity of two vessels
        OS_enu_vel = [OS_spd * np.cos(OS_heading_rad), OS_spd * np.sin(OS_heading_rad)]
        TS_enu_vel = [TS_spd * np.cos(TS_heading_rad), TS_spd * np.sin(TS_heading_rad)]
        rel_vel = np.array(TS_enu_vel) - np.array(OS_enu_vel)
        # Calculate VO from CC
        VO = [value + rel_vel for value in CC]
        return VO
        
    def cal_WVO(self, OS_spd, TS_spd, OS_heading, VO):
        a = self.alpha * OS_spd * self.tmin / 3
        b = a / 2
        ell = self.ellipse([0, 0], a, b, OS_heading) 
        ms = self.mink_sum(ell, VO)
        ch = self.conv_hull(ms)
        WVO = ch

        return WVO

    def ellipse(self, Center, a, b, heading):
        theta = np.linspace(0, 2*np.pi, 10)
        x = np.transpose([value * a for value in theta])
        y = np.transpose([value * b for value in theta])
        ell = np.hstack(x, y)
        r_ell = [self.rotate_mat(heading) * value for value in ell]
        rt_ell = [value + Center for value in r_ell]
        return rt_ell  
    
    def cal_RV(self, heading):
        spd_res = 0.1
        heading_res = 1.0
        spd_arr = np.arange(self.spd_lim[0], self.spd_lim[1], spd_res)
        heading_arr = np.arange(self.heading_lim[0], self.heading_lim[1], heading_res)
        heading_arr = [value + heading for value in heading_arr]

        rv = []
        RV = []
        for spd in spd_arr:
            for hdg in heading_arr:
                if hdg > 180:
                    hdg -= 360
                elif hdg < -180:
                    hdg += 360

                rv = [spd, hdg]
                RV = np.append(RV, rv)

        RV = np.reshape(RV, (-1, 2))

        return RV

    def cal_RP(self, OS_pos, RV):
        if len(RV) == 0:
            RP = []
        else:
            RP = np.zeros((len(RV), 2))
            RP[:, 0] = OS_pos[0] + RV[:, 0] * np.cos(RV[:, 1])
            RP[:, 1] = OS_pos[1] + RV[:, 0] * np.sin(RV[:, 1])

        return RP
        
    def cal_RAV(self, RAV, RAP, WVO):
        if len(RAV) == 0:
            RAV = []
        else:
            polygon_path = mpath.Path(WVO)
            test_points = RAP
            results = polygon_path.contains_points(test_points)
            RAV = test_points[results]

        return RAV

    def cal_cmd_vel(self, RAV):
        if np.size(RAV) != 0:
            ref_ang = np.arctan2(self.waypoint[1] - self.OS_enu_pos[-1, 1], self.waypoint[0] - self.OS_enu_pos[-1, 0])
            ref_ang = math.degrees(ref_ang)
            ref_vel = [self.ref_spd, ref_ang]
            
            RAV = np.array(RAV)  # Convert RAV to a NumPy array
            cost = [value - ref_vel for value in RAV]
            idx = np.argmin(np.linalg.norm(cost))
            cmd_vel = RAV[idx, :]
        else:
            cmd_vel = [self.des_spd[-1], self.des_heading[-1]]

        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    collision_avoidance = Collision_Avoidance()
    collision_avoidance.wait_for_topics()
    rclpy.spin(collision_avoidance)
    collision_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()