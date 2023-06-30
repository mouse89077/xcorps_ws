# X-CORPS
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64, Int32
import pymap3d as pm
import numpy as np

class PWMConverter(Node):
    def __init__(self):
        super().__init__('pwm_converter')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'pwm_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.thrust_lim = config['thrust_lim']
        #     self.servo_lim = config['servo_lim']
        #     self.pwm_lim = config['pwm_lim']
        #     self.dt = config['dt']
        #     self.Kp_thrust = config['Kp_thrust']
        #     self.Kd_thrust = config['Kd_thrust']
        #     self.Kp_servo = config['Kp_servo']
        #     self.Kd_servo = config['Kd_servo']
        default_params = {
            'servo_lim' : 0.1,
            'pwm_lim' : [1100, 1900],
            'dt' : 0.1, 
            'Kp_thrust' : 1.0,
            'Kd_thrust' : 1.0,
            'Kp_servo' : 1.0,
            'Kd_servo' : 1.0,
        }
        self.servo_lim = self.declare_parameter("servo_lim", default_params['servo_lim']).value
        self.pwm_lim = self.declare_parameter("pwm_lim", default_params['pwm_lim']).value
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Kp_thrust = self.declare_parameter("Kp_thrust", default_params['Kp_thrust']).value
        self.Kd_thrust = self.declare_parameter("Kd_thrust", default_params['Kd_thrust']).value
        self.Kp_servo = self.declare_parameter("Kp_servo", default_params['Kp_servo']).value
        self.Kd_servo = self.declare_parameter("Kd_servo", default_params['Kp_servo']).value
            
        namespace_OS = '/OS'
        
        self.OS_des_heading = np.zeros(10)
        self.OS_des_spd = np.zeros(10)
        self.OS_heading = np.zeros(10)
        self.OS_spd = np.zeros(10)
        
        self.err_heading = np.zeros(10)
        self.err_spd = np.zeros(10)
        
        self.thrust_pwm = np.zeros(10)
        self.servo_pwm = np.zeros(10)
        
        self.OS_des_heading_sub = self.create_subscription(Float64, namespace_OS + '/des_heading', self.OS_des_heading_callback, 1)
        self.OS_des_spd_sub =self.create_subscription(Float64, namespace_OS + '/des_spd', self.OS_des_spd_callback, 1)
        self.OS_heading = self.create_subscription(Float64, namespace_OS + '/heading', self.OS_heading_callback, 1)
        self.OS_spd = self.create_subscription(Float64, namespace_OS + '/spd', self.OS_des_spd_callback, 1)
        
        self.pwm_pub =self.create_publisher(Int32, namespace_OS + '/pwm', 1)
        
        self.pwm_timer = self.create_timer(self.dt, self.pub_pwm)

        self.OS_des_heading_received = False
        self.OS_des_spd_received = False
        self.OS_heading_received = False
        self.OS_spd_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.OS_des_heading_received:
            self.get_logger().info('No topic OS_des_heading_received')
        if not self.OS_des_spd_received:
            self.get_logger().info('No topic OS_des_spd_received')
        if not self.OS_heading_received:
            self.get_logger().info('No topic OS_heading_received')
        if not self.OS_spd_received:
            self.get_logger().info('No topic OS_spd_received')
        if self.OS_des_heading_received and self.OS_des_spd_received and self.OS_heading_received and self.OS_spd_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    def OS_des_heading_callback(self, msg):
        self.OS_des_heading_received = True
        self.OS_des_heading = np.append(self.OS_des_heading, msg.data)
        self.OS_des_heading = self.OS_des_heading[1:]
        
    def OS_des_spd_callback(self, msg):
        self.OS_des_spd_received = True
        self.OS_des_spd = np.append(self.OS_des_spd, msg.data)
        self.OS_des_spd = self.OS_des_spd[1:]
        
    def OS_heading_callback(self, msg):
        self.OS_heading_received = True
        self.OS_heading = np.append(self.OS_heading, msg.data)
        self.OS_heading = self.OS_heading[1:]
        
        err_hdg = self.OS_des_heading[-1] - self.OS_heading[-1]
        if err_hdg > 180:
            err_hdg -= 360
        elif err_hdg < -180:
            err_hdg += 360
            
        self.err_heading = np.append(self.err_heading, err_hdg)
        self.err_heading = self.err_heading[1:]
        
    def OS_spd_callback(self, msg):
        self.OS_spd_received = True
        self.OS_spd = np.append(self.OS_spd, msg.data)
        self.OS_spd = self.OS_spd[1:]
        
        err_spd = self.OS_des_spd[-1] - self.OS_spd[-1]
        self.err_spd = np.append(self.err_spd, err_spd)
        self.err_spd = self.err_spd[1:]
        
    def pub_pwm(self):
        self.cal_thrust_pwm
        self.cal_servo_pwm

        pwm = Int32()
        pwm.data = int(str(int(self.thrust_pwm[-1])) + str(int(self.servo_pwm[-1])))
        
        self.pwm_pub.publish(pwm)
        
    # functions that returns something
    def cal_thrust_pwm(self):
        thrust_pwm = self.Kp_thrust * self.err_spd[-1] - self.Kd_thrust * self.err_spd[-2]
        if thrust_pwm > self.pwm_lim[1]:
            thrust_pwm = self.pwm_lim[1]
        elif thrust_pwm < self.pwm_lim[0]:
            thrust_pwm = self.pwm_lim[0]
        
        self.thrust_pwm = np.append(self.thrust_pwm, thrust_pwm)
        self.thrust_pwm = self.thrust_pwm[1:]
    
    def cal_servo_pwm(self):
        servo = self.Kp_servo * self.err_heading[-1] - self.Kd_servo * self.err_heading[-2]
        if servo > self.servo_lim[1]:
            servo = self.servo_lim[1]
        elif servo < self.servo_lim[0]:
            servo = self.servo_lim[0]
        norm_servo = (servo - self.servo_lim[0]) / (self.servo_lim[1] - self.servo_lim[0])
        s_pwm = int(self.pwm_lim[0] + (self.pwm_lim[1] - self.pwm_lim[0]) * norm_servo)
        
        self.servo_pwm = np.append(self.servo_pwm, s_pwm)
        self.servo_pwm = self.servo_pwm[1:]
    
def main(args=None):
    rclpy.init(args=args)
    pwm_converter = PWMConverter()
    rclpy.spin(pwm_converter)
    pwm_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()