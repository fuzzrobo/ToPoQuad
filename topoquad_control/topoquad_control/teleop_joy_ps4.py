# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from topoquad_msgs.msg import QuadRobotNeck
from topoquad_msgs.msg import QuadRobotLeg
from sensor_msgs.msg import Joy

class TeleopJoyXboxNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_xbox')
        # param
        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('linear_v', 2.5)
        self.declare_parameter('angular_w', 3.14)
        self.declare_parameter('angle_pan_max', 1.57)
        self.declare_parameter('angle_tilt_max', 0.78)
        
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.linear_v = self.get_parameter('linear_v').get_parameter_value().double_value
        self.angular_w = self.get_parameter('angular_w').get_parameter_value().double_value
        self.angle_pan_max = self.get_parameter('angle_pan_max').get_parameter_value().double_value
        self.angle_tilt_max = self.get_parameter('angle_tilt_max').get_parameter_value().double_value
        
        self.twist_cmd_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotNeck, 'neck/command', 10)
        self.leg_cmd_pub_ = self.create_publisher(QuadRobotLeg, 'legs/command', 10)
        
        self.joy_sub_ = self.create_subscription(Joy,'joy',self.joy_cb, 10)
        
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.joy = None
        self.joy_rx_time = time.time()
        self.force_control_sample_command = 0.0

    def timer_cb(self):
        diff = time.time() - self.joy_rx_time
        if self.joy is None or (diff > 0.1):
            return 

        neck_cmd = QuadRobotNeck()
        neck_cmd.angle_pan =  self.joy.axes[3] * self.angle_pan_max
        neck_cmd.angle_tilt = self.joy.axes[4] * self.angle_tilt_max
        self.neck_cmd_pub_.publish(neck_cmd)

        leg_cmd = QuadRobotLeg()
        if self.joy.buttons[0]:
            self.get_logger().info("Leg Force Command [H weak (45 deg)]")
            leg_cmd.force_fr.x =  4.0
            leg_cmd.force_fr.y =  4.0
            leg_cmd.force_fr.z = -6.0
            leg_cmd.force_fl.x =  4.0
            leg_cmd.force_fl.y =  4.0
            leg_cmd.force_fl.z = -6.0
            leg_cmd.force_br.x =  4.0
            leg_cmd.force_br.y =  4.0
            leg_cmd.force_br.z = -6.0
            leg_cmd.force_bl.x =  4.0
            leg_cmd.force_bl.y =  4.0
            leg_cmd.force_bl.z = -6.0
            leg_cmd.angles_fr = [0.0, 0.78, 0.78]
            leg_cmd.angles_fl = [0.0, 0.78, 0.78]
            leg_cmd.angles_br = [0.0, 0.78, 0.78]
            leg_cmd.angles_bl = [0.0, 0.78, 0.78]
            self.leg_cmd_pub_.publish(leg_cmd)
            self.twist_cmd_pub_.publish(Twist())
            return
        elif self.joy.buttons[1]:
            self.get_logger().info("Leg Force Command [strong]")
            leg_cmd.torques_fr = [1.2, 1.2, 1.2]
            leg_cmd.torques_fl = [1.2, 1.2, 1.2]
            leg_cmd.torques_br = [1.2, 1.2, 1.2]
            leg_cmd.torques_bl = [1.2, 1.2, 1.2]
            leg_cmd.angles_fr = [0.0, 0.78, 0.78]
            leg_cmd.angles_fl = [0.0, 0.78, 0.78]
            leg_cmd.angles_br = [0.0, 0.78, 0.78]
            leg_cmd.angles_bl = [0.0, 0.78, 0.78]
            self.leg_cmd_pub_.publish(leg_cmd)
            self.twist_cmd_pub_.publish(Twist())
            return
        elif self.joy.buttons[2]:
            self.get_logger().info("Leg Force Command [V weak]")
            leg_cmd.force_fr.x =  3.0
            leg_cmd.force_fr.y =  0.0
            leg_cmd.force_fr.z = -0.5
            leg_cmd.force_fl.x =  3.0
            leg_cmd.force_fl.y =  0.0
            leg_cmd.force_fl.z = -0.5
            leg_cmd.force_br.x =  3.0
            leg_cmd.force_br.y =  0.0
            leg_cmd.force_br.z = -0.5
            leg_cmd.force_bl.x =  3.0
            leg_cmd.force_bl.y =  0.0
            leg_cmd.force_bl.z = -0.5
            leg_cmd.angles_fr = [0.0, 0.78, 0.78]
            leg_cmd.angles_fl = [0.0, 0.78, 0.78]
            leg_cmd.angles_br = [0.0, 0.78, 0.78]
            leg_cmd.angles_bl = [0.0, 0.78, 0.78]
            self.leg_cmd_pub_.publish(leg_cmd)
            self.twist_cmd_pub_.publish(Twist())
            return
        if self.joy.buttons[3]:
            self.get_logger().info("Leg Force Command [H weak (-45 deg)]")
            leg_cmd.force_fr.x = -4.0
            leg_cmd.force_fr.y = +4.0
            leg_cmd.force_fr.z = -6.0
            leg_cmd.force_fl.x = -4.0
            leg_cmd.force_fl.y = +4.0
            leg_cmd.force_fl.z = -6.0
            leg_cmd.force_br.x = -4.0
            leg_cmd.force_br.y = +4.0
            leg_cmd.force_br.z = -6.0
            leg_cmd.force_bl.x = -4.0
            leg_cmd.force_bl.y = +4.0
            leg_cmd.force_bl.z = -6.0
            leg_cmd.angles_fr = [0.0, 0.78, 0.78]
            leg_cmd.angles_fl = [0.0, 0.78, 0.78]
            leg_cmd.angles_br = [0.0, 0.78, 0.78]
            leg_cmd.angles_bl = [0.0, 0.78, 0.78]
            self.leg_cmd_pub_.publish(leg_cmd)
            self.twist_cmd_pub_.publish(Twist())
            return
        
        twist_cmd = Twist()
        twist_cmd.linear.x = self.joy.axes[7] * self.linear_v
        twist_cmd.linear.y = self.joy.axes[6] * self.linear_v
        twist_cmd.angular.z = self.angular_w if self.joy.buttons[4] else (-self.angular_w if self.joy.buttons[5] else 0.0)
        self.twist_cmd_pub_.publish(twist_cmd)
        
    def joy_cb(self, msg):
        self.joy = msg
        self.joy_rx_time = time.time()
        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyXboxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
