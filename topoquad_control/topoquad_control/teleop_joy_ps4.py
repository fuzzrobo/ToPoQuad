# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from topoquad_msgs.msg import QuadRobotNeck
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
        
        self.cmd_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotNeck, 'neck/command', 10)
        
        self.joy_sub_ = self.create_subscription(Joy,'joy',self.joy_cb, 10)
        
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.joy = None
        self.joy_rx_time = time.time()

    def timer_cb(self):
        diff = time.time() - self.joy_rx_time
        cmd = Twist()
        if self.joy is not None and (diff < 0.1):
            cmd.linear.x = self.joy.axes[7] * self.linear_v
            cmd.linear.y = self.joy.axes[6] * self.linear_v
            cmd.angular.z = self.angular_w if self.joy.buttons[4] else (-self.angular_w if self.joy.buttons[5] else 0.0)
            self.cmd_pub_.publish(cmd)
            
            neck_cmd = QuadRobotNeck()
            neck_cmd.angle_pan =  self.joy.axes[3] * self.angle_pan_max
            neck_cmd.angle_tilt = self.joy.axes[4] * self.angle_tilt_max
            self.neck_cmd_pub_.publish(neck_cmd)
    
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
