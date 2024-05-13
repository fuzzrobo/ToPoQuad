# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from topoquad_msgs.msg import QuadRobotCmdNeckAngle

import math

class NeckJoyNode(Node):

    def __init__(self):
        super().__init__('neck_joy_node')
        # Parameters
        self.declare_parameter('angle_pan_limit', math.pi/4)
        self.angle_pan_limit = self.get_parameter('angle_pan_limit').value
        self.declare_parameter('angle_tilt_limit', math.pi/4)
        self.angle_tilt_limit = self.get_parameter('angle_tilt_limit').value
        
        # Publishers
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotCmdNeckAngle, '/neck/angle', 10)
        
        # Subscribers
        self.joy_sub_ = self.create_subscription(Joy,'/joy',self.joy_cb, 10)
    
    def joy_cb(self, msg):
        neck_cmd = QuadRobotCmdNeckAngle()
        neck_cmd.angle_pan = msg.axes[3] * self.angle_pan_limit
        neck_cmd.angle_tilt = msg.axes[4] * self.angle_tilt_limit
        self.neck_cmd_pub_.publish(neck_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NeckJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()