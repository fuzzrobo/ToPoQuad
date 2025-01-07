# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from topoquad_msgs.msg import QuadRobotCmdNeckAngle, QuadRobotCmdLegPoint
from geometry_msgs.msg import Point

from math import pi, sin, cos

# Static
r = 0.00
s = 0.02
h = 0.01

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        # Parameters
        self.declare_parameter('angle_pan_limit', pi/4)
        self.angle_pan_limit = self.get_parameter('angle_pan_limit').value
        self.declare_parameter('angle_tilt_limit', pi/4)
        self.angle_tilt_limit = self.get_parameter('angle_tilt_limit').value
        
        # Variable
        self.theta = 0.0
        self.theta_diff =  (2 * pi) / 20
        self.joy = None
        self.u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
        
        # Publishers
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotCmdNeckAngle, 'neck/angle', 10)
        self.leg_point_pub_ = self.create_publisher(QuadRobotCmdLegPoint, 'legs/point', 10)
        
        # Subscribers
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        
        # Timer
        self.timer = self.create_timer(0.05, self.timer_cb)
        
    def timer_cb(self):
        if self.joy != None:
            # Neck
            neck_cmd = QuadRobotCmdNeckAngle()
            neck_cmd.angle_pan = self.joy.axes[3] * self.angle_pan_limit
            neck_cmd.angle_tilt = self.joy.axes[4] * self.angle_tilt_limit
            self.neck_cmd_pub_.publish(neck_cmd)
            
            # Legs
            vx = -self.joy.axes[6]
            vy = self.joy.axes[7]
            msg = QuadRobotCmdLegPoint()
            if vx != 0:
                if vx > 0:
                    self.theta += self.theta_diff
                elif vx < 0:
                    self.theta -= self.theta_diff
                x, y, z  =  self.u(self.theta)

                msg.leg_fl.x = -0.09+r*x - s*cos(self.theta-pi/2*0)
                msg.leg_fl.y = 0.09+r*y

                msg.leg_fr.x =  0.09+r*x - s*cos(self.theta-pi/2*2)
                msg.leg_fr.y = 0.09+r*y 
        
                msg.leg_bl.x = -0.09+r*x - s*cos(self.theta-pi/2*3)
                msg.leg_bl.y = -0.09+r*y

                msg.leg_br.x = 0.09+r*x  - s*cos(self.theta-pi/2*1)
                msg.leg_br.y = -0.09+r*y
            else:
                if vy > 0:
                    self.theta += self.theta_diff
                elif vy < 0:
                    self.theta -= self.theta_diff

                x, y, z  =  self.u(self.theta)

                msg.leg_fl.x = -0.09+r*x
                msg.leg_fl.y = 0.09+r*y - s*cos(self.theta-pi/2*0)

                msg.leg_fr.x =  0.09+r*x
                msg.leg_fr.y = 0.09+r*y - s*cos(self.theta-pi/2*2)
        
                msg.leg_bl.x = -0.09+r*x
                msg.leg_bl.y = -0.09+r*y - s*cos(self.theta-pi/2*3)

                msg.leg_br.x = 0.09+r*x
                msg.leg_br.y = -0.09+r*y - s*cos(self.theta-pi/2*1)
            
            msg.leg_fl.z = -0.10-r*z + h*sin(self.theta-pi/2*0)
            msg.leg_fr.z = -0.10-r*z + h*sin(self.theta-pi/2*2)
            msg.leg_bl.z = -0.10-r*z + h*sin(self.theta-pi/2*3)
            msg.leg_br.z = -0.10-r*z + h*sin(self.theta-pi/2*1)
            self.leg_point_pub_.publish(msg)
    
    def joy_cb(self, msg):
        self.joy = msg

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()