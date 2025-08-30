# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from topoquad_msgs.msg import QuadRobotNeck, QuadRobotLeg
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
        self.n = 0
        self.leg = 0
        self.theta = 0.0
        self.theta_diff =  (2 * pi) / 20
        self.joy = None
        self.u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
        self.legs = [[0.]*3, [0.]*3, [0.]*3, [0.]*3]
        
        # Publishers
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotNeck, 'neck/command', 10)
        self.leg_cmd_pub_ = self.create_publisher(QuadRobotLeg, 'legs/command', 10)
        
        # Subscribers
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        
        # Timer
        self.timer = self.create_timer(0.05, self.timer_cb)
        
        self.move(0.0, 0.0)
        
    def timer_cb(self):
        if self.joy != None:
            # Neck
            neck_cmd = QuadRobotNeck()
            neck_cmd.angle_pan = self.joy.axes[3] * self.angle_pan_limit
            neck_cmd.angle_tilt = self.joy.axes[4] * self.angle_tilt_limit
            self.neck_cmd_pub_.publish(neck_cmd)
            
            # Legs
            vx = -self.joy.axes[6]
            vy = self.joy.axes[7]
            
            if self.walk(self.leg, 0.02, 0.02):
                self.leg += 1
            if self.leg > 3:
                self.leg = 0
                self.move(-0.02, -0.02)
            self.publish()
            print(self.legs)

    
    def joy_cb(self, msg):
        self.joy = msg
        
    def move(self, vx, vy):
        for l in self.legs:
            l[0] -= vx
            l[1] -= vy
            
    def walk(self, i, dx, dy):
        self.n += 1
        if self.n == 1:
            self.legs[i][0] -= dx / 2
            self.legs[i][1] -= dy / 2
            self.legs[i][2] = 0.01
            return False
        elif self.n == 21:
            self.legs[i][0] -= dx / 2
            self.legs[i][1] -= dy / 2
            self.legs[i][2] = 0.0
            return False
        elif self.n == 41:
            self.n = 0
            return True
        
    def publish(self):
        msg = QuadRobotLeg()
        
        msg.point_fl.x = -0.09 + self.legs[0][0]
        msg.point_fl.y = 0.09 + self.legs[0][1]
        msg.point_fl.z = -0.10 + self.legs[0][2]

        msg.point_fr.x =  0.09 + self.legs[1][0]
        msg.point_fr.y = 0.09 + self.legs[1][1]
        msg.point_fr.z = -0.10 + self.legs[1][2]
        
        msg.point_br.x = 0.09 + self.legs[2][0]
        msg.point_br.y = -0.09 + self.legs[2][1]
        msg.point_br.z = -0.10 + self.legs[2][2]

        msg.point_bl.x = -0.09 + self.legs[3][0]
        msg.point_bl.y = -0.09 + self.legs[3][1]
        msg.point_bl.z = -0.10 + self.legs[3][2]
        self.leg_cmd_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
