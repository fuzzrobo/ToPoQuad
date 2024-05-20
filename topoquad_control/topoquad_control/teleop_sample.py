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
base_radius = 0.09
base_height = 0.10

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        # Parameters
        self.declare_parameter('angle_pan_limit', pi/4)
        self.angle_pan_limit = self.get_parameter('angle_pan_limit').value
        self.declare_parameter('angle_tilt_limit', pi/4)
        self.angle_tilt_limit = self.get_parameter('angle_tilt_limit').value
        
        # Variable
        self.phase_p = 0.0
        self.phase_r = 0.0
        self.phase_diff_max =  1.0 / 10
        self.joy = None
        
        # Publishers
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotCmdNeckAngle, 'neck/angle', 10)
        self.leg_point_pub_ = self.create_publisher(QuadRobotCmdLegPoint, 'legs/point', 10)
        
        # Subscribers
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        
        # Timer
        self.timer = self.create_timer(0.05, self.timer_cb)
    
    def joy_cb(self, msg):
        self.joy = msg
        self.joy.header.stamp = self.get_clock().now().to_msg()

    def timer_cb(self):
        self.get_logger().info('looping', throttle_duration_sec = 5)

        if self.joy == None:
            return
        time_diff = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9 \
                    - self.joy.header.stamp.sec - self.joy.header.stamp.nanosec * 1e-9
        if time_diff > 0.1:
            return

        # Neck
        pan = self.joy.axes[0] * self.angle_pan_limit
        tilt = self.joy.axes[4] * self.angle_tilt_limit
        if pan != 0 and tilt != 0:
            self.get_logger().info(f"pan: {pan}, tilt: {tilt}", throttle_duration_sec = 0.5)
        neck_cmd = QuadRobotCmdNeckAngle()
        neck_cmd.angle_pan = pan
        neck_cmd.angle_tilt = tilt
        self.neck_cmd_pub_.publish(neck_cmd)
        
        # Legs
        rot = self.joy.axes[0]
        vx = -self.joy.axes[6]
        vy = self.joy.axes[7]
        if vx != 0 or vy != 0 or rot != 0:
            self.get_logger().info(f"vx: {vx}, vy: {vy}, rot: {rot}", throttle_duration_sec = 0.5)
        point = QuadRobotCmdLegPoint()
        self.move_parallel(point, vx, vy)
        self.move_rotational(point, rot)
        self.leg_point_pub_.publish(point)
    
    def move_parallel(self, point, vx, vy): # あーリファクタしたいー
        u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]

        if vx != 0:
            self.phase_p += vx * self.phase_diff_max
            x, y, z  =  u( 2*pi*self.phase_p )

            point.leg_fl.x = -base_radius+r*x - s*cos( 2*pi*(self.phase_p-0/4) )
            point.leg_fl.y = +base_radius+r*y

            point.leg_fr.x =  base_radius+r*x - s*cos( 2*pi*(self.phase_p-2/4) )
            point.leg_fr.y = +base_radius+r*y 
    
            point.leg_bl.x = -base_radius+r*x - s*cos( 2*pi*(self.phase_p-3/4) )
            point.leg_bl.y = -base_radius+r*y

            point.leg_br.x = +base_radius+r*x - s*cos( 2*pi*(self.phase_p-1/4) )
            point.leg_br.y = -base_radius+r*y

        else:
            self.phase_p += vy * self.phase_diff_max
            x, y, z  =  u( 2*pi*self.phase_p )

            point.leg_fl.x = -base_radius+r*x
            point.leg_fl.y = +base_radius+r*y - s*cos( 2*pi*(self.phase_p-0/4) )

            point.leg_fr.x =  base_radius+r*x
            point.leg_fr.y = +base_radius+r*y - s*cos( 2*pi*(self.phase_p-2/4) )
    
            point.leg_bl.x = -base_radius+r*x
            point.leg_bl.y = -base_radius+r*y - s*cos( 2*pi*(self.phase_p-3/4) )

            point.leg_br.x = +base_radius+r*x
            point.leg_br.y = -base_radius+r*y - s*cos( 2*pi*(self.phase_p-1/4) )

        point.leg_fl.z = -base_height-r*z + h*sin( 2*pi*(self.phase_p-0/4) )
        point.leg_fr.z = -base_height-r*z + h*sin( 2*pi*(self.phase_p-2/4) )
        point.leg_bl.z = -base_height-r*z + h*sin( 2*pi*(self.phase_p-3/4) )
        point.leg_br.z = -base_height-r*z + h*sin( 2*pi*(self.phase_p-1/4) )

        if self.phase_p > 1:
            self.phase_p -= 1
        elif self.phase_p < 0:
            self.phase_p += 1

        return point

    def move_rotational(self, point, rot): 

        return point



def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()