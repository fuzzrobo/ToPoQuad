# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from topoquad_msgs.msg import QuadRobotLeg
from geometry_msgs.msg import Point
from numpy import sin, cos
from math import pi
import time

import inspect

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("walk_sample")

    pub_legs_cmd = node.create_publisher(QuadRobotLeg, 'legs/command', 10)

    u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
    theta = 0.0
    r = 0.00
    s = 0.04
    h = 0.02

    while rclpy.ok():
        node.get_logger().info(f"theta: {theta}", throttle_duration_sec=1)

        theta += (2*pi)/500
        x, y, z  =  u(theta)

        msg = QuadRobotLeg()

        msg.point_fl = Point()
        msg.point_fl.x = -0.08+r*x
        msg.point_fl.y =  0.09+r*y -s*cos(theta-pi/2*0)
        msg.point_fl.z = -0.08-r*z+ h*sin(theta-pi/2*0) 
        msg.point_fr = Point()
        msg.point_fr.x =  0.08+r*x
        msg.point_fr.y =  0.09+r*y -s*cos(theta-pi/2*2)
        msg.point_fr.z = -0.08-r*z+ h*sin(theta-pi/2*2) 
        msg.point_bl = Point()
        msg.point_bl.x = -0.08+r*x
        msg.point_bl.y = -0.09+r*y -s*cos(theta-pi/2*3)
        msg.point_bl.z = -0.08-r*z+ h*sin(theta-pi/2*3) 
        msg.point_br = Point()
        msg.point_br.x =  0.08+r*x
        msg.point_br.y = -0.09+r*y -s*cos(theta-pi/2*1)
        msg.point_br.z = -0.08-r*z+ h*sin(theta-pi/2*1) 

        pub_legs_cmd.publish(msg)

        time.sleep(0.005)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
