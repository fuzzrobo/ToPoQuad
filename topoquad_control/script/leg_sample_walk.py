#!/usr/bin/env python3

import rospy
from topoquad_master.msg import QuadRobotCmdLegPoint
from geometry_msgs.msg import Point
import numpy as np
from numpy import sin, cos
from math import pi
import time

if __name__=="__main__":
    rospy.init_node('topoquad_control_node')

    pub_legs_cmd = rospy.Publisher('/spider/cmd/leg_point', QuadRobotCmdLegPoint, queue_size=10)

    u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
    theta = 0.0
    r = 0.00
    s = 0.04
    h = 0.02

    while rospy.is_shutdown() is False:

        theta += (2*pi)/500
        x, y, z  =  u(theta)

        msg = QuadRobotCmdLegPoint()

        msg.leg_FL = Point(-0.09+r*x, 0.09+r*y -s*cos(theta-pi/2*0), -0.08-r*z+ h*sin(theta-pi/2*0)) 
        msg.leg_FR = Point( 0.09+r*x, 0.09+r*y -s*cos(theta-pi/2*2), -0.08-r*z+ h*sin(theta-pi/2*2)) 
        msg.leg_BL = Point(-0.09+r*x,-0.09+r*y -s*cos(theta-pi/2*3), -0.08-r*z+ h*sin(theta-pi/2*3)) 
        msg.leg_BR = Point( 0.09+r*x,-0.09+r*y -s*cos(theta-pi/2*1), -0.08-r*z+ h*sin(theta-pi/2*1)) 

        pub_legs_cmd.publish(msg)

        time.sleep(0.005)
    
