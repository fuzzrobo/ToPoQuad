#!/usr/bin/env python3

import rospy
from topoquad_master.msg import QuadRobotCmdLegAngle

import time

if __name__=="__main__":
    rospy.init_node('topoquad_control_node')

    pub_legs_cmd = rospy.Publisher('/spider/cmd/leg_angle', QuadRobotCmdLegAngle, queue_size=10)

    time.sleep(1)

    while rospy.is_shutdown() is False:
        msg = QuadRobotCmdLegAngle()

        msg.angles_FL = [ 0.6, 0.40, 0.75]
        msg.angles_FR = [-0.3, 0.75, 0.75]
        msg.angles_BR = [ 0.3, 0.75, 0.75]
        msg.angles_BL = [ 0.6, 0.75, 0.75]
        pub_legs_cmd.publish(msg)
        time.sleep(0.5)

        msg.angles_FL = [ 0.3, 0.75, 0.75]
        msg.angles_FR = [ 0.6, 0.40, 0.75]
        msg.angles_BR = [ 0.0, 0.75, 0.75]
        msg.angles_BL = [ 0.3, 0.75, 0.75]
        pub_legs_cmd.publish(msg)
        time.sleep(0.5)

        msg.angles_FL = [ 0.0, 0.75, 0.75]
        msg.angles_FR = [ 0.3, 0.75, 0.75]
        msg.angles_BR = [ 0.9, 0.40, 0.75]
        msg.angles_BL = [ 0.0, 0.75, 0.75]
        pub_legs_cmd.publish(msg)
        time.sleep(0.5)

        msg.angles_FL = [-0.3, 0.75, 0.75]
        msg.angles_FR = [ 0.0, 0.75, 0.75]
        msg.angles_BR = [ 0.6, 0.75, 0.75]
        msg.angles_BL = [ 0.6, 0.60, 0.75]
        pub_legs_cmd.publish(msg)
        time.sleep(0.5)

    rospy.spin()
    
