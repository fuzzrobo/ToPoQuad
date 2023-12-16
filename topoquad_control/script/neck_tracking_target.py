#!/usr/bin/env python3

import rospy
import numpy as np
import math
from topoquad_master.msg import QuadRobotCmdNeckAngle
from topoquad_master.msg import QuadRobotStateNeck
from geometry_msgs.msg import Pose2D
from types import SimpleNamespace
import time

now_pos = SimpleNamespace(pan=0.0, tilt=0.0)
pre = SimpleNamespace(x=0.0, y=0.0, sum_x=0, sum_y=0)

def callback_target(msg):
    x = 0.8 * pre.x + 0.2 * max(-1, min(msg.x, 1)) # smoothing
    y = 0.8 * pre.y + 0.2 * max(-1, min(msg.y, 1)) # smoothing
    dx = x - pre.x
    dy = y - pre.y
    sum_x = 0.9*pre.sum_x + x
    sum_y = 0.9*pre.sum_y + y
    pre.x = x
    pre.y = y
    pre.sum_x = sum_x
    pre.sum_y = sum_y

    #pid control
    next_pos_x = now_pos.pan - (p_gain * x + d_gain * dx + i_gain * sum_x)
    next_pos_y = now_pos.tilt - (p_gain * y + d_gain * dy + i_gain * sum_y)

    pantilt_cmd = QuadRobotCmdNeckAngle()
    pantilt_cmd.angle_pan  = max( pan_joint_limit['min'], min(next_pos_x,  pan_joint_limit['max']))
    pantilt_cmd.angle_tilt = max(tilt_joint_limit['min'], min(next_pos_y, tilt_joint_limit['max']))
    pub_pantilt.publish(pantilt_cmd)

def callback_joint(msg):
    now_pos.pan = msg.angle_pan
    now_pos.tilt = msg.angle_tilt

if __name__=="__main__":
    rospy.init_node('tracking_target_node')

    pan_joint_limit = rospy.get_param('~pan_joint_limit', {'min':-math.pi, 'max':math.pi})
    tilt_joint_limit = rospy.get_param('~tilt_joint_limit', {'min':-math.pi, 'max':math.pi})
    p_gain = rospy.get_param('~p_gain', 1.0)
    d_gain = rospy.get_param('~d_gain', 0.05)
    i_gain = rospy.get_param('~i_gain', 0.01)
    
    rospy.Subscriber('/target_position/ratio', Pose2D, callback_target)
    rospy.Subscriber('/pantilt/state', QuadRobotStateNeck, callback_joint)
    pub_pantilt = rospy.Publisher('/pantilt/cmd', QuadRobotCmdNeckAngle, queue_size=10)
    rospy.spin()
    cv2.destroyAllWindows()
