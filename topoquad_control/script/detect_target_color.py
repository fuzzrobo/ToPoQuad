#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import time


def callback_img(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv_img, lower_target_color1, upper_target_color1)
    mask2 = cv2.inRange(hsv_img, lower_target_color2, upper_target_color2)
    mask = mask1 + mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)!=0:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        p = Pose2D()
        p.x = ((x+w/2) - msg.width /2) / (msg.width /2)  # Center of FoV is 0,
        p.y = ((y+h/2) - msg.height/2) / (msg.height/2) # the rihgt edge is +1
        p.theta = 0
        pub_target.publish(p)

if __name__=="__main__":
    rospy.init_node('detect_target_color_node')

    lower_target_color1 = np.array(rospy.get_param('~hsv_mask1_lower', [  0,  50,   0]))
    upper_target_color1 = np.array(rospy.get_param('~hsv_mask1_upper', [ 10, 255, 255]))
    lower_target_color2 = np.array(rospy.get_param('~hsv_mask2_lower', [160,  50,   0]))
    upper_target_color2 = np.array(rospy.get_param('~hsv_mask2_upper', [180, 255, 255]))

    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)
    pub_target = rospy.Publisher('/target_position/ratio', Pose2D, queue_size=10)
    rospy.spin()
