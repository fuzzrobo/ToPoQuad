# -*- coding: utf-8 -*-
import time
from math import pi

import rclpy

from geometry_msgs.msg import Point
from numpy import cos, sin
from topoquad_msgs.msg import QuadRobotLeg

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("walk_sample")
    pub_legs_cmd = node.create_publisher(QuadRobotLeg, 'legs/command', 10)
    u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
    theta = 0.0
    r = float(node.declare_parameter("r", 0.00).value)  # 周回軌道の半径
    s = float(node.declare_parameter("s", 0.04).value)  # 左右方向の歩幅オフセット
    h = float(node.declare_parameter("h", 0.02).value)  # 上下方向の脚上げ高さ
    x_offset = float(node.declare_parameter("x_offset", 0.08).value)  # 足先軌道中心の x オフセット量
    y_offset = float(node.declare_parameter("y_offset", 0.09).value)  # 足先軌道中心の y オフセット量
    z_offset = float(node.declare_parameter("z_offset", 0.08).value)  # 足先軌道中心の z オフセット量
    node.get_logger().info(
        f"walk params: r={r:.3f}, s={s:.3f}, h={h:.3f}, x_offset={x_offset:.3f}, y_offset={y_offset:.3f}, z_offset={z_offset:.3f}"
    )

    while rclpy.ok():
        node.get_logger().info(f"theta: {theta}", throttle_duration_sec=1)

        theta += (2*pi)/500
        x, y, z  =  u(theta)

        msg = QuadRobotLeg()

        msg.point_fl = Point()
        msg.point_fl.x = -x_offset+r*x
        msg.point_fl.y =  y_offset+r*y -s*cos(theta-pi/2*0)
        msg.point_fl.z = -z_offset-r*z+ h*sin(theta-pi/2*0)
        msg.point_fr = Point()
        msg.point_fr.x =  x_offset+r*x
        msg.point_fr.y =  y_offset+r*y -s*cos(theta-pi/2*2)
        msg.point_fr.z = -z_offset-r*z+ h*sin(theta-pi/2*2)
        msg.point_bl = Point()
        msg.point_bl.x = -x_offset+r*x
        msg.point_bl.y = -y_offset+r*y -s*cos(theta-pi/2*3)
        msg.point_bl.z = -z_offset-r*z+ h*sin(theta-pi/2*3)
        msg.point_br = Point()
        msg.point_br.x =  x_offset+r*x
        msg.point_br.y = -y_offset+r*y -s*cos(theta-pi/2*1)
        msg.point_br.z = -z_offset-r*z+ h*sin(theta-pi/2*1)

        pub_legs_cmd.publish(msg)

        time.sleep(0.005)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
