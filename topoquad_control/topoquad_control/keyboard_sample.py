# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from topoquad_msgs.msg import QuadRobotCmdNeckAngle, QuadRobotCmdLegPoint
from geometry_msgs.msg import Twist

from math import pi, sin, cos, sqrt, hypot

# Static
r = 0.025
s = 0.040
h = 0.022
base_radius = 0.080
base_height = 0.100
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
        self.phase_diff =  4.0 / 1000 
        self.twist = None
        
        # Publishers
        self.neck_cmd_pub_ = self.create_publisher(QuadRobotCmdNeckAngle, 'neck/angle', 10)
        self.leg_point_pub_ = self.create_publisher(QuadRobotCmdLegPoint, 'legs/point', 10)
        
        # Subscribers
        self.vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 10)
        
        # Timer
        self.timer = self.create_timer(0.01, self.timer_cb)
    
    def vel_cb(self, msg):
        self.twist = msg
        # self.twist.header.stamp = self.get_clock().now().to_msg()

    def timer_cb(self):
        self.get_logger().info('looping', throttle_duration_sec = 5)

        if self.twist == None:
            self.twist = Twist()

        # Legs
        rot = clamp(-self.twist.angular.z, -1.5, 1.5)
        vx_  = clamp(-self.twist.linear.y, -1.8, 1.8)
        vy_  = clamp( self.twist.linear.x, -1.8, 1.8)
        vx = vx_ if vx_==0 else sqrt(abs(vx_)) * vx_ / abs(vx_)
        vy = vy_ if vy_==0 else sqrt(abs(vy_)) * vy_ / abs(vy_)
        point = QuadRobotCmdLegPoint()
        if rot != 0:
            self.get_logger().info(f"rot: {rot}", throttle_duration_sec = 0.5)
            self.move_rotational(point, rot)
        else:
            self.get_logger().info(f"vx: {vx}, vy: {vy}", throttle_duration_sec = 0.5)
            self.move_parallel(point, vx, vy)
        if vx or vy or rot:
            self.leg_point_pub_.publish(point)
    
    def move_parallel(self, point, vx, vy):
        body_motion = lambda time: [
            1.0 * r*cos(2*pi*(time+1/8)) * sqrt(hypot(vx, vy)),
            1.0 * r*sin(2*pi*(time+1/8)) * sqrt(hypot(vx, vy)),
            -0.01
        ] # 重心位置
        leg_motion = lambda time: [ # 足先の軌道, 足先ベクトルを返すthetaの関数として歩行軌道を定義        
            vx * s*cos(2*pi*time) / (1 if hypot(vx, vy) < 1 else hypot(vx, vy)), # [vx, vy]ベクトルの正規化
            vy * s*cos(2*pi*time) / (1 if hypot(vx, vy) < 1 else hypot(vx, vy)), # [vx, vy]ベクトルの正規化
            h * min(sin(2*pi*time), 0) # 半円状の動作を作り，円弧となるtime \in [0.5, 1]が遊脚期 [0, 0.5]が支持脚期とする。
        ]
        a = 0.8 # 支持脚期の時間的な割有
        time_traj = (lambda phase:max(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))) 
                    if a>0.5 else min(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))))
        
        phase = norm(self.phase_p - hypot(vx, vy)*self.phase_diff) # 0~1の範囲に収めつつ，位相を更新
        x0, y0, z0 = body_motion((phase+2/4+1/16))
        x, y, z  =  leg_motion( time_traj(phase-0/4) )
        point.leg_fr.x = +base_radius - x0 - x # 右＋　左足なので+
        point.leg_fr.y = +base_radius - y0 - y # 前＋
        point.leg_fr.z = -base_height - z0 - z # 上＋
        x, y, z  =  leg_motion( time_traj(phase-1/4) )
        point.leg_fl.x = -base_radius - x0 - x # 右＋　左足なので-
        point.leg_fl.y = +base_radius - y0 - y # 前＋
        point.leg_fl.z = -base_height - z0 - z # 上＋
        x, y, z  =  leg_motion( time_traj(phase-3/4) )
        point.leg_br.x = +base_radius - x0 - x # 右＋　右足なので+
        point.leg_br.y = -base_radius - y0 - y # 前＋
        point.leg_br.z = -base_height - z0 - z # 上＋
        x, y, z  =  leg_motion( time_traj(phase-2/4) )
        point.leg_bl.x = -base_radius - x0 - x # 右＋　左足なので-
        point.leg_bl.y = -base_radius - y0 - y # 前＋
        point.leg_bl.z = -base_height - z0 - z # 上＋

        self.phase_p = phase
        return point

    def move_rotational(self, point, rot): 
        body_motion = lambda time: [ 0.0, 0.0, 0.0 ] # 重心位置
        leg_motion = lambda time: [ # 足先の軌道, 足先ベクトルを返すtimeの関数として歩行軌道を定義
            +rot*s*cos(2*pi*time), 
            -rot*s*cos(2*pi*time), 
            h * min(sin(2*pi*time), 0.0) # 0~piの範囲が遊脚期
        ]
        a = 3/4 # 支持脚期の割有
        time_traj = (lambda phase:max(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))) 
                    if a>0.5 else min(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))))

        phase = norm(self.phase_r + self.phase_diff) # 0~1の範囲に収めつつ，位相を更新
        x0, y0, z0 = body_motion((phase))
        x, y, z  =  leg_motion( time_traj(phase - 0/4*(-1 if rot>0 else 1)) )
        point.leg_fl.x = -base_radius - x0 + x
        point.leg_fl.y = +base_radius - y0 - y
        point.leg_fl.z = -base_height - z0 - z
        x, y, z  =  leg_motion( time_traj(phase - 3/4*(-1 if rot>0 else 1)) )
        point.leg_fr.x =  base_radius - x0 + x
        point.leg_fr.y = +base_radius - y0 + y
        point.leg_fr.z = -base_height - z0 - z
        x, y, z  =  leg_motion( time_traj(phase - 1/4*(-1 if rot>0 else 1)) )
        point.leg_bl.x = -base_radius - x0 - x
        point.leg_bl.y = -base_radius - y0 - y
        point.leg_bl.z = -base_height - z0 - z
        x, y, z  =  leg_motion( time_traj(phase - 2/4*(-1 if rot>0 else 1)) )
        point.leg_br.x = +base_radius - x0 - x
        point.leg_br.y = -base_radius - y0 + y
        point.leg_br.z = -base_height - z0 - z
        
        self.phase_r = phase
        return point

def norm(phase):
    while phase > 1:
        phase -= 1
    while phase < 0:
        phase += 1
    return phase

def clamp(val, min_val, max_val):
    return min(max(val, min_val), max_val)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()