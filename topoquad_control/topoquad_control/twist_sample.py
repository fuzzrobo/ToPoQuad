# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from topoquad_msgs.msg import QuadRobotNeck, QuadRobotLeg
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from math import pi, sin, cos, sqrt, hypot

# Static
r = 0.025
s = 0.030
h = 0.025
base_radius = 0.085
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
        self.phase_diff =  8.0 / 1000
        self.twist = None
        self.vx  = 0.0
        self.vy  = 0.0
        self.rot = 0.0
        
        # Publishers
        self.leg_cmd_pub_ = self.create_publisher(QuadRobotLeg, 'legs/command', 10)
        self.debug_pub_ = self.create_publisher(Float64MultiArray, 'debug', 10)
        
        # Subscribers
        self.vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 10)
        
        # Timer
        self.timer = self.create_timer(0.02, self.timer_cb)
    
    def vel_cb(self, msg):
        self.twist = msg
        # self.twist.header.stamp = self.get_clock().now().to_msg()

    def timer_cb(self):
        self.get_logger().info('looping', throttle_duration_sec = 5)

        if self.twist == None:
            self.twist = Twist()

        vx_ = clamp(-self.twist.linear.y, -1.8, 1.8)
        vy_ = clamp( self.twist.linear.x, -1.8, 1.8)
        rot_= clamp( self.twist.angular.z, -1.5, 1.5)
        if vx_==0.0 and vy_==0.0 and rot_==0.0:
            return

        vx  = vx_ if vx_==0 else sqrt(abs(vx_)) * vx_ / abs(vx_)
        vy  = vy_ if vy_==0 else sqrt(abs(vy_)) * vy_ / abs(vy_)
        rot = rot_ if rot_==0 else rot_ * abs(rot_) / hypot(rot_, 2*hypot(vx, vy))  
        cmd = QuadRobotLeg()
        cmd.torques_fr = [0.7, 0.7, 0.7]
        cmd.torques_fl = [0.7, 0.7, 0.7]
        cmd.torques_br = [0.7, 0.7, 0.7]
        cmd.torques_bl = [0.7, 0.7, 0.7]
        self.move_pal_rot(cmd, vx, vy, rot)
        self.leg_cmd_pub_.publish(cmd)
    
    def move_pal_rot(self, cmd, vx_, vy_, rot_):
        vx= 0.93*self.vx if vx_==0 else self.vx+ (-abs(vx_) if self.vx-vx_ > -0.001 else abs(vx_) if self.vx-vx_< 0.001 else 0)/50
        vy= 0.93*self.vy if vy_==0 else self.vy+ (-abs(vy_) if self.vy-vy_ > -0.001 else abs(vy_) if self.vy-vy_< 0.001 else 0)/50
        rot= 0.93*self.rot if rot_==0 else self.rot+ (-abs(rot_) if self.rot-rot_ > -0.001 else abs(rot_) if self.rot-rot_< 0.001 else 0)/50
        V = max(0.01, hypot(vx, vy))
        R = max(0.01, abs(rot) )
        rot_dir = -1 if rot>0 else 1
        self.get_logger().info(f"vx: {vx}, vy: {vy}, rot: {rot_dir}", throttle_duration_sec = 1.0)

        body_motion = lambda time: [
            r * cos(2*pi*(time+1/4)) * sqrt(hypot(vx, vy)),
            r * sin(2*pi*(time+1/4)) * sqrt(hypot(vx, vy)) - 0.01,
            -0.01
        ] # 重心位置
        leg_motion_paralell = lambda time: [ # 足先の軌道, 足先ベクトルを返すthetaの関数として歩行軌道を定義        
            s * cos(2*pi*time) * vx / V, # [vx, vy]ベクトルの正規化
            s * cos(2*pi*time) * vy / V, # [vx, vy]ベクトルの正規化
            h * min(sin(2*pi*time), 0.0)
        ] # 前進
        leg_motion_rotation = lambda time: [ # 足先の軌道, 足先ベクトルを返すtimeの関数として歩行軌道を定義
            +s*cos(2*pi*time) * rot * 0.8, 
            -s*cos(2*pi*time) * rot * 0.8, 
            h * min(sin(2*pi*time), 0.0)
        ] # 旋回
        a = 0.8 # 支持脚期の時間的な割有
        time_traj = (lambda phase:max(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))) 
                        if a>0.5 else min(norm(phase)/(2*a), 1+(norm(phase)-1)/(2*(1-a))))

        phase = norm(self.phase_p - hypot(rot, hypot(vx, vy))*self.phase_diff) # 0~1の範囲に収めつつ，位相を更新
        x0, y0, z0 = body_motion(((phase+1/8-(1-a)/2 + 0.04)*rot_dir))
        xp, yp, zp  =  leg_motion_paralell( time_traj(phase-1.5/4*rot_dir) )
        xr, yr, zr  =  leg_motion_rotation( time_traj(phase-1.5/4*rot_dir*abs(rot)/R) )
        # self.debug_pub_.publish(Float64MultiArray(data=[(-zp * V - zr * R) / hypot(R, V)] ))
        cmd.point_fr.x = +base_radius - x0 + (-xp * V + xr * R) / hypot(R, V) # 右＋　左足なので+
        cmd.point_fr.y = +base_radius - y0 + (-yp * V + yr * R) / hypot(R, V) # 前＋
        cmd.point_fr.z = -base_height - z0 + (-zp * V - zr * R) / hypot(R, V) # 上＋
        xp, yp, zp  =  leg_motion_paralell( time_traj(phase+1.5/4*rot_dir) )
        xr, yr, zr  =  leg_motion_rotation( time_traj(phase+1.5/4*rot_dir*abs(rot)/R) )
        cmd.point_fl.x = -base_radius - x0 + (-xp * V + xr * R) / hypot(R, V) # 右＋　左足なので-
        cmd.point_fl.y = +base_radius - y0 + (-yp * V - yr * R) / hypot(R, V) # 前＋
        cmd.point_fl.z = -base_height - z0 + (-zp * V - zr * R) / hypot(R, V) # 上＋
        xp, yp, zp  =  leg_motion_paralell( time_traj(phase-0.5/4*rot_dir) )
        xr, yr, zr  =  leg_motion_rotation( time_traj(phase-0.5/4*rot_dir*abs(rot)/R) )
        cmd.point_br.x = +base_radius - x0 + (-xp * V - xr * R) / hypot(R, V) # 右＋　右足なので+
        cmd.point_br.y = -base_radius - y0 + (-yp * V + yr * R) / hypot(R, V) # 前＋
        cmd.point_br.z = -base_height - z0 + (-zp * V - zr * R) / hypot(R, V) # 上＋
        xp, yp, zp  =  leg_motion_paralell( time_traj(phase+0.5/4*rot_dir) )
        xr, yr, zr  =  leg_motion_rotation( time_traj(phase+0.5/4*rot_dir*abs(rot)/R) )
        cmd.point_bl.x = -base_radius - x0 + (-xp * V - xr * R) / hypot(R, V) # 右＋　左足なので-
        cmd.point_bl.y = -base_radius - y0 + (-yp * V - yr * R) / hypot(R, V) # 前＋
        cmd.point_bl.z = -base_height - z0 + (-zp * V - zr * R) / hypot(R, V) # 上＋

        self.phase_p = phase
        self.vx = vx 
        self.vy = vy 
        self.rot= rot
        return cmd

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
