# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from topoquad_msgs.msg import QuadRobotCmdNeckAngle, QuadRobotCmdLegPoint
from geometry_msgs.msg import Point

from math import pi, sin, cos, sqrt

# Static
r = 0.00
s = 0.02
h = 0.01

base_radius = 0.085
base_height = 0.1
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
        self.phase_diff =  2.0 / 50 
        self.theta_diff =  (2 * pi) / 20
        self.joy = None
        self.u = lambda theta: [1.0 * cos(theta), 1.0 * sin(theta), 0*sin(theta/2)]
        
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
        pan = self.joy.axes[3] * self.angle_pan_limit
        tilt = self.joy.axes[4] * self.angle_tilt_limit
        neck_cmd = QuadRobotCmdNeckAngle()
        if pan != 0 or tilt != 0:
            self.get_logger().info(f"pan: {pan}, tilt: {tilt}", throttle_duration_sec = 0.5)
            neck_cmd.angle_pan = pan
            neck_cmd.angle_tilt = tilt
        self.neck_cmd_pub_.publish(neck_cmd)
        
        # Legs
        rot = -1.0 if self.joy.buttons[4] else (1.0 if self.joy.buttons[5] else 0.0)
        vx = -self.joy.axes[6]
        vy = self.joy.axes[7]
        point = QuadRobotCmdLegPoint()
        if rot != 0:
            self.get_logger().info(f"rot: {rot}", throttle_duration_sec = 0.5)
            self.move_rotational(point, rot)
        else:
            self.get_logger().info(f"vx: {vx}, vy: {vy}", throttle_duration_sec = 0.5)
            self.move_parallel_old(point, vx, vy)
        if vx or vy or rot:
            self.leg_point_pub_.publish(point)
    
    def move_parallel(self, point, vx, vy):
        body_motion = lambda time: [
            0.0, # r * clamp(min(-1+8*time,+7-8*time), -1, 1) , 
            0.0, # r * - clamp(min(+1-8*time,-5+8*time), -1, 1) , 
            0.0 
        ] # 重心位置
        leg_motion = lambda time: [ # 足先の軌道, 足先ベクトルを返すthetaの関数として歩行軌道を定義        
            vx * s*cos(2*pi*time) / (1 if vx**2 + vy**2 < 1 else sqrt(vx**2 + vy**2)), # [vx, vy]ベクトルの正規化
            vy * s*cos(2*pi*time) / (1 if vx**2 + vy**2 < 1 else sqrt(vx**2 + vy**2)), # [vx, vy]ベクトルの正規化
            h * min(sin(2*pi*time), 0) # 半円状の動作を作り，円弧を描く0~piの範囲を遊脚期とする
        ]
        a = 0.6 # 支持脚期の時間的な割有
        time_traj = lambda phase: max(norm(phase)/(2*a), 1/2+(norm(phase)-a)/(2*(1-a)))

        phase = norm(self.phase_p - self.phase_diff) # 0~1の範囲に収めつつ，位相を更新
        x0, y0, z0 = body_motion((phase))
        x, y, z  =  leg_motion( time_traj(phase-0/4) )
        point.leg_fl.x = -base_radius - x0 - x 
        point.leg_fl.y = +base_radius - y0 - y 
        point.leg_fl.z = -base_height - z0 - z 
        x, y, z  =  leg_motion( time_traj(phase-2/4) )
        point.leg_fr.x =  base_radius - x0 - x
        point.leg_fr.y = +base_radius - y0 - y
        point.leg_fr.z = -base_height - z0 - z
        x, y, z  =  leg_motion( time_traj(phase-3/4) )
        point.leg_bl.x = -base_radius - x0 - x
        point.leg_bl.y = -base_radius - y0 - y
        point.leg_bl.z = -base_height - z0 - z
        x, y, z  =  leg_motion( time_traj(phase-1/4) )
        point.leg_br.x = +base_radius - x0 - x
        point.leg_br.y = -base_radius - y0 - y
        point.leg_br.z = -base_height - z0 - z

        self.phase_p = phase
        return point
    
    def move_parallel_old(self, point, vx, vy):
        if vx != 0:
            if vx > 0:
                self.phase_p += self.theta_diff
            elif vx < 0:
                self.phase_p -= self.theta_diff
            x, y, z  =  self.u(self.phase_p)

            point.leg_fl.x = -0.09+r*x - s*cos(self.phase_p-pi/2*0)
            point.leg_fl.y = 0.09+r*y

            point.leg_fr.x =  0.09+r*x - s*cos(self.phase_p-pi/2*2)
            point.leg_fr.y = 0.09+r*y 
    
            point.leg_bl.x = -0.09+r*x - s*cos(self.phase_p-pi/2*3)
            point.leg_bl.y = -0.09+r*y

            point.leg_br.x = 0.09+r*x  - s*cos(self.phase_p-pi/2*1)
            point.leg_br.y = -0.09+r*y
        else:
            if vy > 0:
                self.phase_p += self.theta_diff
            elif vy < 0:
                self.phase_p -= self.theta_diff

            x, y, z  =  self.u(self.phase_p)

            point.leg_fl.x = -0.09+r*x
            point.leg_fl.y = 0.09+r*y - s*cos(self.phase_p-pi/2*0)

            point.leg_fr.x =  0.09+r*x
            point.leg_fr.y = 0.09+r*y - s*cos(self.phase_p-pi/2*2)
    
            point.leg_bl.x = -0.09+r*x
            point.leg_bl.y = -0.09+r*y - s*cos(self.phase_p-pi/2*3)

            point.leg_br.x = 0.09+r*x
            point.leg_br.y = -0.09+r*y - s*cos(self.phase_p-pi/2*1)
        
        point.leg_fl.z = -0.10-r*z + h*sin(self.phase_p-pi/2*0)
        point.leg_fr.z = -0.10-r*z + h*sin(self.phase_p-pi/2*2)
        point.leg_bl.z = -0.10-r*z + h*sin(self.phase_p-pi/2*3)
        point.leg_br.z = -0.10-r*z + h*sin(self.phase_p-pi/2*1)
        return point

    def move_rotational(self, point, rot): 
        body_motion = lambda time: [ 0.0, 0.0, 0.0 ] # 重心位置
        leg_motion = lambda time: [ # 足先の軌道, 足先ベクトルを返すtimeの関数として歩行軌道を定義
            +rot*s*cos(2*pi*time), 
            -rot*s*cos(2*pi*time), 
            h * min(sin(2*pi*time), 0.0) # 0~piの範囲が遊脚期
        ]
        a = 3/4 # 支持脚期の割有
        time_traj = lambda phase: max(norm(phase)/(2*a), 1/2+(norm(phase)-a)/(2*(1-a)))

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