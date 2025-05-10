#ifndef COMMON_H_
#define COMMON_H_

#define rad2deg (180.0 / M_PI)
#define deg2rad (M_PI / 180.0)

#include <cmath>
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <map>

using Eigen::Vector3d;

class Joint {
   public:
    Joint(int id) : servo_angle_(0.0), joint_angle_(0.0), gear_ratio_(1.0), servo_current_(0.0), joint_torque_(0.0), torque_ratio_(1.0), servo_velocity_(0.0), id_(id), fixed_coord_(Vector3d(0.0, 0.0, 0.0)) {}
    Joint(int id, double gear_ratio, double joint_angle, double torque_ratio, double joint_torque) : joint_angle_(joint_angle), servo_angle_(joint_angle * gear_ratio), servo_velocity_(0.0), gear_ratio_(gear_ratio), joint_torque_(joint_torque), servo_current_(joint_torque / gear_ratio / torque_ratio), torque_ratio_(torque_ratio), id_(id), fixed_coord_(Vector3d(0.0, 0.0, 0.0)) {}
    // 角度の入力
    void SetJointAngle(double angle) {
        joint_angle_ = angle;
        servo_angle_ = angle * gear_ratio_;  // todo 可動域の制限など
    }
    void SetServoAngle(double angle) {
        servo_angle_ = angle;
        joint_angle_ = angle / gear_ratio_;
    }

    // トルクの入力
    void SetJointTorque(double torque) {
        joint_torque_ = torque;
        servo_current_ = torque / gear_ratio_ / torque_ratio_;  // todo 可動域の制限など
    }
    void SetServoCurrent(double current) {
        servo_current_ = current;
        joint_torque_ = current * torque_ratio_ * gear_ratio_;
    }

    double servo_velocity_;  //[rad/s]

    double servo_angle_;  //[rad]
    double joint_angle_;  //[rad] // サーボと関節の角度,ギア比分だけ異なる

    double servo_current_;  //[mA] //サーボの電流
    double joint_torque_;   //[Nm] //関節のトルク

    int id_;                // DynamixelのID, 固定値
    double gear_ratio_;     // Dynamixelと関節のギア比 (逆転は負の値)
    double torque_ratio_;   // [Nm/mA]// Dynamixelの電流とトルクの比
    Vector3d fixed_coord_;  // [m]  // 直前の関節座標系から見たこの関節座標系の原点の位置ベクトル 軸方向がx軸，サーボ回転軸がy軸，z軸は右手系.
};

#endif  // COMMON_H_
