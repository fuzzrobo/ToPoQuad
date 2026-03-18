#ifndef TOPOQUAD_MASTER_SRC_COMMON_HPP_
#define TOPOQUAD_MASTER_SRC_COMMON_HPP_

#include <cstdint>
#include <cmath>

inline constexpr double pi = 3.14159265358979323846;
inline constexpr double rad_to_deg = 180.0 / pi;
inline constexpr double deg_to_rad = pi / 180.0;

class Joint {
 public:
  explicit Joint(uint8_t id) : servo_velocity_(0.0), servo_angle_(0.0), joint_angle_(0.0), servo_current_(0.0), joint_torque_(0.0), id_(id), gear_ratio_(1.0), torque_ratio_(1.0) {}

  Joint(uint8_t id, double gear_ratio, double joint_angle, double torque_ratio, double joint_torque)
      : servo_velocity_(0.0), servo_angle_(joint_angle * gear_ratio), joint_angle_(joint_angle), servo_current_(joint_torque / gear_ratio / torque_ratio), joint_torque_(joint_torque), id_(id), gear_ratio_(gear_ratio), torque_ratio_(torque_ratio) {}

  void SetJointAngle(double angle) {
    joint_angle_ = angle;
    servo_angle_ = angle * gear_ratio_;
  }

  void SetServoAngle(double angle) {
    servo_angle_ = angle;
    joint_angle_ = angle / gear_ratio_;
  }

  void SetJointTorque(double torque) {
    joint_torque_  = torque;
    servo_current_ = torque / gear_ratio_ / torque_ratio_;
  }

  void SetServoCurrent(double current) {
    servo_current_ = current;
    joint_torque_  = current * torque_ratio_ * gear_ratio_;
  }

  double servo_velocity_;
  double servo_angle_;
  double joint_angle_;
  double servo_current_;
  double joint_torque_;

  uint8_t id_;
  double gear_ratio_;
  double torque_ratio_;
};

#endif  // TOPOQUAD_MASTER_SRC_COMMON_HPP_
