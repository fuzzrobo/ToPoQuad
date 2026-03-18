#ifndef TOPOQUAD_MASTER_SRC_NECK_NODE_HPP_
#define TOPOQUAD_MASTER_SRC_NECK_NODE_HPP_

#include <cmath>

#include "common.hpp"

class Neck {
 public:
  Neck() : pan_(1), tilt_(2) {}

  void initialize(const Joint& pan, const Joint& tilt) {
    pan_ = pan;
    tilt_ = tilt;
    is_updated_ = true;
  }

  void SetJointAngles(double pan_angle, double tilt_angle) {
    pan_.SetJointAngle(pan_angle);
    tilt_.SetJointAngle(tilt_angle);
    is_updated_ = true;
  }

  bool operator==(const Neck& neck) const {
    return (std::fabs(pan_.joint_angle_ - neck.pan_.joint_angle_) < 5e-3 &&
            std::fabs(tilt_.joint_angle_ - neck.tilt_.joint_angle_) < 5e-3);
  }

  bool operator!=(const Neck& neck) const { return !(*this == neck); }

  bool is_updated_{true};
  Joint pan_;
  Joint tilt_;
};

#endif  // TOPOQUAD_MASTER_SRC_NECK_NODE_HPP_
