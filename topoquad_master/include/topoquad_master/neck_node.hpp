#ifndef TOPOQUAD_NECK_H_
#define TOPOQUAD_NECK_H_

#include <topoquad_master/common.hpp>

#include "dynamixel_handler/msg/dxl_commands_x.hpp"
#include "dynamixel_handler/msg/dxl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoquad_msgs/msg/quad_robot_cmd_neck_angle.hpp"
#include "topoquad_msgs/msg/quad_robot_state_neck.hpp"

using std::isnan;
using std::ref;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;

class Neck {
   public:
    Neck() : is_updated_(true), pan_(Joint(1)), tilt_(Joint(2)) {}
    void initialize(const Joint& pan, const Joint& tilt) {
        pan_ = pan;
        tilt_ = tilt;
        is_updated_ = true;
    }

    void SetJointAngles(const double& pan_angle, const double& tilt_angle) {
        pan_.SetJointAngle(pan_angle);
        tilt_.SetJointAngle(tilt_angle);
        is_updated_ = true;
    }

    bool operator==(const Neck& neck) const {
        return (fabs(pan_.joint_angle_ - neck.pan_.joint_angle_) < 5e-3 && fabs(tilt_.joint_angle_ - neck.tilt_.joint_angle_) < 5e-3);
    }
    bool operator!=(const Neck& neck) const {
        return !(*this == neck);
    }

    bool is_updated_;  // 関節角が更新されたかどうか
    Joint pan_;
    Joint tilt_;
};

#endif /* TOPOQUAD_NECK_H_ */