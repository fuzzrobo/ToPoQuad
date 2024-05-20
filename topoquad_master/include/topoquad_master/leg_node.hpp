#ifndef TOPOQUAD_LEG_H_
#define TOPOQUAD_LEG_H_

#include <topoquad_master/common.hpp>

#include "dynamixel_handler/msg/dynamixel_command_x_control_current_position.hpp"
#include "dynamixel_handler/msg/dynamixel_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoquad_msgs/msg/quad_robot_cmd_leg_angle.hpp"
#include "topoquad_msgs/msg/quad_robot_cmd_leg_point.hpp"
#include "topoquad_msgs/msg/quad_robot_state_leg.hpp"

#define ANGLE_FR (M_PI_4 + M_PI_2 * 0)
#define ANGLE_FL (M_PI_4 + M_PI_2 * 1)
#define ANGLE_BL (M_PI_4 + M_PI_2 * 2)
#define ANGLE_BR (M_PI_4 + M_PI_2 * 3)

#define LENGTH_BASE 0.052
#define LENGTH_HIP_YAW 0.0445
#define LENGTH_HIP_PITCH 0.0445
#define LENGTH_KNEE_PITCH 0.0715

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose2D;
using std::isnan;
using std::ref;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;

// https://qiita.com/Ninagawa123/items/4ae058d819de1d5b698a
inline double two_link_ik_t1(const double x, const double y, const double l1, const double l2) {
    return atan2(y, x) + acos(((x * x + y * y) + l1 * l1 - l2 * l2) / (2 * l1 * sqrt(x * x + y * y)));
}

inline double two_link_ik_t2(const double x, const double y, const double l1, const double l2) {
    return -acos(((x * x + y * y) - l1 * l1 - l2 * l2) / (2 * l1 * l2));
}

inline double normalizeAngle(const double theta) {
    return theta - (2 * M_PI) * floor((theta + M_PI) / (2 * M_PI));
}

vector<double> leg_ik(const Point& tp, const Pose2D& fp, const int& sign) {
    double dx = tp.x - fp.x;
    double dy = tp.y - fp.y;
    double x = hypot(dx, dy) - LENGTH_HIP_YAW;
    vector<double> angles(3);
    angles[0] = sign * normalizeAngle(atan2(dy, dx) - fp.theta);
    angles[1] = -two_link_ik_t1(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    angles[2] = -two_link_ik_t2(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    return angles;
}

Point leg_k(const vector<double>& angles, const Pose2D& fp, const int& sign) {
    Point k;
    double l = LENGTH_HIP_PITCH * cos(angles[1]) + LENGTH_KNEE_PITCH * cos(angles[1] + angles[2]);
    k.x = fp.x + (LENGTH_HIP_YAW + l) * cos(fp.theta + sign * angles[0]);
    k.y = fp.y + (LENGTH_HIP_YAW + l) * sin(fp.theta + sign * angles[0]);
    k.z = LENGTH_HIP_PITCH * sin(angles[1]) + LENGTH_KNEE_PITCH * sin(angles[1] + angles[2]);
    return k;
}

class Leg {
   public:
    Leg() : Leg(0, 0, 0) {}
    Leg(double x, double y, double theta) : is_updated_(true),
                                            hip_yaw_(Joint(1)),
                                            hip_pitch_(Joint(2)),
                                            knee_pitch_(Joint(3)) {
        fixed_pose_.x = x;
        fixed_pose_.y = y;
        fixed_pose_.theta = theta;
    }

    void initialize(const Joint& hip_yaw, const Joint& hip_pitch, const Joint& knee_pitch) {
        hip_yaw_ = hip_yaw;
        hip_pitch_ = hip_pitch;
        knee_pitch_ = knee_pitch;
        is_updated_ = true;
    }
    void SetJointAngles(const vector<double>& angles) {
        if (angles.size() != 3) {
            std::cout << "The size of angles must be 3" << std::endl;
            return;
        }
        hip_yaw_.SetJointAngle(angles[0]);
        hip_pitch_.SetJointAngle(angles[1]);
        knee_pitch_.SetJointAngle(angles[2]);
        is_updated_ = true;
    }

    void SetJointTorques(const vector<double>& torques) {
        if (torques.size() != 3) {
            std::cout << "The size of torques must be 3" << std::endl;
            return;
        }
        hip_yaw_.SetJointAngle(torques[0]);
        hip_pitch_.SetJointAngle(torques[1]);
        knee_pitch_.SetJointAngle(torques[2]);
        is_updated_ = true;
    }

    vector<double> GetJointAngles() {
        vector<double> angles(3);
        angles[0] = hip_yaw_.joint_angle_;
        angles[1] = hip_pitch_.joint_angle_;
        angles[2] = knee_pitch_.joint_angle_;
        return angles;
    }

    vector<double> GetJointTorques() {
        vector<double> torques(3);
        torques[0] = hip_yaw_.joint_torque_;
        torques[1] = hip_pitch_.joint_torque_;
        torques[2] = knee_pitch_.joint_torque_;
        return torques;
    }

    bool operator==(const Leg& leg) const {
        return (fabs(hip_yaw_.joint_angle_ - leg.hip_yaw_.joint_angle_) < 2e-2 && fabs(hip_pitch_.joint_angle_ - leg.hip_pitch_.joint_angle_) < 2e-2 && fabs(knee_pitch_.joint_angle_ - leg.knee_pitch_.joint_angle_) < 2e-2);
    }
    bool operator!=(const Leg& leg) const {
        return !(*this == leg);
    }

    bool is_updated_;            // 関節角が更新されたかどうか
    double updated_time_;  // 関節角が更新された時間
    Joint hip_yaw_;
    Joint hip_pitch_;
    Joint knee_pitch_;
    Pose2D fixed_pose_;
};

#endif /* TOPOQUAD_LEG_H_ */