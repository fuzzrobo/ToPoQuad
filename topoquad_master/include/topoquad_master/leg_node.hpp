#ifndef TOPOQUAD_LEG_H_
#define TOPOQUAD_LEG_H_

#include <topoquad_master/common.hpp>

#include <eigen3/Eigen/Dense>
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoquad_msgs/msg/quad_robot_leg.hpp"

#define ANGLE_FR (M_PI_4 + M_PI_2 * 0)
#define ANGLE_FL (M_PI_4 + M_PI_2 * 1)
#define ANGLE_BL (M_PI_4 + M_PI_2 * 2)
#define ANGLE_BR (M_PI_4 + M_PI_2 * 3)

#define LENGTH_BASE 0.063
#define LENGTH_HIP_YAW 0.0445
#define LENGTH_HIP_PITCH 0.0445
#define LENGTH_KNEE_PITCH 0.0715

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Vector3;
using std::isnan;
using std::ref;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;

// Small helpers
inline bool is_zero(const Point& p) {
    return p.x == 0.0 && p.y == 0.0 && p.z == 0.0;
}
inline bool is_zero(const Vector3& v) {
    return v.x == 0.0 && v.y == 0.0 && v.z == 0.0;
}

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

vector<double> leg_inverse_kinematics(const Point& tp, const Pose2D& fp, const int& sign) {
    double dx = tp.x - fp.x;
    double dy = tp.y - fp.y;
    double x = hypot(dx, dy) - LENGTH_HIP_YAW;
    vector<double> angles(3);
    angles[0] = sign * normalizeAngle(atan2(dy, dx) - fp.theta);
    angles[1] = -two_link_ik_t1(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    angles[2] = -two_link_ik_t2(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    return angles;
}

Point leg_forward_kinematics(const vector<double>& angles, const Pose2D& fp, const int& sign) {
    Point k;
    double l = LENGTH_HIP_PITCH * cos(angles[1]) + LENGTH_KNEE_PITCH * cos(angles[1] + angles[2]);
    k.x = fp.x + (LENGTH_HIP_YAW + l) * cos(fp.theta + sign * angles[0]);
    k.y = fp.y + (LENGTH_HIP_YAW + l) * sin(fp.theta + sign * angles[0]);
    k.z = -(LENGTH_HIP_PITCH * sin(angles[1]) + LENGTH_KNEE_PITCH * sin(angles[1] + angles[2]));
    return k;
}

// Map end-effector force (in world frame) to joint torques via J^T.
// angles: [hip_yaw, hip_pitch, knee_pitch]
// fp: fixed base pose of the leg in world frame
// sign: +1 for right legs (FR, BR), -1 for left legs (FL, BL) — same as leg_forward_kinematics/leg_inverse_kinematics
inline vector<double> leg_inverse_statics(const vector<double>& angles,
                                          const Pose2D& fp,
                                          const int& sign,
                                          const Vector3& Fw) {
    vector<double> tau(3, 0.0);
    if (angles.size() != 3) return tau;

    const double t1 = angles[0];
    const double t2 = angles[1];
    const double t3 = angles[2];

    const double phi  = fp.theta + sign * t1;
    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double c2   = std::cos(t2);
    const double s2   = std::sin(t2);
    const double c23  = std::cos(t2 + t3);
    const double s23  = std::sin(t2 + t3);

    const double l    = LENGTH_HIP_PITCH * c2 + LENGTH_KNEE_PITCH * c23;
    const double dl2  = -LENGTH_HIP_PITCH * s2 - LENGTH_KNEE_PITCH * s23;  // dl/dt2
    const double dl3  = -LENGTH_KNEE_PITCH * s23;                          // dl/dt3

    // Jacobian columns (x,y,z) for t1,t2,t3
    const double j1x = -sign * (LENGTH_HIP_YAW + l) * sphi;
    const double j1y =  sign * (LENGTH_HIP_YAW + l) * cphi;
    const double j1z =  0.0;

    const double j2x = cphi * dl2;
    const double j2y = sphi * dl2;
    const double j2z = -(LENGTH_HIP_PITCH * c2 + LENGTH_KNEE_PITCH * c23);

    const double j3x = cphi * dl3;
    const double j3y = sphi * dl3;
    const double j3z = -LENGTH_KNEE_PITCH * c23;

    const double Fx = Fw.x;
    const double Fy = Fw.y;
    const double Fz = Fw.z;

    tau[0] = j1x * Fx + j1y * Fy + j1z * Fz;
    tau[1] = j2x * Fx + j2y * Fy + j2z * Fz;
    tau[2] = j3x * Fx + j3y * Fy + j3z * Fz;

    return tau;
}

// Map joint torques to end-effector force via (J^T)^{-1}
inline Vector3 leg_forward_statics(const vector<double>& angles,
                                   const Pose2D& fp,
                                   const int& sign,
                                   const vector<double>& tau) {
    Vector3 F;
    F.x = F.y = F.z = 0.0;
    if (angles.size() != 3 || tau.size() != 3) return F;

    const double t1 = angles[0];
    const double t2 = angles[1];
    const double t3 = angles[2];

    const double phi  = fp.theta + sign * t1;
    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double c2   = std::cos(t2);
    const double s2   = std::sin(t2);
    const double c23  = std::cos(t2 + t3);
    const double s23  = std::sin(t2 + t3);

    const double l    = LENGTH_HIP_PITCH * c2 + LENGTH_KNEE_PITCH * c23;
    const double dl2  = -LENGTH_HIP_PITCH * s2 - LENGTH_KNEE_PITCH * s23;  // dl/dt2
    const double dl3  = -LENGTH_KNEE_PITCH * s23;                          // dl/dt3

    // Jacobian columns (x,y,z) for t1,t2,t3
    const double j1x = -sign * (LENGTH_HIP_YAW + l) * sphi;
    const double j1y =  sign * (LENGTH_HIP_YAW + l) * cphi;
    const double j1z =  0.0;

    const double j2x = cphi * dl2;
    const double j2y = sphi * dl2;
    const double j2z = -(LENGTH_HIP_PITCH * c2 + LENGTH_KNEE_PITCH * c23);

    const double j3x = cphi * dl3;
    const double j3y = sphi * dl3;
    const double j3z = -LENGTH_KNEE_PITCH * c23;

    Eigen::Matrix3d JT;
    JT << j1x, j1y, j1z,
          j2x, j2y, j2z,
          j3x, j3y, j3z;

    Eigen::Vector3d tau_v(tau[0], tau[1], tau[2]);
    Eigen::Vector3d F_v = JT.colPivHouseholderQr().solve(tau_v);

    F.x = F_v(0);
    F.y = F_v(1);
    F.z = F_v(2);
    return F;
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
        hip_yaw_.SetJointTorque(torques[0]);
        hip_pitch_.SetJointTorque(torques[1]);
        knee_pitch_.SetJointTorque(torques[2]);
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
