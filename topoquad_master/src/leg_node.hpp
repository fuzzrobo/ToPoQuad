#ifndef TOPOQUAD_MASTER_SRC_LEG_NODE_HPP_
#define TOPOQUAD_MASTER_SRC_LEG_NODE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "common.hpp"

using namespace geometry_msgs::msg;
using std::acos;
using std::array;
using std::atan2;
using std::cos;
using std::floor;
using std::hypot;
using std::sin;
using std::sqrt;
using std::vector;

struct LinkLengths {
  double hip_yaw;
  double hip_pitch;
  double knee_pitch;
};

struct MountConfig {
  array<double, 2> position_xy;
  double yaw;
  int sign;
};

struct JointConfig {
  uint8_t id;
  double gear_ratio;
  double torque_ratio;
  double default_torque;
};

struct LegProfile {
  MountConfig mount;
  JointConfig hip_yaw;
  JointConfig hip_pitch;
  JointConfig knee_pitch;
  array<double, 3> initial_pose;
};

struct RobotProfile {
  LinkLengths link_lengths;
  array<LegProfile, 4> legs;
};

inline double TwoLinkIkTheta1(double x, double y, double l1, double l2) {
  return atan2(y, x) + acos(((x * x + y * y) + l1 * l1 - l2 * l2) / (2 * l1 * sqrt(x * x + y * y)));
}

inline double TwoLinkIkTheta2(double x, double y, double l1, double l2) {
  return -acos(((x * x + y * y) - l1 * l1 - l2 * l2) / (2 * l1 * l2));
}

inline double NormalizeAngle(double theta) {
  return theta - (2.0 * pi) * floor((theta + pi) / (2.0 * pi));
}

inline vector<double> LegInverseKinematics(const Point& target_point, const Pose2D& fixed_pose, int sign, const LinkLengths& L) {
  const double dx = target_point.x - fixed_pose.x;
  const double dy = target_point.y - fixed_pose.y;
  const double x  = hypot(dx, dy) - L.hip_yaw;

  vector<double> angles(3);
  angles[0] =  sign * NormalizeAngle(atan2(dy, dx) - fixed_pose.theta);
  angles[1] = -TwoLinkIkTheta1(x, target_point.z, L.hip_pitch, L.knee_pitch);
  angles[2] = -TwoLinkIkTheta2(x, target_point.z, L.hip_pitch, L.knee_pitch);
  return angles;
}

inline Point LegForwardKinematics(const vector<double>& angles, const Pose2D& fixed_pose, int sign, const LinkLengths& L) {
  Point point;
  const double l = L.hip_pitch * cos(angles[1]) + L.knee_pitch * cos(angles[1] + angles[2]);
  point.x =  fixed_pose.x + (L.hip_yaw + l) * cos(fixed_pose.theta + sign * angles[0]);
  point.y =  fixed_pose.y + (L.hip_yaw + l) * sin(fixed_pose.theta + sign * angles[0]);
  point.z = -(L.hip_pitch * sin(angles[1]) + L.knee_pitch * sin(angles[1] + angles[2]));
  return point;
}

inline vector<double> LegInverseStatics(const vector<double>& angles, const Pose2D& fixed_pose, int sign, const Vector3& force_world, const LinkLengths& L) {
  if (angles.size() != 3) return {0.0, 0.0, 0.0};

  const double t1 = angles[0], t2 = angles[1], t3 = angles[2];
  const double phi  = fixed_pose.theta + sign * t1;
  const double l   = L.hip_pitch * cos(t2) + L.knee_pitch * cos(t2 + t3);
  const double dl2 = -L.hip_pitch * sin(t2) - L.knee_pitch * sin(t2 + t3);
  const double dl3 = -L.knee_pitch * sin(t2 + t3);

  const double j1x = -sign * (L.hip_yaw + l) * sin(phi);
  const double j1y =  sign * (L.hip_yaw + l) * cos(phi);
  const double j2x =  cos(phi) * dl2;
  const double j2y =  sin(phi) * dl2;
  const double j2z = -(L.hip_pitch * cos(t2) + L.knee_pitch * cos(t2 + t3));
  const double j3x =  cos(phi) * dl3;
  const double j3y =  sin(phi) * dl3;
  const double j3z = -L.knee_pitch * cos(t2 + t3);

  const double tau0 = j1x * force_world.x + j1y * force_world.y;
  const double tau1 = j2x * force_world.x + j2y * force_world.y + j2z * force_world.z;
  const double tau2 = j3x * force_world.x + j3y * force_world.y + j3z * force_world.z;
  return {tau0, tau1, tau2};
}

inline Vector3 LegForwardStatics(const vector<double>& angles, const Pose2D& fixed_pose, int sign, const vector<double>& tau, const LinkLengths& L) {
  if (angles.size() != 3 || tau.size() != 3) return Vector3();

  const double t1 = angles[0], t2 = angles[1], t3 = angles[2];
  const double phi  = fixed_pose.theta + sign * t1;
  const double l   = L.hip_pitch * cos(t2) + L.knee_pitch * cos(t2 + t3);
  const double dl2 = -L.hip_pitch * sin(t2) - L.knee_pitch * sin(t2 + t3);
  const double dl3 = -L.knee_pitch * sin(t2 + t3);

  Eigen::Matrix3d jacobian_transpose;
  jacobian_transpose << -sign * (L.hip_yaw + l) * sin(phi), sign * (L.hip_yaw + l) * cos(phi), 0.0,
                         cos(phi) * dl2, sin(phi) * dl2, -(L.hip_pitch * cos(t2) + L.knee_pitch * cos(t2 + t3)),
                         cos(phi) * dl3, sin(phi) * dl3, -L.knee_pitch * cos(t2 + t3);

  const Eigen::Vector3d tau_vector(tau[0], tau[1], tau[2]);
  const Eigen::Vector3d force_vector = jacobian_transpose.colPivHouseholderQr().solve(tau_vector);
  return Vector3().set__x(force_vector(0)).set__y(force_vector(1)).set__z(force_vector(2));
}

class Leg {
 public:
  Leg() : hip_yaw_(1), hip_pitch_(2), knee_pitch_(3) {
    fixed_pose_.x     = 0.0;
    fixed_pose_.y     = 0.0;
    fixed_pose_.theta = 0.0;
  }

  void initialize(const LegProfile& profile) {
    fixed_pose_.x     = profile.mount.position_xy[0];
    fixed_pose_.y     = profile.mount.position_xy[1];
    fixed_pose_.theta = profile.mount.yaw;
    auto p_hy=profile.hip_yaw   ; 
    auto p_hp=profile.hip_pitch ; 
    auto p_kp=profile.knee_pitch; 
    hip_yaw_    = Joint(p_hy.id, p_hy.gear_ratio, profile.initial_pose[0], p_hy.torque_ratio, p_hy.default_torque);
    hip_pitch_  = Joint(p_hp.id, p_hp.gear_ratio, profile.initial_pose[1], p_hp.torque_ratio, p_hp.default_torque);
    knee_pitch_ = Joint(p_kp.id, p_kp.gear_ratio, profile.initial_pose[2], p_kp.torque_ratio, p_kp.default_torque);

    is_updated_   = true;
    updated_time_ = 0.0;
  }

  void SetJointAngles(const vector<double>& angles) {
    if (angles.size() != 3) return;
    hip_yaw_.SetJointAngle(angles[0]);
    hip_pitch_.SetJointAngle(angles[1]);
    knee_pitch_.SetJointAngle(angles[2]);
    is_updated_ = true;
  }

  void SetJointTorques(const vector<double>& torques) {
    if (torques.size() != 3) return;
    hip_yaw_.SetJointTorque(torques[0]);
    hip_pitch_.SetJointTorque(torques[1]);
    knee_pitch_.SetJointTorque(torques[2]);
    is_updated_ = true;
  }

  std::vector<double> GetJointAngles() const { return {hip_yaw_.joint_angle_, hip_pitch_.joint_angle_, knee_pitch_.joint_angle_};}

  std::vector<double> GetJointTorques() const { return {hip_yaw_.joint_torque_, hip_pitch_.joint_torque_, knee_pitch_.joint_torque_};}

  bool is_updated_{true};
  double updated_time_{0.0};
  Joint hip_yaw_;
  Joint hip_pitch_;
  Joint knee_pitch_;
  Pose2D fixed_pose_;
};

inline RobotProfile MakeDefaultRobotProfile() {
  constexpr double base_radius       = 0.063;
  constexpr double hip_yaw_length    = 0.0445;
  constexpr double hip_pitch_length  = 0.0445;
  constexpr double knee_pitch_length = 0.0715;
  constexpr double torque_ratio      = 0.92 / 800.0;
  constexpr double default_torque    = 0.6;
  constexpr double zero              = 0.0;
  constexpr double home_pitch        = pi / 4.0;

  return RobotProfile{
      LinkLengths{hip_yaw_length, hip_pitch_length, knee_pitch_length},
      {{
          {MountConfig{{base_radius * std::cos(1.0 * pi / 4.0),
                        base_radius * std::sin(1.0 * pi / 4.0)}, 1.0 * pi / 4.0, +1},
           JointConfig{14, -1.0, torque_ratio, default_torque},
           JointConfig{13, +1.0, torque_ratio, default_torque},
           JointConfig{12, +1.0, torque_ratio, default_torque},
           {zero, home_pitch, home_pitch}},
          {MountConfig{{base_radius * std::cos(3.0 * pi / 4.0),
                        base_radius * std::sin(3.0 * pi / 4.0)}, 3.0 * pi / 4.0, -1},
           JointConfig{24, +1.0, torque_ratio, default_torque},
           JointConfig{23, +1.0, torque_ratio, default_torque},
           JointConfig{22, +1.0, torque_ratio, default_torque},
           {zero, home_pitch, home_pitch}},
          {MountConfig{{base_radius * std::cos(7.0 * pi / 4.0),
                        base_radius * std::sin(7.0 * pi / 4.0)}, 7.0 * pi / 4.0, +1},
           JointConfig{4, -1.0, torque_ratio, default_torque},
           JointConfig{3, +1.0, torque_ratio, default_torque},
           JointConfig{2, +1.0, torque_ratio, default_torque},
           {zero, home_pitch, home_pitch}},
          {MountConfig{{base_radius * std::cos(5.0 * pi / 4.0),
                        base_radius * std::sin(5.0 * pi / 4.0)}, 5.0 * pi / 4.0, -1},
           JointConfig{34, +1.0, torque_ratio, default_torque},
           JointConfig{33, +1.0, torque_ratio, default_torque},
           JointConfig{32, +1.0, torque_ratio, default_torque},
           {zero, home_pitch, home_pitch}},
      }}};
}

#endif  // TOPOQUAD_MASTER_SRC_LEG_NODE_HPP_
