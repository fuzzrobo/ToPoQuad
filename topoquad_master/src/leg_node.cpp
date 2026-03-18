#include "leg_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoquad_msgs/msg/quad_robot_leg.hpp"

using namespace dynamixel_handler_msgs::msg;
using namespace topoquad_msgs::msg;
using std::isnan;
using std::string;
using std::vector;

constexpr std::array<const char*, 4> leg_suffixes = {"fr", "fl", "br", "bl"};
constexpr std::chrono::milliseconds timer_period(50);
constexpr std::chrono::milliseconds background_period(200);
constexpr double initial_pose_velocity_deg_s = 50.0;

enum LegIndex : std::size_t {
  front_right = 0,
  front_left = 1,
  back_right = 2,
  back_left = 3,
};

class LegNode : public rclcpp::Node {
 public:
  LegNode() : Node("leg_node"), robot_profile_(MakeDefaultRobotProfile()) {
    prev_cmd_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    LoadRobotProfile();
    DeclareControlParameters();
    for (std::size_t i = 0; i < target_legs_.size(); ++i) target_legs_[i].initialize(robot_profile_.legs[i]);
    present_legs_ = target_legs_;
    goal_legs_ = target_legs_;

    dyn_cmd_pub_     = this->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
    leg_state_p_pub_ = this->create_publisher<QuadRobotLeg>("legs/state/present", 10);
    leg_state_g_pub_ = this->create_publisher<QuadRobotLeg>("legs/state/goal", 10);
    leg_state_t_pub_ = this->create_publisher<QuadRobotLeg>("legs/state/target", 10);

    using std::placeholders::_1;
    leg_command_sub_ = this->create_subscription<QuadRobotLeg>("legs/command", 10, std::bind(&LegNode::leg_command_cb, this, _1));
    dyn_state_sub_   = this->create_subscription<DxlStates>("dynamixel/states", 10, std::bind(&LegNode::dyn_state_cb, this, _1));
    timer_ = this->create_wall_timer(timer_period, std::bind(&LegNode::main_loop, this));
  }

 private:
  enum ControlMode {
    MODE_VELOCITY,
    MODE_VELOCITY_SAFE,
    MODE_POSITION
  };

  rclcpp::Publisher<DxlCommandsX>::SharedPtr dyn_cmd_pub_;
  rclcpp::Publisher<QuadRobotLeg>::SharedPtr leg_state_p_pub_, leg_state_g_pub_, leg_state_t_pub_;
  rclcpp::Subscription<QuadRobotLeg>::SharedPtr leg_command_sub_;
  rclcpp::Subscription<DxlStates>::SharedPtr dyn_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time prev_cmd_time_;

  RobotProfile robot_profile_;
  std::array<Leg, 4> target_legs_;
  std::array<Leg, 4> goal_legs_;
  std::array<Leg, 4> present_legs_;

  double outer_gain_p_{5.0};
  double outer_gain_d_{0.0};
  double dxl_vel_gain_{50.0};
  double dxl_pos_gain_{800.0};
  double overshoot_{50.0 * deg_to_rad};
  double deadband_{1.0 * deg_to_rad};
  ControlMode control_mode_{MODE_POSITION};

  template <std::size_t N>
  std::array<double, N> DeclareDoubleArray(const string& name, const std::array<double, N>& default_values) {
    const vector<double> default_vector(default_values.begin(), default_values.end());
    const auto values = this->declare_parameter<vector<double>>(name, default_vector);
    if (values.size() != N)
      throw std::runtime_error(name + " must have " + std::to_string(N) + " elements");

    std::array<double, N> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void LoadRobotProfile() {
    robot_profile_.link_lengths.hip_yaw    = this->declare_parameter<double>("legs.link_lengths.hip_yaw", robot_profile_.link_lengths.hip_yaw);
    robot_profile_.link_lengths.hip_pitch  = this->declare_parameter<double>("legs.link_lengths.hip_pitch", robot_profile_.link_lengths.hip_pitch);
    robot_profile_.link_lengths.knee_pitch = this->declare_parameter<double>("legs.link_lengths.knee_pitch", robot_profile_.link_lengths.knee_pitch);

    for (std::size_t i = 0; i < leg_suffixes.size(); ++i) {
      auto& leg = robot_profile_.legs[i];

      const string mnt = string("legs.mounts.") + leg_suffixes[i];
      leg.mount.position_xy = DeclareDoubleArray<2>(mnt + ".position_xy", leg.mount.position_xy);
      leg.mount.yaw         = this->declare_parameter<double>(mnt + ".yaw", leg.mount.yaw);

      const string ini = string("legs.initial_pose.") + leg_suffixes[i];
      leg.initial_pose = DeclareDoubleArray<3>(ini, leg.initial_pose);

      const string jnt = string("legs.joints.") + leg_suffixes[i];
      auto& hy = leg.hip_yaw;
      hy.id             = (uint8_t)this->declare_parameter<int64_t>(jnt + ".hip_yaw.id", hy.id);
      hy.gear_ratio     = this->declare_parameter<double>(jnt + ".hip_yaw.gear_ratio", hy.gear_ratio);
      hy.torque_ratio   = this->declare_parameter<double>(jnt + ".hip_yaw.torque_ratio", hy.torque_ratio);
      hy.default_torque = this->declare_parameter<double>(jnt + ".hip_yaw.default_torque", hy.default_torque);

      auto& hp = leg.hip_pitch;
      hp.id             = (uint8_t)this->declare_parameter<int64_t>(jnt + ".hip_pitch.id", hp.id);
      hp.gear_ratio     = this->declare_parameter<double>(jnt + ".hip_pitch.gear_ratio", hp.gear_ratio);
      hp.torque_ratio   = this->declare_parameter<double>(jnt + ".hip_pitch.torque_ratio", hp.torque_ratio);
      hp.default_torque = this->declare_parameter<double>(jnt + ".hip_pitch.default_torque", hp.default_torque);

      auto& kp = leg.knee_pitch;
      kp.id             = (uint8_t)this->declare_parameter<int64_t>(jnt + ".knee_pitch.id", kp.id);
      kp.gear_ratio     = this->declare_parameter<double>(jnt + ".knee_pitch.gear_ratio", kp.gear_ratio);
      kp.torque_ratio   = this->declare_parameter<double>(jnt + ".knee_pitch.torque_ratio", kp.torque_ratio);
      kp.default_torque = this->declare_parameter<double>(jnt + ".knee_pitch.default_torque", kp.default_torque);
    }
  }

  void DeclareControlParameters() {
    const auto mode_str = this->declare_parameter<string>("control_mode", "position");
    if      (mode_str == "velocity"         || mode_str == "vel")    control_mode_ = MODE_VELOCITY;
    else if (mode_str == "position"         || mode_str == "pos")    control_mode_ = MODE_POSITION;
    else if (mode_str == "velocity_current" || mode_str == "velcur") control_mode_ = MODE_VELOCITY_SAFE;
    else RCLCPP_ERROR(get_logger(), "Invalid control mode [velocity, position, velocity_current]");

    outer_gain_p_ = this->declare_parameter<double>("outer_gain.p", outer_gain_p_);
    this->declare_parameter<double>("outer_gain.i", 0.0);
    outer_gain_d_ = this->declare_parameter<double>("outer_gain.d", outer_gain_d_);
    dxl_vel_gain_ = this->declare_parameter<double>("dynamixel_gain.vel_p", dxl_vel_gain_);
    dxl_pos_gain_ = this->declare_parameter<double>("dynamixel_gain.pos_p", dxl_pos_gain_);
    overshoot_    = this->declare_parameter<double>("overshoot", overshoot_ * rad_to_deg) * deg_to_rad;
    deadband_     = this->declare_parameter<double>("deadband", deadband_ * rad_to_deg) * deg_to_rad;
  }

  void main_loop() {
    const auto now = this->get_clock()->now();
    if ((now - prev_cmd_time_) < rclcpp::Duration(background_period)) return;

    DxlCommandsX dyn_msg;
    AppendStatusRequest(dyn_msg);
    AppendHomePoseCommand(dyn_msg);

    dyn_cmd_pub_->publish(dyn_msg);
  }

  void leg_command_cb(const QuadRobotLeg::SharedPtr msg) {
    ApplyKinematics(front_right, msg->angles_fr, msg->point_fr);
    ApplyKinematics(front_left, msg->angles_fl, msg->point_fl);
    ApplyKinematics(back_right, msg->angles_br, msg->point_br);
    ApplyKinematics(back_left, msg->angles_bl, msg->point_bl);

    ApplyEffort(front_right, msg->torques_fr, msg->force_fr);
    ApplyEffort(front_left, msg->torques_fl, msg->force_fl);
    ApplyEffort(back_right, msg->torques_br, msg->force_br);
    ApplyEffort(back_left, msg->torques_bl, msg->force_bl);

    DxlCommandsX dyn_msg;
    switch (control_mode_) {
      case MODE_VELOCITY:      return AppendVelocityCommand(dyn_msg);
      case MODE_POSITION:      return AppendPositionCommand(dyn_msg);
      case MODE_VELOCITY_SAFE: return AppendVelocityPositionCommand(dyn_msg);
    }
    AppendStatusRequest(dyn_msg);
    dyn_cmd_pub_->publish(dyn_msg);
    prev_cmd_time_ = this->get_clock()->now();

    PublishLegState(target_legs_, leg_state_t_pub_);
  }

  void dyn_state_cb(const DxlStates::SharedPtr msg) {
    const auto state_time = rclcpp::Time(msg->stamp).seconds();
    UpdateLegsFromPresentState(msg, state_time);
    PublishLegState(present_legs_, leg_state_p_pub_);

    UpdateLegsFromGoalState(msg);
    PublishLegState(goal_legs_, leg_state_g_pub_);

    if (!msg->gain.id_list.empty() &&
        (!msg->gain.velocity_p_gain_pulse.empty() &&
         !msg->gain.position_p_gain_pulse.empty()) &&
        (msg->gain.velocity_p_gain_pulse[0] != dxl_vel_gain_ ||
         msg->gain.position_p_gain_pulse[0] != dxl_pos_gain_)) {
      DxlCommandsX dyn_msg;
      for (const auto& leg : target_legs_) {
        dyn_msg.gain.id_list.push_back(leg.hip_yaw_.id_);
        dyn_msg.gain.id_list.push_back(leg.hip_pitch_.id_);
        dyn_msg.gain.id_list.push_back(leg.knee_pitch_.id_);
        dyn_msg.gain.velocity_p_gain_pulse.push_back(dxl_vel_gain_);
        dyn_msg.gain.velocity_p_gain_pulse.push_back(dxl_vel_gain_);
        dyn_msg.gain.velocity_p_gain_pulse.push_back(dxl_vel_gain_);
        dyn_msg.gain.position_p_gain_pulse.push_back(dxl_pos_gain_);
        dyn_msg.gain.position_p_gain_pulse.push_back(dxl_pos_gain_);
        dyn_msg.gain.position_p_gain_pulse.push_back(dxl_pos_gain_);
      }
      dyn_cmd_pub_->publish(dyn_msg);
    }
  }

  void ApplyKinematics(std::size_t leg_index, const vector<double>& angles, const Point& point) {
    auto& leg = target_legs_[leg_index];
    if (!angles.empty()) {
      leg.SetJointAngles(angles);
      return;
    }
    if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0) return;

    const auto ik_angles = LegInverseKinematics(point, leg.fixed_pose_, robot_profile_.legs[leg_index].mount.sign, robot_profile_.link_lengths);
    if (!isnan(ik_angles[0]) && !isnan(ik_angles[1]) && !isnan(ik_angles[2]))
      leg.SetJointAngles(ik_angles);
  }

  void ApplyEffort(std::size_t leg_index, const vector<double>& torques, const Vector3& force) {
    if (!torques.empty()) {
      target_legs_[leg_index].SetJointTorques(torques);
      return;
    }
    if (force.x == 0.0 && force.y == 0.0 && force.z == 0.0) return;

    const auto current_angles = present_legs_[leg_index].GetJointAngles();
    const auto joint_torques = LegInverseStatics(current_angles, target_legs_[leg_index].fixed_pose_, robot_profile_.legs[leg_index].mount.sign, force, robot_profile_.link_lengths);
    target_legs_[leg_index].SetJointTorques(joint_torques);
  }

  void UpdateLegsFromPresentState(const DxlStates::SharedPtr& msg, double state_time) {
    const auto& present = msg->present;
    if (present.id_list.empty()) return;

    for (auto& leg : present_legs_) {
      bool matched = false;
      for (std::size_t i = 0; i < present.id_list.size(); ++i) {
        if (present.id_list[i] == leg.hip_yaw_.id_) {
          leg.hip_yaw_.servo_velocity_ = present.velocity_deg_s[i] * deg_to_rad;
          leg.hip_yaw_.SetServoAngle(present.position_deg[i] * deg_to_rad);
          leg.hip_yaw_.SetServoCurrent(present.current_ma[i]);
          matched = true;
        }
        if (present.id_list[i] == leg.hip_pitch_.id_) {
          leg.hip_pitch_.servo_velocity_ = present.velocity_deg_s[i] * deg_to_rad;
          leg.hip_pitch_.SetServoAngle(present.position_deg[i] * deg_to_rad);
          leg.hip_pitch_.SetServoCurrent(present.current_ma[i]);
          matched = true;
        }
        if (present.id_list[i] == leg.knee_pitch_.id_) {
          leg.knee_pitch_.servo_velocity_ = present.velocity_deg_s[i] * deg_to_rad;
          leg.knee_pitch_.SetServoAngle(present.position_deg[i] * deg_to_rad);
          leg.knee_pitch_.SetServoCurrent(present.current_ma[i]);
          matched = true;
        }
      }
      if (!matched) continue;
      leg.is_updated_ = true;
      leg.updated_time_ = state_time;
    }
  }

  void UpdateLegsFromGoalState(const DxlStates::SharedPtr& msg) {
    const auto& goal = msg->goal;
    if (goal.id_list.empty()) return;

    for (auto& leg : goal_legs_) {
      for (std::size_t i = 0; i < goal.id_list.size(); ++i) {
        if (goal.id_list[i] == leg.hip_yaw_.id_) {
          leg.hip_yaw_.SetServoAngle(goal.position_deg[i] * deg_to_rad);
          leg.hip_yaw_.SetServoCurrent(goal.current_ma[i]);
        }
        if (goal.id_list[i] == leg.hip_pitch_.id_) {
          leg.hip_pitch_.SetServoAngle(goal.position_deg[i] * deg_to_rad);
          leg.hip_pitch_.SetServoCurrent(goal.current_ma[i]);
        }
        if (goal.id_list[i] == leg.knee_pitch_.id_) {
          leg.knee_pitch_.SetServoAngle(goal.position_deg[i] * deg_to_rad);
          leg.knee_pitch_.SetServoCurrent(goal.current_ma[i]);
        }
      }
      leg.is_updated_ = true;
    }
  }

  void AppendStatusRequest(DxlCommandsX& dyn_msg) const {
    for (const auto& leg : target_legs_) {
      dyn_msg.status.id_list.push_back(leg.hip_yaw_.id_);
      dyn_msg.status.id_list.push_back(leg.hip_pitch_.id_);
      dyn_msg.status.id_list.push_back(leg.knee_pitch_.id_);
      dyn_msg.status.error.push_back(false);
      dyn_msg.status.error.push_back(false);
      dyn_msg.status.error.push_back(false);
    }
  }

  void AppendHomePoseCommand(DxlCommandsX& dyn_msg) const {
    // TODO: For HEAD compatibility, revisit the home-pose packet shape later.
    // HEAD used hard-coded 0/45/45 deg and did not populate current_ma/profile_acc_deg_ss.
    auto& ctrl_msg = dyn_msg.current_base_position_control;
    for (const auto& leg : target_legs_) {
      ctrl_msg.id_list.push_back(leg.hip_yaw_.id_);
      ctrl_msg.id_list.push_back(leg.hip_pitch_.id_);
      ctrl_msg.id_list.push_back(leg.knee_pitch_.id_);
      ctrl_msg.position_deg.push_back(leg.hip_yaw_.servo_angle_ * rad_to_deg);
      ctrl_msg.position_deg.push_back(leg.hip_pitch_.servo_angle_ * rad_to_deg);
      ctrl_msg.position_deg.push_back(leg.knee_pitch_.servo_angle_ * rad_to_deg);
      ctrl_msg.profile_vel_deg_s.push_back(initial_pose_velocity_deg_s);
      ctrl_msg.profile_vel_deg_s.push_back(initial_pose_velocity_deg_s);
      ctrl_msg.profile_vel_deg_s.push_back(initial_pose_velocity_deg_s);
      ctrl_msg.current_ma.push_back(leg.hip_yaw_.servo_current_);
      ctrl_msg.current_ma.push_back(leg.hip_pitch_.servo_current_);
      ctrl_msg.current_ma.push_back(leg.knee_pitch_.servo_current_);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
    }
  }

  void AppendTargetCommand(DxlCommandsX& dyn_msg) {
    
  }

  void AppendVelocityPositionCommand(DxlCommandsX& dyn_msg) {
    auto& ctrl_msg = dyn_msg.current_base_position_control;
    static std::array<double, 4> diff_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> diff_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> diff_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now().seconds();
    for (std::size_t i = 0; i < target_legs_.size(); ++i) {
      auto& target = target_legs_[i];
      const auto& present = present_legs_[i];
      const double dt = now - present.updated_time_;

      const double target_hy = target.hip_yaw_.servo_angle_;
      const double target_hp = target.hip_pitch_.servo_angle_;
      const double target_kp = target.knee_pitch_.servo_angle_;
      const double present_hy = present.hip_yaw_.servo_angle_   + dt * present.hip_yaw_.servo_velocity_;
      const double present_hp = present.hip_pitch_.servo_angle_ + dt * present.hip_pitch_.servo_velocity_;
      const double present_kp = present.knee_pitch_.servo_angle_ + dt * present.knee_pitch_.servo_velocity_;

      const double diff_hy = target_hy - present_hy;
      const double diff_hp = target_hp - present_hp;
      const double diff_kp = target_kp - present_kp;

      const double vel_hy = std::fabs(diff_hy) < deadband_ ? 0.0001 : outer_gain_p_ * diff_hy + outer_gain_d_ * (diff_hy - diff_hy_pre[i]);
      const double vel_hp = std::fabs(diff_hp) < deadband_ ? 0.0001 : outer_gain_p_ * diff_hp + outer_gain_d_ * (diff_hp - diff_hp_pre[i]);
      const double vel_kp = std::fabs(diff_kp) < deadband_ ? 0.0001 : outer_gain_p_ * diff_kp + outer_gain_d_ * (diff_kp - diff_kp_pre[i]);

      const double pos_hy = diff_hy > 0.0 ? target_hy + overshoot_ : target_hy - overshoot_;
      const double pos_hp = diff_hp > 0.0 ? target_hp + overshoot_ : target_hp - overshoot_;
      const double pos_kp = diff_kp > 0.0 ? target_kp + overshoot_ : target_kp - overshoot_;

      ctrl_msg.id_list.push_back(target.hip_yaw_.id_);
      ctrl_msg.id_list.push_back(target.hip_pitch_.id_);
      ctrl_msg.id_list.push_back(target.knee_pitch_.id_);
      ctrl_msg.position_deg.push_back(pos_hy * rad_to_deg);
      ctrl_msg.position_deg.push_back(pos_hp * rad_to_deg);
      ctrl_msg.position_deg.push_back(pos_kp * rad_to_deg);
      ctrl_msg.profile_vel_deg_s.push_back(std::fabs(vel_hy * rad_to_deg));
      ctrl_msg.profile_vel_deg_s.push_back(std::fabs(vel_hp * rad_to_deg));
      ctrl_msg.profile_vel_deg_s.push_back(std::fabs(vel_kp * rad_to_deg));
      ctrl_msg.current_ma.push_back(target.hip_yaw_.servo_current_);
      ctrl_msg.current_ma.push_back(target.hip_pitch_.servo_current_);
      ctrl_msg.current_ma.push_back(target.knee_pitch_.servo_current_);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);

      diff_hy_pre[i] = diff_hy;
      diff_hp_pre[i] = diff_hp;
      diff_kp_pre[i] = diff_kp;
    }
  }

  void AppendVelocityCommand(DxlCommandsX& dyn_msg) {
    auto& ctrl_msg = dyn_msg.velocity_control;
    static std::array<double, 4> diff_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> diff_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> diff_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now().seconds();
    for (std::size_t i = 0; i < target_legs_.size(); ++i) {
      auto& target = target_legs_[i];
      const auto& present = present_legs_[i];
      const double dt = now - present.updated_time_;

      const double target_hy = target.hip_yaw_.servo_angle_;
      const double target_hp = target.hip_pitch_.servo_angle_;
      const double target_kp = target.knee_pitch_.servo_angle_;
      const double present_hy = present.hip_yaw_.servo_angle_   + dt * present.hip_yaw_.servo_velocity_;
      const double present_hp = present.hip_pitch_.servo_angle_ + dt * present.hip_pitch_.servo_velocity_;
      const double present_kp = present.knee_pitch_.servo_angle_ + dt * present.knee_pitch_.servo_velocity_;

      const double diff_hy = target_hy - present_hy;
      const double diff_hp = target_hp - present_hp;
      const double diff_kp = target_kp - present_kp;

      const double vel_hy = std::fabs(diff_hy) < deadband_ ? 0.0 : outer_gain_p_ * diff_hy + outer_gain_d_ * (diff_hy - diff_hy_pre[i]);
      const double vel_hp = std::fabs(diff_hp) < deadband_ ? 0.0 : outer_gain_p_ * diff_hp + outer_gain_d_ * (diff_hp - diff_hp_pre[i]);
      const double vel_kp = std::fabs(diff_kp) < deadband_ ? 0.0 : outer_gain_p_ * diff_kp + outer_gain_d_ * (diff_kp - diff_kp_pre[i]);

      ctrl_msg.id_list.push_back(target.hip_yaw_.id_);
      ctrl_msg.id_list.push_back(target.hip_pitch_.id_);
      ctrl_msg.id_list.push_back(target.knee_pitch_.id_);
      ctrl_msg.velocity_deg_s.push_back(vel_hy * rad_to_deg);
      ctrl_msg.velocity_deg_s.push_back(vel_hp * rad_to_deg);
      ctrl_msg.velocity_deg_s.push_back(vel_kp * rad_to_deg);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      ctrl_msg.profile_acc_deg_ss.push_back(0.0);

      diff_hy_pre[i] = diff_hy;
      diff_hp_pre[i] = diff_hp;
      diff_kp_pre[i] = diff_kp;
    }
  }

  void AppendPositionCommand(DxlCommandsX& dyn_msg) {
    auto& ctrl_msg = dyn_msg.current_base_position_control;
    static auto prev = this->get_clock()->now();
    static double dt = 0.01;
    static std::array<double, 4> ang_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> ang_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static std::array<double, 4> ang_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now();
    dt = 0.8 * dt + 0.2 * (now.seconds() - prev.seconds());
    prev = now;

    for (std::size_t i = 0; i < target_legs_.size(); ++i) {
      auto& target = target_legs_[i];
      const auto& present = present_legs_[i];

      const double prev_hy = ang_hy_pre[i];
      const double prev_hp = ang_hp_pre[i];
      const double prev_kp = ang_kp_pre[i];
      const double target_hy = ang_hy_pre[i] = target.hip_yaw_.servo_angle_;
      const double target_hp = ang_hp_pre[i] = target.hip_pitch_.servo_angle_;
      const double target_kp = ang_kp_pre[i] = target.knee_pitch_.servo_angle_;
      const double present_hy = present.hip_yaw_.servo_angle_;
      const double present_hp = present.hip_pitch_.servo_angle_;
      const double present_kp = present.knee_pitch_.servo_angle_;

      ctrl_msg.id_list.push_back(target.hip_yaw_.id_);
      ctrl_msg.id_list.push_back(target.hip_pitch_.id_);
      ctrl_msg.id_list.push_back(target.knee_pitch_.id_);
      ctrl_msg.position_deg.push_back(target_hy * rad_to_deg);
      ctrl_msg.position_deg.push_back(target_hp * rad_to_deg);
      ctrl_msg.position_deg.push_back(target_kp * rad_to_deg);
      ctrl_msg.current_ma.push_back(target.hip_yaw_.servo_current_);
      ctrl_msg.current_ma.push_back(target.hip_pitch_.servo_current_);
      ctrl_msg.current_ma.push_back(target.knee_pitch_.servo_current_);

      if (i == 0 || i == 1) {
        ctrl_msg.profile_vel_deg_s.push_back((target_hy - present_hy) / dt * rad_to_deg);
        ctrl_msg.profile_vel_deg_s.push_back((target_hp - present_hp) / dt * rad_to_deg);
        ctrl_msg.profile_vel_deg_s.push_back((target_kp - present_kp) / dt * rad_to_deg);
        ctrl_msg.profile_acc_deg_ss.push_back(i == 0 ? 5000.0 : 0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(i == 0 ? 5000.0 : 0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(i == 0 ? 5000.0 : 0.0);
      } else if (i == 2) {
        ctrl_msg.profile_vel_deg_s.push_back(0.0);
        ctrl_msg.profile_vel_deg_s.push_back(0.0);
        ctrl_msg.profile_vel_deg_s.push_back(0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      } else {
        ctrl_msg.profile_vel_deg_s.push_back((target_hy - prev_hy) / dt * rad_to_deg);
        ctrl_msg.profile_vel_deg_s.push_back((target_hp - prev_hp) / dt * rad_to_deg);
        ctrl_msg.profile_vel_deg_s.push_back((target_kp - prev_kp) / dt * rad_to_deg);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
        ctrl_msg.profile_acc_deg_ss.push_back(0.0);
      }
    }
  }

  void PublishLegState(const std::array<Leg, 4>& legs, const rclcpp::Publisher<QuadRobotLeg>::SharedPtr& publisher) const {
    // TODO: For HEAD compatibility, revisit state publish behavior later.
    // HEAD populated only updated legs in each message and skipped publish when nothing changed.
    // The refactor currently publishes a full 4-leg snapshot every time this helper is called.
    QuadRobotLeg state_msg;
    state_msg.stamp = this->now();
    state_msg.angles_fr = legs[front_right].GetJointAngles();
    state_msg.angles_fl = legs[front_left].GetJointAngles();
    state_msg.angles_br = legs[back_right].GetJointAngles();
    state_msg.angles_bl = legs[back_left].GetJointAngles();

    state_msg.torques_fr = legs[front_right].GetJointTorques();
    state_msg.torques_fl = legs[front_left].GetJointTorques();
    state_msg.torques_br = legs[back_right].GetJointTorques();
    state_msg.torques_bl = legs[back_left].GetJointTorques();

    state_msg.point_fr = LegForwardKinematics(state_msg.angles_fr, legs[front_right].fixed_pose_, robot_profile_.legs[front_right].mount.sign, robot_profile_.link_lengths);
    state_msg.point_fl = LegForwardKinematics(state_msg.angles_fl, legs[front_left].fixed_pose_, robot_profile_.legs[front_left].mount.sign, robot_profile_.link_lengths);
    state_msg.point_br = LegForwardKinematics(state_msg.angles_br, legs[back_right].fixed_pose_, robot_profile_.legs[back_right].mount.sign, robot_profile_.link_lengths);
    state_msg.point_bl = LegForwardKinematics(state_msg.angles_bl, legs[back_left].fixed_pose_, robot_profile_.legs[back_left].mount.sign, robot_profile_.link_lengths);

    state_msg.force_fr = LegForwardStatics(state_msg.angles_fr, legs[front_right].fixed_pose_, robot_profile_.legs[front_right].mount.sign, state_msg.torques_fr, robot_profile_.link_lengths);
    state_msg.force_fl = LegForwardStatics(state_msg.angles_fl, legs[front_left].fixed_pose_, robot_profile_.legs[front_left].mount.sign, state_msg.torques_fl, robot_profile_.link_lengths);
    state_msg.force_br = LegForwardStatics(state_msg.angles_br, legs[back_right].fixed_pose_, robot_profile_.legs[back_right].mount.sign, state_msg.torques_br, robot_profile_.link_lengths);
    state_msg.force_bl = LegForwardStatics(state_msg.angles_bl, legs[back_left].fixed_pose_, robot_profile_.legs[back_left].mount.sign, state_msg.torques_bl, robot_profile_.link_lengths);
    publisher->publish(state_msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegNode>());
  rclcpp::shutdown();
  return 0;
}
