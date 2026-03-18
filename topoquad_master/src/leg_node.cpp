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
using std::array;

constexpr array<const char*, 4> leg_suffixes = {"fr", "fl", "br", "bl"};
constexpr std::chrono::milliseconds timer_period(50);
constexpr std::chrono::milliseconds background_period(200);

enum LegIndex : size_t {
  FR = 0,
  FL = 1,
  BR = 2,
  BL = 3,
};

class LegNode : public rclcpp::Node {
 public:
  LegNode() : Node("leg_node"), robot_profile_(MakeDefaultRobotProfile()) {
    prev_cmd_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    LoadRobotProfile();
    DeclareControlParameters();
    for (size_t i = 0; i < target_legs_.size(); ++i) target_legs_[i].initialize(robot_profile_.legs[i]);
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
  array<Leg, 4> target_legs_;
  array<Leg, 4> goal_legs_;
  array<Leg, 4> present_legs_;

  double outer_gain_p_{5.0};
  double outer_gain_d_{0.0};
  double dxl_vel_gain_{50.0};
  double dxl_pos_gain_{800.0};
  double overshoot_{50.0 * deg_to_rad};
  double deadband_{1.0 * deg_to_rad};
  ControlMode control_mode_{MODE_POSITION};

  template <size_t N>
  array<double, N> DeclareDoubleArray(const string& name, const array<double, N>& default_values) {
    const vector<double> default_vector(default_values.begin(), default_values.end());
    const auto values = this->declare_parameter<vector<double>>(name, default_vector);
    if (values.size() != N) throw std::runtime_error(name + " must have " + std::to_string(N) + " elements");

    array<double, N> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void LoadRobotProfile() {
    robot_profile_.link_lengths.hip_yaw    = this->declare_parameter<double>("legs.link_lengths.hip_yaw", robot_profile_.link_lengths.hip_yaw);
    robot_profile_.link_lengths.hip_pitch  = this->declare_parameter<double>("legs.link_lengths.hip_pitch", robot_profile_.link_lengths.hip_pitch);
    robot_profile_.link_lengths.knee_pitch = this->declare_parameter<double>("legs.link_lengths.knee_pitch", robot_profile_.link_lengths.knee_pitch);

    for (size_t i = 0; i < leg_suffixes.size(); ++i) {
      auto& leg = robot_profile_.legs[i];

      const string mnt = string("legs.mounts.") + leg_suffixes[i];
      leg.mount.position_Rtheta = DeclareDoubleArray<2>(mnt + ".position_polar", leg.mount.position_Rtheta);
      leg.mount.yaw_deg         = this->declare_parameter<double>(mnt + ".yaw_deg", leg.mount.yaw_deg);

      const string jnt = string("legs.joints.") + leg_suffixes[i];
      auto& hy = leg.hip_yaw;
      hy.gear_ratio     = this->declare_parameter<double>(jnt + ".hip_yaw.gear_ratio", hy.gear_ratio);
      hy.torque_ratio   = this->declare_parameter<double>(jnt + ".hip_yaw.torque_ratio", hy.torque_ratio);
      hy.default_torque = this->declare_parameter<double>(jnt + ".hip_yaw.default_torque", hy.default_torque);
      auto& hp = leg.hip_pitch;
      hp.gear_ratio     = this->declare_parameter<double>(jnt + ".hip_pitch.gear_ratio", hp.gear_ratio);
      hp.torque_ratio   = this->declare_parameter<double>(jnt + ".hip_pitch.torque_ratio", hp.torque_ratio);
      hp.default_torque = this->declare_parameter<double>(jnt + ".hip_pitch.default_torque", hp.default_torque);
      auto& kp = leg.knee_pitch;
      kp.gear_ratio     = this->declare_parameter<double>(jnt + ".knee_pitch.gear_ratio", kp.gear_ratio);
      kp.torque_ratio   = this->declare_parameter<double>(jnt + ".knee_pitch.torque_ratio", kp.torque_ratio);
      kp.default_torque = this->declare_parameter<double>(jnt + ".knee_pitch.default_torque", kp.default_torque);

      const string dyn = string("leg_dynamixel_IDs.") + leg_suffixes[i];
      auto id_list = this->declare_parameter<vector<int>>(dyn, {1,2,3});
      if (id_list.size() != 3) throw std::runtime_error(dyn + " must have 3 elements");
      hy.id = (uint8_t)id_list[0]; hp.id = (uint8_t)id_list[1]; kp.id = (uint8_t)id_list[2];

      const string ini = string("initial_pose_deg.") + leg_suffixes[i];
      leg.initial_pose_deg = DeclareDoubleArray<3>(ini, leg.initial_pose_deg);
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

    DxlCommandsX dyn;
    
    for (const auto& leg : target_legs_) {
      dyn.status.id_list.push_back(leg.hip_yaw_.id_);
      dyn.status.id_list.push_back(leg.hip_pitch_.id_);
      dyn.status.id_list.push_back(leg.knee_pitch_.id_);
      dyn.status.error.push_back(false);
      dyn.status.error.push_back(false);
      dyn.status.error.push_back(false);
    }

    auto& ctrl = dyn.current_base_position_control;
    for (size_t i = 0; i < target_legs_.size(); ++i) {
      const auto& leg = target_legs_[i];
      const auto& rp = robot_profile_.legs[i];
      ctrl.id_list.push_back(leg.hip_yaw_.id_);
      ctrl.id_list.push_back(leg.hip_pitch_.id_);
      ctrl.id_list.push_back(leg.knee_pitch_.id_);
      ctrl.position_deg.push_back(rp.initial_pose_deg[0]);
      ctrl.position_deg.push_back(rp.initial_pose_deg[1]);
      ctrl.position_deg.push_back(rp.initial_pose_deg[2]);
      ctrl.profile_vel_deg_s.push_back(50.0 /*deg/s*/);
      ctrl.profile_vel_deg_s.push_back(50.0 /*deg/s*/);
      ctrl.profile_vel_deg_s.push_back(50.0 /*deg/s*/);
      // todo 
      // ctrl.profile_vel_deg_s.push_back(500.0 /*deg/s*/);
      // ctrl.profile_vel_deg_s.push_back(500.0 /*deg/s*/);
      // ctrl.profile_vel_deg_s.push_back(500.0 /*deg/s*/);
      // ctrl.current_ma.push_back(rp.hip_yaw.default_torque / rp.hip_yaw.torque_ratio);
      // ctrl.current_ma.push_back(rp.hip_pitch.default_torque / rp.hip_pitch.torque_ratio);
      // ctrl.current_ma.push_back(rp.knee_pitch.default_torque / rp.knee_pitch.torque_ratio);
    }

    dyn_cmd_pub_->publish(dyn);
  }

  void leg_command_cb(const QuadRobotLeg::SharedPtr msg) {
    array<bool, 4> active_legs = {false, false, false, false};

    active_legs[FR] = ApplyKinematics(FR, msg->angles_fr, msg->point_fr);
    active_legs[FL]  = ApplyKinematics(FL, msg->angles_fl, msg->point_fl);
    active_legs[BR]  = ApplyKinematics(BR, msg->angles_br, msg->point_br);
    active_legs[BL]   = ApplyKinematics(BL, msg->angles_bl, msg->point_bl);

    active_legs[FR] = ApplyEffort(FR, msg->torques_fr, msg->force_fr) || active_legs[FR];
    active_legs[FL]  = ApplyEffort(FL, msg->torques_fl, msg->force_fl) || active_legs[FL];
    active_legs[BR]  = ApplyEffort(BR, msg->torques_br, msg->force_br) || active_legs[BR];
    active_legs[BL]   = ApplyEffort(BL, msg->torques_bl, msg->force_bl) || active_legs[BL];

    DxlCommandsX dyn_msg;
    switch (control_mode_) {
      case MODE_VELOCITY:      AppendVelocityCommand(dyn_msg); break;
      case MODE_POSITION:      AppendPositionCommand(dyn_msg); break;
      case MODE_VELOCITY_SAFE: AppendVelocityPositionCommand(dyn_msg); break;
      default: RCLCPP_ERROR(get_logger(), "Unknown control mode: %d", (int)control_mode_); break;
    }
    dyn_cmd_pub_->publish(dyn_msg);
    prev_cmd_time_ = this->get_clock()->now();

    PublishTargetLegState(active_legs);
  }

  void PublishTargetLegState(const array<bool, 4>& active_legs) const {
    QuadRobotLeg msg;
    msg.stamp = this->now();
    auto& rp = robot_profile_;

    if (active_legs[FR]) {
      msg.angles_fr = target_legs_[FR].GetJointAngles();
      msg.torques_fr = target_legs_[FR].GetJointTorques();
      msg.point_fr = LegForwardKinematics(msg.angles_fr, target_legs_[FR].fixed_pose_, rp.legs[FR].mount.sign, rp.link_lengths);
      msg.force_fr = LegForwardStatics   (msg.angles_fr, target_legs_[FR].fixed_pose_, rp.legs[FR].mount.sign, msg.torques_fr, rp.link_lengths);
    }
    if (active_legs[FL]) {
      msg.angles_fl = target_legs_[FL].GetJointAngles();
      msg.torques_fl = target_legs_[FL].GetJointTorques();
      msg.point_fl = LegForwardKinematics(msg.angles_fl, target_legs_[FL].fixed_pose_, rp.legs[FL].mount.sign, rp.link_lengths);
      msg.force_fl = LegForwardStatics   (msg.angles_fl, target_legs_[FL].fixed_pose_, rp.legs[FL].mount.sign, msg.torques_fl, rp.link_lengths);
    }
    if (active_legs[BR]) {
      msg.angles_br = target_legs_[BR].GetJointAngles();
      msg.torques_br = target_legs_[BR].GetJointTorques();
      msg.point_br = LegForwardKinematics(msg.angles_br, target_legs_[BR].fixed_pose_, rp.legs[BR].mount.sign, rp.link_lengths);
      msg.force_br = LegForwardStatics   (msg.angles_br, target_legs_[BR].fixed_pose_, rp.legs[BR].mount.sign, msg.torques_br, rp.link_lengths);
    }
    if (active_legs[BL]) {
      msg.angles_bl = target_legs_[BL].GetJointAngles();
      msg.torques_bl = target_legs_[BL].GetJointTorques();
      msg.point_bl = LegForwardKinematics(msg.angles_bl, target_legs_[BL].fixed_pose_, rp.legs[BL].mount.sign, rp.link_lengths);
      msg.force_bl = LegForwardStatics   (msg.angles_bl, target_legs_[BL].fixed_pose_, rp.legs[BL].mount.sign, msg.torques_bl, rp.link_lengths);
    }

    leg_state_t_pub_->publish(msg);
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

  bool ApplyKinematics(size_t leg_index, const vector<double>& angles, const Point& point) {
    auto& leg = target_legs_[leg_index];
    if (!angles.empty()) {
      leg.SetJointAngles(angles);
      return true;
    }
    if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0) return false;

    const auto ik_angles = LegInverseKinematics(point, leg.fixed_pose_, robot_profile_.legs[leg_index].mount.sign, robot_profile_.link_lengths);
    if (!isnan(ik_angles[0]) && !isnan(ik_angles[1]) && !isnan(ik_angles[2])) {
      leg.SetJointAngles(ik_angles);
      return true;
    }
    return false;
  }

  bool ApplyEffort(size_t leg_index, const vector<double>& torques, const Vector3& force) {
    if (!torques.empty()) {
      target_legs_[leg_index].SetJointTorques(torques);
      return true;
    }
    if (force.x == 0.0 && force.y == 0.0 && force.z == 0.0) return false;

    const auto current_angles = present_legs_[leg_index].GetJointAngles();
    const auto joint_torques = LegInverseStatics(current_angles, target_legs_[leg_index].fixed_pose_, robot_profile_.legs[leg_index].mount.sign, force, robot_profile_.link_lengths);
    target_legs_[leg_index].SetJointTorques(joint_torques);
    return true;
  }

  void UpdateLegsFromPresentState(const DxlStates::SharedPtr& msg, double state_time) {
    const auto& present = msg->present;
    if (present.id_list.empty()) return;

    for (auto& leg : present_legs_) {
      bool matched = false;
      // TODO(ryo_michi): Legacy baseline iterates id_list length without array-size guards.
      // Keep for refactor parity; add bounds checks in a dedicated bugfix patch later.
      for (size_t i = 0; i < present.id_list.size(); ++i) {
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
      // TODO(ryo_michi): Legacy baseline marks leg updated regardless of ID match,
      // and iterates id_list length without bounds guards. Keep for refactor parity.
      for (size_t i = 0; i < goal.id_list.size(); ++i) {
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

  void AppendVelocityPositionCommand(DxlCommandsX& dyn_msg) {
    auto& ctrl_msg = dyn_msg.current_base_position_control;
    static array<double, 4> diff_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> diff_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> diff_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now().seconds();
    for (size_t i = 0; i < target_legs_.size(); ++i) {
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
    static array<double, 4> diff_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> diff_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> diff_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now().seconds();
    for (size_t i = 0; i < target_legs_.size(); ++i) {
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
    static array<double, 4> ang_hy_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> ang_hp_pre = {0.0, 0.0, 0.0, 0.0};
    static array<double, 4> ang_kp_pre = {0.0, 0.0, 0.0, 0.0};

    const auto now = this->get_clock()->now();
    dt = 0.8 * dt + 0.2 * (now.seconds() - prev.seconds());
    prev = now;

    for (size_t i = 0; i < target_legs_.size(); ++i) {
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

  void PublishLegState(const array<Leg, 4>& legs, const rclcpp::Publisher<QuadRobotLeg>::SharedPtr& publisher) const {
    QuadRobotLeg msg;
    msg.stamp = this->now();
    msg.angles_fr = legs[FR].GetJointAngles();
    msg.angles_fl = legs[FL].GetJointAngles();
    msg.angles_br = legs[BR].GetJointAngles();
    msg.angles_bl = legs[BL].GetJointAngles();

    msg.torques_fr = legs[FR].GetJointTorques();
    msg.torques_fl = legs[FL].GetJointTorques();
    msg.torques_br = legs[BR].GetJointTorques();
    msg.torques_bl = legs[BL].GetJointTorques();

    msg.point_fr = LegForwardKinematics(msg.angles_fr, legs[FR].fixed_pose_, robot_profile_.legs[FR].mount.sign, robot_profile_.link_lengths);
    msg.point_fl = LegForwardKinematics(msg.angles_fl, legs[FL].fixed_pose_, robot_profile_.legs[FL].mount.sign, robot_profile_.link_lengths);
    msg.point_br = LegForwardKinematics(msg.angles_br, legs[BR].fixed_pose_, robot_profile_.legs[BR].mount.sign, robot_profile_.link_lengths);
    msg.point_bl = LegForwardKinematics(msg.angles_bl, legs[BL].fixed_pose_, robot_profile_.legs[BL].mount.sign, robot_profile_.link_lengths);

    msg.force_fr = LegForwardStatics(msg.angles_fr, legs[FR].fixed_pose_, robot_profile_.legs[FR].mount.sign, msg.torques_fr, robot_profile_.link_lengths);
    msg.force_fl = LegForwardStatics(msg.angles_fl, legs[FL].fixed_pose_, robot_profile_.legs[FL].mount.sign, msg.torques_fl, robot_profile_.link_lengths);
    msg.force_br = LegForwardStatics(msg.angles_br, legs[BR].fixed_pose_, robot_profile_.legs[BR].mount.sign, msg.torques_br, robot_profile_.link_lengths);
    msg.force_bl = LegForwardStatics(msg.angles_bl, legs[BL].fixed_pose_, robot_profile_.legs[BL].mount.sign, msg.torques_bl, robot_profile_.link_lengths);
    publisher->publish(msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegNode>());
  rclcpp::shutdown();
  return 0;
}
