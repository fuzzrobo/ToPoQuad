#include "neck_node.hpp"

#include <functional>
#include <memory>
#include <vector>

#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoquad_msgs/msg/quad_robot_neck.hpp"

using namespace dynamixel_handler_msgs::msg;
using namespace topoquad_msgs::msg;

class NeckNode : public rclcpp::Node {
 public:
  NeckNode() : Node("neck_node") {
    const auto ids_pantilt = this->declare_parameter<std::vector<int64_t>>("pantilt_dynamixel_ID", std::vector<int64_t>({43, 42}));

    target_neck_.initialize(Joint{(uint8_t)ids_pantilt[0], +1.0, 0.0, +0.92 / 800.0, 0.5}, Joint{(uint8_t)ids_pantilt[1], +1.0, 0.0, +0.92 / 800.0, 0.5});
    goal_neck_ = target_neck_;
    present_neck_ = target_neck_;

    dyn_cmd_pub_      = this->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
    neck_state_p_pub_ = this->create_publisher<QuadRobotNeck>("neck/state/present", 10);
    neck_state_g_pub_ = this->create_publisher<QuadRobotNeck>("neck/state/goal", 10);

    using std::placeholders::_1;
    dyn_state_sub_    = this->create_subscription<DxlStates>("dynamixel/states", 10, std::bind(&NeckNode::dyn_state_cb, this, _1));
    neck_command_sub_ = this->create_subscription<QuadRobotNeck>("neck/command", 10, std::bind(&NeckNode::neck_command_cb, this, _1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&NeckNode::timer_cb, this));
  }

 private:
  rclcpp::Publisher<DxlCommandsX>::SharedPtr dyn_cmd_pub_;
  rclcpp::Publisher<QuadRobotNeck>::SharedPtr neck_state_p_pub_, neck_state_g_pub_;
  rclcpp::Subscription<DxlStates>::SharedPtr dyn_state_sub_;
  rclcpp::Subscription<QuadRobotNeck>::SharedPtr neck_command_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Neck target_neck_;
  Neck goal_neck_;
  Neck present_neck_;

  void timer_cb() {
    if (target_neck_.is_updated_ || target_neck_ != goal_neck_) {
      DxlCommandsX dyn_msg;
      auto& ctrl_msg = dyn_msg.position_control;
      ctrl_msg.id_list.push_back(target_neck_.pan_.id_);
      ctrl_msg.id_list.push_back(target_neck_.tilt_.id_);
      ctrl_msg.position_deg.push_back(target_neck_.pan_.servo_angle_ * rad_to_deg);
      ctrl_msg.position_deg.push_back(target_neck_.tilt_.servo_angle_ * rad_to_deg);
      ctrl_msg.profile_vel_deg_s.push_back(0.0);
      ctrl_msg.profile_vel_deg_s.push_back(0.0);
      dyn_cmd_pub_->publish(dyn_msg);
      target_neck_.is_updated_ = false;
    }

    if (present_neck_.is_updated_) {
      QuadRobotNeck neck_msg_p;
      neck_msg_p.stamp = this->get_clock()->now();
      neck_msg_p.angle_pan = present_neck_.pan_.joint_angle_;
      neck_msg_p.angle_tilt = present_neck_.tilt_.joint_angle_;
      neck_msg_p.torques_pan = present_neck_.pan_.joint_torque_;
      neck_msg_p.torques_tilt = present_neck_.tilt_.joint_torque_;
      neck_state_p_pub_->publish(neck_msg_p);
      present_neck_.is_updated_ = false;
    }

    if (goal_neck_.is_updated_) {
      QuadRobotNeck neck_msg_g;
      neck_msg_g.stamp = this->get_clock()->now();
      neck_msg_g.angle_pan = goal_neck_.pan_.joint_angle_;
      neck_msg_g.angle_tilt = goal_neck_.tilt_.joint_angle_;
      neck_msg_g.torques_pan = goal_neck_.pan_.joint_torque_;
      neck_msg_g.torques_tilt = goal_neck_.tilt_.joint_torque_;
      neck_state_g_pub_->publish(neck_msg_g);
      goal_neck_.is_updated_ = false;
    }
  }

  void dyn_state_cb(const DxlStates::SharedPtr msg) {
    for (std::size_t i = 0; i < msg->present.id_list.size(); ++i) {
      if (msg->present.id_list[i] == present_neck_.pan_.id_) {
        present_neck_.pan_.servo_velocity_ = msg->present.velocity_deg_s[i] * deg_to_rad;
        present_neck_.pan_.SetServoAngle(msg->present.position_deg[i] * deg_to_rad);
        present_neck_.pan_.SetServoCurrent(msg->present.current_ma[i]);
      }
      if (msg->present.id_list[i] == present_neck_.tilt_.id_) {
        present_neck_.tilt_.servo_velocity_ = msg->present.velocity_deg_s[i] * deg_to_rad;
        present_neck_.tilt_.SetServoAngle(msg->present.position_deg[i] * deg_to_rad);
        present_neck_.tilt_.SetServoCurrent(msg->present.current_ma[i]);
      }
      // TODO(ryo_michi): Legacy baseline updates publish flag even for unrelated IDs.
      // Keep for refactor parity; tighten to ID-matched updates in a dedicated bugfix patch.
      present_neck_.is_updated_ = true;
    }

    for (std::size_t i = 0; i < msg->goal.id_list.size(); ++i) {
      if (msg->goal.id_list[i] == goal_neck_.pan_.id_) {
        goal_neck_.pan_.SetServoAngle(msg->goal.position_deg[i] * deg_to_rad);
        goal_neck_.pan_.SetServoCurrent(msg->goal.current_ma[i]);
      }
      if (msg->goal.id_list[i] == goal_neck_.tilt_.id_) {
        goal_neck_.tilt_.SetServoAngle(msg->goal.position_deg[i] * deg_to_rad);
        goal_neck_.tilt_.SetServoCurrent(msg->goal.current_ma[i]);
      }
      // TODO(ryo_michi): Legacy baseline marks goal updated regardless of ID match.
      // Keep for refactor parity; tighten to ID-matched updates in a dedicated bugfix patch.
      goal_neck_.is_updated_ = true;
    }
  }

  void neck_command_cb(const QuadRobotNeck::SharedPtr msg) {
    target_neck_.SetJointAngles(msg->angle_pan, msg->angle_tilt);
    if (msg->torques_pan  != 0.0) target_neck_.pan_.SetJointTorque(msg->torques_pan);
    if (msg->torques_tilt != 0.0) target_neck_.tilt_.SetJointTorque(msg->torques_tilt);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeckNode>());
  rclcpp::shutdown();
  return 0;
}
