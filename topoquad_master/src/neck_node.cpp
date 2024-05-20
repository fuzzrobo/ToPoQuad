#include <topoquad_master/neck_node.hpp>

class NeckNode : public rclcpp::Node {
    rclcpp::Publisher<dynamixel_handler::msg::DynamixelCommandXControlPosition>::SharedPtr dyn_cmd_pub_;
    rclcpp::Publisher<topoquad_msgs::msg::QuadRobotStateNeck>::SharedPtr neck_state_p_pub_, neck_state_g_pub_;

    rclcpp::Subscription<dynamixel_handler::msg::DynamixelState>::SharedPtr dyn_state_sub_;
    rclcpp::Subscription<topoquad_msgs::msg::QuadRobotCmdNeckAngle>::SharedPtr neck_angle_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Neck target_neck_, goal_neck_, present_neck_;

   public:
    NeckNode()
        : Node("neck_node") {
        // Parameters
        this->declare_parameter<std::vector<int64_t>>("pantilt_dynamixel_ID", std::vector<int64_t>({43, 42}));

        auto ids_pantilt = this->get_parameter("pantilt_dynamixel_ID").as_integer_array();

        target_neck_.initialize(Joint{ids_pantilt[0], +1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.5 /*Nm*/},
                                Joint{ids_pantilt[1], +1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.5 /*Nm*/});
        goal_neck_ = target_neck_;
        present_neck_ = target_neck_;

        // Publishers
        dyn_cmd_pub_ = this->create_publisher<dynamixel_handler::msg::DynamixelCommandXControlPosition>("dynamixel/cmd/x/position", 10);
        neck_state_p_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateNeck>("neck/state/present", 10);
        neck_state_g_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateNeck>("neck/state/goal", 10);

        // Subscribers
        dyn_state_sub_ = this->create_subscription<dynamixel_handler::msg::DynamixelState>(
            "dynamixel/state", 10, std::bind(&NeckNode::dyn_state_cb, this, _1));
        neck_angle_sub_ = this->create_subscription<topoquad_msgs::msg::QuadRobotCmdNeckAngle>(
            "neck/angle", 10, std::bind(&NeckNode::neck_angle_cb, this, _1));

        // Timer
        timer_ = this->create_wall_timer(5ms, std::bind(&NeckNode::timer_cb, this));
    }

   private:
    void timer_cb() {
        if (target_neck_.is_updated_ || target_neck_ != goal_neck_) {
            dynamixel_handler::msg::DynamixelCommandXControlPosition dyn_msg;
            dyn_msg.id_list.push_back(target_neck_.pan_.id_);
            dyn_msg.id_list.push_back(target_neck_.tilt_.id_);
            dyn_msg.position_deg.push_back(target_neck_.pan_.servo_angle_ * rad2deg);
            dyn_msg.position_deg.push_back(target_neck_.tilt_.servo_angle_ * rad2deg);
            dyn_msg.profile_vel_deg_s.push_back(0);
            dyn_msg.profile_vel_deg_s.push_back(0);
            dyn_cmd_pub_->publish(dyn_msg);
            target_neck_.is_updated_ = false;
        }

        if (present_neck_.is_updated_) {
            topoquad_msgs::msg::QuadRobotStateNeck neck_msg_p;
            neck_msg_p.angle_pan = present_neck_.pan_.joint_angle_;
            neck_msg_p.angle_tilt = present_neck_.tilt_.joint_angle_;
            neck_state_p_pub_->publish(neck_msg_p);
            present_neck_.is_updated_ = false;
        }

        if (goal_neck_.is_updated_) {
            topoquad_msgs::msg::QuadRobotStateNeck neck_msg_g;
            neck_msg_g.angle_pan = goal_neck_.pan_.joint_angle_;
            neck_msg_g.angle_tilt = goal_neck_.tilt_.joint_angle_;
            neck_state_g_pub_->publish(neck_msg_g);
            goal_neck_.is_updated_ = false;
        }
    }
    void dyn_state_cb(const dynamixel_handler::msg::DynamixelState::SharedPtr msg) {
        for (int i = 0; i < msg->id_list.size(); i++) {
            if (msg->id_list[i] == present_neck_.pan_.id_) present_neck_.pan_.SetServoAngle(msg->position_deg[i] * deg2rad);
            if (msg->id_list[i] == present_neck_.tilt_.id_) present_neck_.tilt_.SetServoAngle(msg->position_deg[i] * deg2rad);
            present_neck_.is_updated_ = true;
            if (msg->id_list[i] == goal_neck_.pan_.id_) goal_neck_.pan_.SetServoAngle(msg->position_deg[i] * rad2deg);
            if (msg->id_list[i] == goal_neck_.tilt_.id_) goal_neck_.tilt_.SetServoAngle(msg->position_deg[i] * rad2deg);
            goal_neck_.is_updated_ = true;
        }
    }
    void neck_angle_cb(const topoquad_msgs::msg::QuadRobotCmdNeckAngle::SharedPtr msg) {
        target_neck_.SetJointAngles(msg->angle_pan, msg->angle_tilt);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NeckNode>());
    rclcpp::shutdown();
    return 0;
}