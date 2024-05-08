#include <topoquad_master/leg_node.hpp>

class LegNode : public rclcpp::Node {
    rclcpp::Publisher<dynamixel_handler::msg::DynamixelCommandXControlCurrentPosition>::SharedPtr dyn_cmd_pub_;
    rclcpp::Publisher<topoquad_msgs::msg::QuadRobotStateLeg>::SharedPtr leg_state_p_pub_, leg_state_g_pub_;

    rclcpp::Subscription<topoquad_msgs::msg::QuadRobotCmdLegPoint>::SharedPtr leg_point_sub_;
    rclcpp::Subscription<topoquad_msgs::msg::QuadRobotCmdLegAngle>::SharedPtr leg_angle_sub_;
    rclcpp::Subscription<dynamixel_handler::msg::DynamixelState>::SharedPtr dyn_state_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Leg target_leg_fr_, goal_leg_fr_, present_leg_fr_;
    Leg target_leg_fl_, goal_leg_fl_, present_leg_fl_;
    Leg target_leg_br_, goal_leg_br_, present_leg_br_;
    Leg target_leg_bl_, goal_leg_bl_, present_leg_bl_;

   public:
    LegNode()
        : Node("leg_node"),
          target_leg_fr_(LENGTH_BASE * cos(ANGLE_FR), LENGTH_BASE * sin(ANGLE_FR), ANGLE_FR),
          target_leg_fl_(LENGTH_BASE * cos(ANGLE_FL), LENGTH_BASE * sin(ANGLE_FL), ANGLE_FL),
          target_leg_br_(LENGTH_BASE * cos(ANGLE_BR), LENGTH_BASE * sin(ANGLE_BR), ANGLE_BR),
          target_leg_bl_(LENGTH_BASE * cos(ANGLE_BL), LENGTH_BASE * sin(ANGLE_BL), ANGLE_BL) {
        // Parameters
        this->declare_parameter<std::vector<int64_t>>("BR_leg_dynamixel_ID", std::vector<int64_t>({4, 3, 2}));
        this->declare_parameter<std::vector<int64_t>>("FR_leg_dynamixel_ID", std::vector<int64_t>({14, 13, 12}));
        this->declare_parameter<std::vector<int64_t>>("FL_leg_dynamixel_ID", std::vector<int64_t>({24, 23, 22}));
        this->declare_parameter<std::vector<int64_t>>("BL_leg_dynamixel_ID", std::vector<int64_t>({34, 33, 32}));

        auto ids_br = this->get_parameter("BR_leg_dynamixel_ID").as_integer_array();
        auto ids_fr = this->get_parameter("FR_leg_dynamixel_ID").as_integer_array();
        auto ids_fl = this->get_parameter("FL_leg_dynamixel_ID").as_integer_array();
        auto ids_bl = this->get_parameter("BL_leg_dynamixel_ID").as_integer_array();

        target_leg_br_.initialize(Joint{ids_br[0], -1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_br[1], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_br[2], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/});
        target_leg_fr_.initialize(Joint{ids_fr[0], -1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_fr[1], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_fr[2], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/});
        target_leg_fl_.initialize(Joint{ids_fl[0], +1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_fl[1], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_fl[2], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/});
        target_leg_bl_.initialize(Joint{ids_bl[0], +1.0, 0.0 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_bl[1], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/},
                                  Joint{ids_bl[2], +1.0, M_PI / 4 /*rad*/, +0.92 / 800 /*Nm/mA*/, 0.6 /*Nm*/});

        present_leg_fr_ = target_leg_fr_;
        present_leg_fl_ = target_leg_fl_;
        present_leg_br_ = target_leg_br_;
        present_leg_bl_ = target_leg_bl_;

        goal_leg_fr_ = target_leg_fr_;
        goal_leg_fl_ = target_leg_fl_;
        goal_leg_br_ = target_leg_br_;
        goal_leg_bl_ = target_leg_bl_;

        // Publishers
        dyn_cmd_pub_ = this->create_publisher<dynamixel_handler::msg::DynamixelCommandXControlCurrentPosition>("/dynamixel/cmd/x/current_position", 10);
        leg_state_p_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateLeg>("/legs/state/present", 10);
        leg_state_g_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateLeg>("/legs/state/goal", 10);

        // Subscribers
        leg_point_sub_ = this->create_subscription<topoquad_msgs::msg::QuadRobotCmdLegPoint>(
            "/legs/point", 10, std::bind(&LegNode::leg_point_cb, this, _1));
        leg_angle_sub_ = this->create_subscription<topoquad_msgs::msg::QuadRobotCmdLegAngle>(
            "/legs/angle", 10, std::bind(&LegNode::leg_angle_cb, this, _1));
        dyn_state_sub_ = this->create_subscription<dynamixel_handler::msg::DynamixelState>(
            "/dynamixel/state", 10, std::bind(&LegNode::dyn_state_cb, this, _1));

        // Timer
        timer_ = this->create_wall_timer(1000ms, std::bind(&LegNode::timer_cb, this));
    }

   private:
    void timer_cb() {
        for (auto& leg : {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)}) {
            leg.get().is_updated_ = true;
        }
        BroadcastDynamixelCommand();
        BroadcastLegState("present");
        BroadcastLegState("goal");
    }
    void leg_point_cb(const topoquad_msgs::msg::QuadRobotCmdLegPoint::SharedPtr msg) {
        auto angle_fr = leg_ik(msg->leg_fr, target_leg_fr_.fixed_pose_, 1);
        if (!isnan(angle_fr[0]) && !isnan(angle_fr[1]) && !isnan(angle_fr[2])) target_leg_fr_.SetJointAngles(angle_fr);
        auto angle_fl = leg_ik(msg->leg_fl, target_leg_fl_.fixed_pose_, -1);
        if (!isnan(angle_fl[0]) && !isnan(angle_fl[1]) && !isnan(angle_fl[2])) target_leg_fl_.SetJointAngles(angle_fl);
        auto angle_br = leg_ik(msg->leg_br, target_leg_br_.fixed_pose_, 1);
        if (!isnan(angle_br[0]) && !isnan(angle_br[1]) && !isnan(angle_br[2])) target_leg_br_.SetJointAngles(angle_br);
        auto angle_bl = leg_ik(msg->leg_bl, target_leg_bl_.fixed_pose_, -1);
        if (!isnan(angle_bl[0]) && !isnan(angle_bl[1]) && !isnan(angle_bl[2])) target_leg_bl_.SetJointAngles(angle_bl);
        BroadcastDynamixelCommand();
    }
    void leg_angle_cb(const topoquad_msgs::msg::QuadRobotCmdLegAngle::SharedPtr msg) {
        if (msg->angles_fr.size() > 1) target_leg_fr_.SetJointAngles(msg->angles_fr);
        if (msg->angles_fl.size() > 1) target_leg_fl_.SetJointAngles(msg->angles_fl);
        if (msg->angles_br.size() > 1) target_leg_br_.SetJointAngles(msg->angles_br);
        if (msg->angles_bl.size() > 1) target_leg_bl_.SetJointAngles(msg->angles_bl);
        BroadcastDynamixelCommand();
    }
    void dyn_state_cb(const dynamixel_handler::msg::DynamixelState::SharedPtr msg) {
        for (auto& leg : {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)}) {
            for (int i = 0; i < msg->id_list.size(); i++) {
                if (msg->id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.servo_velocity_ = msg->velocity_deg_s[i] * deg2rad;
                if (msg->id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(msg->current_ma[i]);
                if (msg->id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.servo_velocity_ = msg->velocity_deg_s[i] * deg2rad;
                if (msg->id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(msg->current_ma[i]);
                if (msg->id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.servo_velocity_ = msg->velocity_deg_s[i] * deg2rad;
                if (msg->id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(msg->current_ma[i]);
            }
            leg.get().is_updated_ = true;
            leg.get().updated_time_ = msg->stamp;
        }
        BroadcastLegState("present");

        for (auto& leg : {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)}) {
            for (int i = 0; i < msg->id_list.size(); i++) {
                if (msg->id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(msg->current_ma[i]);
                if (msg->id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(msg->current_ma[i]);
                if (msg->id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(msg->position_deg[i] * deg2rad);
                if (msg->id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(msg->current_ma[i]);
            }
            leg.get().is_updated_ = true;
        }
        BroadcastLegState("goal");
    }

    void BroadcastDynamixelCommand() {
        static vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        static double dt = 0.01;
        static rclcpp::Time prev = this->get_clock()->now();
        rclcpp::Time now = this->get_clock()->now();
        dt = 0.8 * dt + 0.2 * (now - prev).seconds();
        prev = now;  // ほぼ定数になるはずの値なので，平滑化して扱う．

        dynamixel_handler::msg::DynamixelCommandXControlCurrentPosition dyn_msg;
        bool is_diff = false;
        static double ang_hy_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double ang_hp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double ang_kp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; i++) {
            auto& tleg = targets[i];
            auto& pleg = presents[i];
            if (!tleg.get().is_updated_) continue;  // 更新されたときだけpublishする
            auto Dt = (now - pleg.get().updated_time_).seconds();

            auto ang_hy = tleg.get().hip_yaw_.servo_angle_;
            auto ang_hp = tleg.get().hip_pitch_.servo_angle_;
            auto ang_kp = tleg.get().knee_pitch_.servo_angle_;
            auto vel_hy_1 = (ang_hy - ang_hy_pre[i]) / dt;
            auto vel_hp_1 = (ang_hp - ang_hp_pre[i]) / dt;
            auto vel_kp_1 = (ang_kp - ang_kp_pre[i]) / dt;
            auto vel_hy_2 = (ang_hy - (pleg.get().hip_yaw_.servo_angle_ + Dt * pleg.get().hip_yaw_.servo_velocity_)) / dt;
            auto vel_hp_2 = (ang_hp - (pleg.get().hip_pitch_.servo_angle_ + Dt * pleg.get().hip_pitch_.servo_velocity_)) / dt;
            auto vel_kp_2 = (ang_kp - (pleg.get().knee_pitch_.servo_angle_ + Dt * pleg.get().knee_pitch_.servo_velocity_)) / dt;
            auto vel_hy = 0.975 * vel_hy_1 + 0.025 * vel_hy_2;
            auto vel_hp = 0.975 * vel_hp_1 + 0.025 * vel_hp_2;
            auto vel_kp = 0.975 * vel_kp_1 + 0.025 * vel_kp_2;
            // if (tleg.get().knee_pitch_.id_==2) ROS_ERROR("vel: %0.2f, vel_1: %0.2f, vel_2: %0.2f, Dt: %f| p %0.4f  p~ %0.4f pe %0.4f v %0.4f", vel_kp*rad2deg, vel_kp_1*rad2deg, vel_kp_2*rad2deg, Dt, ang_kp*rad2deg,  (pleg.get().knee_pitch_.servo_angle_+Dt*pleg.get().knee_pitch_.servo_velocity_)*rad2deg , pleg.get().knee_pitch_.servo_angle_*rad2deg, pleg.get().knee_pitch_.servo_velocity_*rad2deg);

            dyn_msg.id_list.push_back(tleg.get().hip_yaw_.id_);
            dyn_msg.id_list.push_back(tleg.get().hip_pitch_.id_);
            dyn_msg.id_list.push_back(tleg.get().knee_pitch_.id_);
            dyn_msg.position_deg.push_back((ang_hy + 1.0 * vel_hy_1 * dt) * rad2deg);  // 真の目標値より少し先を目標値にすることで，位置制御特融の加減速の連続を抑制したい．が，効いているのかよくわからん．
            dyn_msg.position_deg.push_back((ang_hp + 1.0 * vel_hy_1 * dt) * rad2deg);
            dyn_msg.position_deg.push_back((ang_kp + 1.0 * vel_hy_1 * dt) * rad2deg);
            dyn_msg.current_ma.push_back(tleg.get().hip_yaw_.servo_current_);
            dyn_msg.current_ma.push_back(tleg.get().hip_pitch_.servo_current_);
            dyn_msg.current_ma.push_back(tleg.get().knee_pitch_.servo_current_);
            dyn_msg.profile_vel_deg_s.push_back(fabs(vel_hy * rad2deg));
            dyn_msg.profile_vel_deg_s.push_back(fabs(vel_hp * rad2deg));
            dyn_msg.profile_vel_deg_s.push_back(fabs(vel_kp * rad2deg));
            dyn_msg.profile_acc_deg_ss.push_back(1000 + 25.0 * fabs(pleg.get().hip_yaw_.servo_velocity_ - vel_hy) * rad2deg / dt);     // todo fabsの中身負号逆じゃない...？
            dyn_msg.profile_acc_deg_ss.push_back(1000 + 25.0 * fabs(pleg.get().hip_pitch_.servo_velocity_ - vel_hp) * rad2deg / dt);   // todo fabsの中身負号逆じゃない...？
            dyn_msg.profile_acc_deg_ss.push_back(1000 + 25.0 * fabs(pleg.get().knee_pitch_.servo_velocity_ - vel_kp) * rad2deg / dt);  // todo fabsの中身負号逆じゃない...？

            ang_hy_pre[i] = ang_hy;
            ang_hp_pre[i] = ang_hp;
            ang_kp_pre[i] = ang_kp;

            tleg.get().is_updated_ = false;
        }
        if (dyn_msg.id_list.size() != 0) dyn_cmd_pub_->publish(dyn_msg);
    }

    void BroadcastLegState(std::string flag) {
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> goals = {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)};
        auto legs = (flag == "present") ? presents : goals;

        topoquad_msgs::msg::QuadRobotStateLeg leg_msg;
        bool is_any_updated_p = false;
        if (legs[0].get().is_updated_) {
            leg_msg.angles_fr = legs[0].get().GetJointAngles();
            leg_msg.torques_fr = legs[0].get().GetJointTorques();
            leg_msg.point_fr = leg_k(leg_msg.angles_fr, legs[0].get().fixed_pose_, 1);
            legs[0].get().is_updated_ = false;
            is_any_updated_p = true;
        }
        if (legs[1].get().is_updated_) {
            leg_msg.angles_fl = legs[1].get().GetJointAngles();
            leg_msg.torques_fl = legs[1].get().GetJointTorques();
            leg_msg.point_fl = leg_k(leg_msg.angles_fl, legs[1].get().fixed_pose_, -1);
            legs[1].get().is_updated_ = false;
            is_any_updated_p = true;
        }
        if (legs[2].get().is_updated_) {
            leg_msg.angles_br = legs[2].get().GetJointAngles();
            leg_msg.torques_br = legs[2].get().GetJointTorques();
            leg_msg.point_br = leg_k(leg_msg.angles_br, legs[2].get().fixed_pose_, 1);
            legs[2].get().is_updated_ = false;
            is_any_updated_p = true;
        }
        if (legs[3].get().is_updated_) {
            leg_msg.angles_bl = legs[3].get().GetJointAngles();
            leg_msg.torques_bl = legs[3].get().GetJointTorques();
            leg_msg.point_bl = leg_k(leg_msg.angles_bl, legs[3].get().fixed_pose_, -1);
            legs[3].get().is_updated_ = false;
            is_any_updated_p = true;
        }

        if (is_any_updated_p) (flag == "present") ? leg_state_p_pub_->publish(leg_msg) : leg_state_g_pub_->publish(leg_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegNode>());
    rclcpp::shutdown();
    return 0;
}