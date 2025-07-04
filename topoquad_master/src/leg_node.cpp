#include <topoquad_master/leg_node.hpp>

using std::map;
using std::string;

class LegNode : public rclcpp::Node {
    rclcpp::Publisher<dynamixel_handler::msg::DxlCommandsX>::SharedPtr dyn_cmd_pub_;
    rclcpp::Publisher<topoquad_msgs::msg::QuadRobotStateLeg>::SharedPtr leg_state_p_pub_, leg_state_g_pub_, leg_state_t_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::Subscription<topoquad_msgs::msg::QuadRobotCmdLegPoint>::SharedPtr leg_point_sub_;
    rclcpp::Subscription<topoquad_msgs::msg::QuadRobotCmdLegAngle>::SharedPtr leg_angle_sub_;
    rclcpp::Subscription<dynamixel_handler::msg::DxlStates>::SharedPtr dyn_state_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time prev_cmd_time_;

    Leg target_leg_fr_, goal_leg_fr_, present_leg_fr_;
    Leg target_leg_fl_, goal_leg_fl_, present_leg_fl_;
    Leg target_leg_br_, goal_leg_br_, present_leg_br_;
    Leg target_leg_bl_, goal_leg_bl_, present_leg_bl_;

    enum ControlMode{
        MODE_VELOCITY,
        MODE_VELOCITY_SAFE,
        MODE_POSITION
    };

    map<string, double> gain;
    double overshoot_;
    double deadband_;
    ControlMode control_mode_;

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

        this->declare_parameter<string>("control_mode", "position");
        auto mode_str = this->get_parameter("control_mode").as_string();
             if (mode_str == "velocity"         || mode_str == "vel"   ) control_mode_ = MODE_VELOCITY;
        else if (mode_str == "position"         || mode_str == "pos"   ) control_mode_ = MODE_POSITION;
        else if (mode_str == "velocity_current" || mode_str == "velcur") control_mode_ = MODE_VELOCITY_SAFE;
        else RCLCPP_ERROR(get_logger(), "Invalid control mode [velocity, position, velocity_current]");

        this->declare_parameter<double>("outer_gain.p", 5.0);
        this->declare_parameter<double>("outer_gain.i", 0.0);
        this->declare_parameter<double>("outer_gain.d", 0.0);
        gain["p"] = this->get_parameter("outer_gain.p").as_double();
        gain["i"] = this->get_parameter("outer_gain.i").as_double();
        gain["d"] = this->get_parameter("outer_gain.d").as_double();

        this->declare_parameter<double>("dynamixel_gain.vel_p", 50.0);
        this->declare_parameter<double>("dynamixel_gain.pos_p", 800.0);
        gain["dxl_vel_p"] = this->get_parameter("dynamixel_gain.vel_p").as_double();
        gain["dxl_pos_p"] = this->get_parameter("dynamixel_gain.pos_p").as_double();

        this->declare_parameter<double>("overshoot", 50.0);
        overshoot_ = this->get_parameter("overshoot").as_double()*deg2rad;
        this->declare_parameter<double>("deadband", 1.0);
        deadband_ = this->get_parameter("deadband").as_double()*deg2rad;

        present_leg_fr_ = target_leg_fr_;
        present_leg_fl_ = target_leg_fl_;
        present_leg_br_ = target_leg_br_;
        present_leg_bl_ = target_leg_bl_;

        goal_leg_fr_ = target_leg_fr_;
        goal_leg_fl_ = target_leg_fl_;
        goal_leg_br_ = target_leg_br_;
        goal_leg_bl_ = target_leg_bl_;

        // Publishers
        dyn_cmd_pub_ = this->create_publisher<dynamixel_handler::msg::DxlCommandsX>("dynamixel/commands/x", 10);
        leg_state_p_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateLeg>("legs/state/present", 10);
        leg_state_g_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateLeg>("legs/state/goal", 10);
        leg_state_t_pub_ = this->create_publisher<topoquad_msgs::msg::QuadRobotStateLeg>("legs/state/target", 10);
        // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("jointstate/legs", 10);

        // Subscribers
        leg_point_sub_ = this->create_subscription<topoquad_msgs::msg::QuadRobotCmdLegPoint>(
            "legs/point", 10, std::bind(&LegNode::leg_point_cb, this, _1));
        leg_angle_sub_ = this->create_subscription<topoquad_msgs::msg::QuadRobotCmdLegAngle>(
            "legs/angle", 10, std::bind(&LegNode::leg_angle_cb, this, _1));
        dyn_state_sub_ = this->create_subscription<dynamixel_handler::msg::DxlStates>(
            "dynamixel/states", 10, std::bind(&LegNode::dyn_state_cb, this, _1));

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(0.05s, std::bind(&LegNode::main_loop, this));
    }
    
    private:
    
    void main_loop(){
        auto now = this->get_clock()->now();
        if (now.seconds() - prev_cmd_time_.seconds() < 0.2) return;
        dynamixel_handler::msg::DxlCommandsX dyn_msg;
        for (auto& leg : {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)}) {
            if (true){
                dyn_msg.current_base_position_control.id_list.push_back( leg.get().hip_yaw_.id_);  
                dyn_msg.current_base_position_control.id_list.push_back( leg.get().hip_pitch_.id_);
                dyn_msg.current_base_position_control.id_list.push_back( leg.get().knee_pitch_.id_);
                dyn_msg.current_base_position_control.position_deg.push_back(  0.0 );  
                dyn_msg.current_base_position_control.position_deg.push_back( 45.0 );
                dyn_msg.current_base_position_control.position_deg.push_back( 45.0 );
                dyn_msg.current_base_position_control.profile_vel_deg_s.push_back( 50.0 );
                dyn_msg.current_base_position_control.profile_vel_deg_s.push_back( 50.0 );
                dyn_msg.current_base_position_control.profile_vel_deg_s.push_back( 50.0 );
            }else{
                dyn_msg.velocity_control.id_list.push_back( leg.get().hip_yaw_.id_);  
                dyn_msg.velocity_control.id_list.push_back( leg.get().hip_pitch_.id_);
                dyn_msg.velocity_control.id_list.push_back( leg.get().knee_pitch_.id_);
                dyn_msg.velocity_control.velocity_deg_s.push_back(0.0);
                dyn_msg.velocity_control.velocity_deg_s.push_back(0.0);
                dyn_msg.velocity_control.velocity_deg_s.push_back(0.0);
            }
            dyn_msg.status.id_list.push_back( leg.get().hip_yaw_.id_);  
            dyn_msg.status.id_list.push_back( leg.get().hip_pitch_.id_);
            dyn_msg.status.id_list.push_back( leg.get().knee_pitch_.id_);
            dyn_msg.status.error.push_back( false );  
            dyn_msg.status.error.push_back( false );
            dyn_msg.status.error.push_back( false );
        }
        dyn_cmd_pub_->publish(dyn_msg);
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
        // sensor_msgs::msg::JointState joint_msg;
        switch (control_mode_) {
            case MODE_VELOCITY:      BroadcastDynamixelCommand_VelocityBase(); break;
            case MODE_VELOCITY_SAFE: BroadcastDynamixelCommand_VelocityPosBase();   break;
            case MODE_POSITION: 
            default:                 BroadcastDynamixelCommand_PositionBase(); break;
        }
        BroadcastLegState("target");
    }
    void leg_angle_cb(const topoquad_msgs::msg::QuadRobotCmdLegAngle::SharedPtr msg) {
        if (msg->angles_fr.size() > 1) target_leg_fr_.SetJointAngles(msg->angles_fr);
        if (msg->angles_fl.size() > 1) target_leg_fl_.SetJointAngles(msg->angles_fl);
        if (msg->angles_br.size() > 1) target_leg_br_.SetJointAngles(msg->angles_br);
        if (msg->angles_bl.size() > 1) target_leg_bl_.SetJointAngles(msg->angles_bl);
        switch (control_mode_) {
            case MODE_VELOCITY:      BroadcastDynamixelCommand_VelocityBase(); break;
            case MODE_VELOCITY_SAFE: BroadcastDynamixelCommand_VelocityPosBase();   break;
            case MODE_POSITION: 
            default:                 BroadcastDynamixelCommand_PositionBase(); break;
        }
        BroadcastLegState("target");

    }
    void dyn_state_cb(const dynamixel_handler::msg::DxlStates::SharedPtr msg) {
        for (auto& leg : {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)}) {
            if (auto& p=msg->present; !p.id_list.empty()) 
                for (size_t i = 0; i < p.id_list.size(); i++) {
                    if (p.id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.servo_velocity_ = p.velocity_deg_s[i] * deg2rad;
                    if (p.id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(p.position_deg[i] * deg2rad);
                    if (p.id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(p.current_ma[i]);
                    if (p.id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.servo_velocity_ = p.velocity_deg_s[i] * deg2rad;
                    if (p.id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(p.position_deg[i] * deg2rad);
                    if (p.id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(p.current_ma[i]);
                    if (p.id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.servo_velocity_ = p.velocity_deg_s[i] * deg2rad;
                    if (p.id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(p.position_deg[i] * deg2rad);
                    if (p.id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(p.current_ma[i]);
            }
            leg.get().is_updated_ = true;
            leg.get().updated_time_ = msg->stamp.sec + msg->stamp.nanosec * 10e-9;
            leg.get().updated_time_ = this->get_clock()->now().seconds();
        }
        BroadcastLegState("present");

        for (auto& leg : {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)}) {
            if (auto& g=msg->goal; !g.id_list.empty()) 
                for (size_t i = 0; i < g.id_list.size(); i++) {
                    if (g.id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(g.position_deg[i] * deg2rad);
                    if (g.id_list[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(g.current_ma[i]);
                    if (g.id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(g.position_deg[i] * deg2rad);
                    if (g.id_list[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(g.current_ma[i]);
                    if (g.id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(g.position_deg[i] * deg2rad);
                    if (g.id_list[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(g.current_ma[i]);
            }
            leg.get().is_updated_ = true;
        }
        BroadcastLegState("goal");

        // gain 調整
        if (auto& g=msg->gain; !g.id_list.empty()){
            if (g.velocity_p_gain_pulse[0] == gain["dxl_vel_p"] && 
                g.position_p_gain_pulse[0] == gain["dxl_pos_p"] ) return;
            dynamixel_handler::msg::DxlCommandsX dyn_msg;
            for (auto& leg : {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)}) {
                dyn_msg.gain.id_list.push_back( leg.get().hip_yaw_.id_);  
                dyn_msg.gain.id_list.push_back( leg.get().hip_pitch_.id_);
                dyn_msg.gain.id_list.push_back( leg.get().knee_pitch_.id_);
                dyn_msg.gain.velocity_p_gain_pulse.push_back(gain["dxl_vel_p"]);
                dyn_msg.gain.velocity_p_gain_pulse.push_back(gain["dxl_vel_p"]);
                dyn_msg.gain.velocity_p_gain_pulse.push_back(gain["dxl_vel_p"]);
                dyn_msg.gain.position_p_gain_pulse.push_back(gain["dxl_pos_p"]);
                dyn_msg.gain.position_p_gain_pulse.push_back(gain["dxl_pos_p"]);
                dyn_msg.gain.position_p_gain_pulse.push_back(gain["dxl_pos_p"]);
            }
            dyn_cmd_pub_->publish(dyn_msg);
        }
    }

    void BroadcastDynamixelCommand_VelocityPosBase() {
        static vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        auto now = this->get_clock()->now();

        dynamixel_handler::msg::DxlCommandsX dyn_msg;
        auto& ctrl_msg = dyn_msg.current_base_position_control;
        static double diff_hy_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double diff_hp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double diff_kp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; i++) {
            auto& tleg = targets[i].get();
            auto& pleg = presents[i].get();
            auto Dt = (now.seconds() - pleg.updated_time_);

            auto tar_ang_hy = tleg.hip_yaw_.servo_angle_;
            auto tar_ang_hp = tleg.hip_pitch_.servo_angle_;
            auto tar_ang_kp = tleg.knee_pitch_.servo_angle_;
            auto now_ang_hy = pleg.hip_yaw_.servo_angle_+ Dt * pleg.hip_yaw_.servo_velocity_;
            auto now_ang_hp = pleg.hip_pitch_.servo_angle_+ Dt * pleg.hip_pitch_.servo_velocity_;
            auto now_ang_kp = pleg.knee_pitch_.servo_angle_+ Dt * pleg.knee_pitch_.servo_velocity_;
            auto diff_hy = tar_ang_hy - now_ang_hy;
            auto diff_hp = tar_ang_hp - now_ang_hp;
            auto diff_kp = tar_ang_kp - now_ang_kp;
            auto vel_hy = fabs(diff_hy)<deadband_ ? 0.0001 : gain["p"] * diff_hy + gain["d"] * (diff_hy-diff_hy_pre[i]);
            auto vel_hp = fabs(diff_hp)<deadband_ ? 0.0001 : gain["p"] * diff_hp + gain["d"] * (diff_hp-diff_hp_pre[i]);
            auto vel_kp = fabs(diff_kp)<deadband_ ? 0.0001 : gain["p"] * diff_kp + gain["d"] * (diff_kp-diff_kp_pre[i]);
            auto pos_hy = (0<diff_hy) ? tar_ang_hy+overshoot_ : tar_ang_hy-overshoot_;
            auto pos_hp = (0<diff_hp) ? tar_ang_hp+overshoot_ : tar_ang_hp-overshoot_;
            auto pos_kp = (0<diff_kp) ? tar_ang_kp+overshoot_ : tar_ang_kp-overshoot_;

            ctrl_msg.id_list.push_back(tleg.hip_yaw_.id_);
            ctrl_msg.id_list.push_back(tleg.hip_pitch_.id_);
            ctrl_msg.id_list.push_back(tleg.knee_pitch_.id_);

            ctrl_msg.position_deg.push_back(pos_hy * rad2deg);
            ctrl_msg.position_deg.push_back(pos_hp * rad2deg);
            ctrl_msg.position_deg.push_back(pos_kp * rad2deg);
            ctrl_msg.profile_vel_deg_s.push_back(fabs(vel_hy * rad2deg));
            ctrl_msg.profile_vel_deg_s.push_back(fabs(vel_hp * rad2deg));
            ctrl_msg.profile_vel_deg_s.push_back(fabs(vel_kp * rad2deg));
            ctrl_msg.current_ma.push_back(tleg.hip_yaw_.servo_current_);
            ctrl_msg.current_ma.push_back(tleg.hip_pitch_.servo_current_);
            ctrl_msg.current_ma.push_back(tleg.knee_pitch_.servo_current_);
            ctrl_msg.profile_acc_deg_ss.push_back(0);
            ctrl_msg.profile_acc_deg_ss.push_back(0);
            ctrl_msg.profile_acc_deg_ss.push_back(0);

            diff_hy_pre[i] = diff_hy;
            diff_hp_pre[i] = diff_hp;
            diff_kp_pre[i] = diff_kp;
        }
        
        prev_cmd_time_ = now;
        dyn_cmd_pub_->publish(dyn_msg);
    }

    void BroadcastDynamixelCommand_VelocityBase(){
        static vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        auto now = this->get_clock()->now();

        dynamixel_handler::msg::DxlCommandsX dyn_msg;
        auto& ctrl_msg = dyn_msg.velocity_control;
        static double diff_hy_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double diff_hp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double diff_kp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; i++) {
            auto& tleg = targets[i].get();
            auto& pleg = presents[i].get();
            auto Dt = (now.seconds() - pleg.updated_time_);

            auto tar_ang_hy = tleg.hip_yaw_.servo_angle_;
            auto tar_ang_hp = tleg.hip_pitch_.servo_angle_;
            auto tar_ang_kp = tleg.knee_pitch_.servo_angle_;
            auto now_ang_hy = pleg.hip_yaw_.servo_angle_+ Dt * pleg.hip_yaw_.servo_velocity_;
            auto now_ang_hp = pleg.hip_pitch_.servo_angle_+ Dt * pleg.hip_pitch_.servo_velocity_;
            auto now_ang_kp = pleg.knee_pitch_.servo_angle_+ Dt * pleg.knee_pitch_.servo_velocity_;
            auto diff_hy = tar_ang_hy - now_ang_hy;
            auto diff_hp = tar_ang_hp - now_ang_hp;
            auto diff_kp = tar_ang_kp - now_ang_kp;
            auto vel_hy = fabs(diff_hy)<deadband_ ? 0.0 : gain["p"] * diff_hy + gain["d"] * (diff_hy-diff_hy_pre[i]);
            auto vel_hp = fabs(diff_hp)<deadband_ ? 0.0 : gain["p"] * diff_hp + gain["d"] * (diff_hp-diff_hp_pre[i]);
            auto vel_kp = fabs(diff_kp)<deadband_ ? 0.0 : gain["p"] * diff_kp + gain["d"] * (diff_kp-diff_kp_pre[i]);

            ctrl_msg.id_list.push_back(tleg.hip_yaw_.id_);
            ctrl_msg.id_list.push_back(tleg.hip_pitch_.id_);
            ctrl_msg.id_list.push_back(tleg.knee_pitch_.id_);
            ctrl_msg.velocity_deg_s.push_back((vel_hy * rad2deg));
            ctrl_msg.velocity_deg_s.push_back((vel_hp * rad2deg));
            ctrl_msg.velocity_deg_s.push_back((vel_kp * rad2deg));
            ctrl_msg.profile_acc_deg_ss.push_back(0);
            ctrl_msg.profile_acc_deg_ss.push_back(0);
            ctrl_msg.profile_acc_deg_ss.push_back(0);

            diff_hy_pre[i] = diff_hy;
            diff_hp_pre[i] = diff_hp;
            diff_kp_pre[i] = diff_kp;
        }
        
        prev_cmd_time_ = now;
        dyn_cmd_pub_->publish(dyn_msg);
    }

    void BroadcastDynamixelCommand_PositionBase(){
        static vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        static auto prev = this->get_clock()->now();
        auto now = this->get_clock()->now();
        static double dt = 0.01;
        dt = 0.8 * dt + 0.2 * (now.seconds() - prev.seconds());
        prev = now;  // ほぼ定数になるはずの値なので，平滑化して扱う．

        dynamixel_handler::msg::DxlCommandsX dyn_msg;
        auto& ctrl_msg = dyn_msg.current_base_position_control;
        static double ang_hy_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double ang_hp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        static double ang_kp_pre[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; i++) {
            auto& tleg = targets[i].get();
            auto& pleg = presents[i].get();

            auto tar_ang_hy_pre = ang_hy_pre[i];
            auto tar_ang_hp_pre = ang_hp_pre[i];
            auto tar_ang_kp_pre = ang_kp_pre[i];
            auto tar_ang_hy = ang_hy_pre[i] = tleg.hip_yaw_.servo_angle_;
            auto tar_ang_hp = ang_hp_pre[i] = tleg.hip_pitch_.servo_angle_;
            auto tar_ang_kp = ang_kp_pre[i] = tleg.knee_pitch_.servo_angle_;
            auto now_ang_hy = pleg.hip_yaw_.servo_angle_;
            auto now_ang_hp = pleg.hip_pitch_.servo_angle_;
            auto now_ang_kp = pleg.knee_pitch_.servo_angle_;

            ctrl_msg.id_list.push_back(tleg.hip_yaw_.id_);
            ctrl_msg.id_list.push_back(tleg.hip_pitch_.id_);
            ctrl_msg.id_list.push_back(tleg.knee_pitch_.id_);
            ctrl_msg.position_deg.push_back(tar_ang_hy * rad2deg);
            ctrl_msg.position_deg.push_back(tar_ang_hp * rad2deg);
            ctrl_msg.position_deg.push_back(tar_ang_kp * rad2deg);
            ctrl_msg.current_ma.push_back(tleg.hip_yaw_.servo_current_);
            ctrl_msg.current_ma.push_back(tleg.hip_pitch_.servo_current_);
            ctrl_msg.current_ma.push_back(tleg.knee_pitch_.servo_current_);
            if (i==0){
                RCLCPP_INFO(get_logger(), "%f, %f, %f, %f", dt, tar_ang_hy_pre, tar_ang_hy, now_ang_hy);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hy- now_ang_hy)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hp- now_ang_hp)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_kp- now_ang_kp)/dt* rad2deg);
                ctrl_msg.profile_acc_deg_ss.push_back(5000);
                ctrl_msg.profile_acc_deg_ss.push_back(5000);
                ctrl_msg.profile_acc_deg_ss.push_back(5000);
            } else if (i==1){
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hy- now_ang_hy)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hp- now_ang_hp)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_kp- now_ang_kp)/dt* rad2deg);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
            } else if (i==2){
                ctrl_msg.profile_vel_deg_s.push_back(0);
                ctrl_msg.profile_vel_deg_s.push_back(0);
                ctrl_msg.profile_vel_deg_s.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
            } else {
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hy- tar_ang_hy_pre)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_hp- tar_ang_hp_pre)/dt* rad2deg);
                ctrl_msg.profile_vel_deg_s.push_back((tar_ang_kp- tar_ang_kp_pre)/dt* rad2deg);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
                ctrl_msg.profile_acc_deg_ss.push_back(0);
            }
        }

        prev_cmd_time_ = now;
        dyn_cmd_pub_->publish(dyn_msg);
    }


    void BroadcastLegState(std::string flag) {
        static vector<std::reference_wrapper<Leg>> presents = {ref(present_leg_fr_), ref(present_leg_fl_), ref(present_leg_br_), ref(present_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_fr_), ref(target_leg_fl_), ref(target_leg_br_), ref(target_leg_bl_)};
        static vector<std::reference_wrapper<Leg>> goals = {ref(goal_leg_fr_), ref(goal_leg_fl_), ref(goal_leg_br_), ref(goal_leg_bl_)};
        auto legs = (flag == "present") ? presents : (flag == "target") ? targets : goals;

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

        if (is_any_updated_p) 
            (flag == "present") ? leg_state_p_pub_->publish(leg_msg) :
            (flag == "target" ) ? leg_state_t_pub_->publish(leg_msg) : leg_state_g_pub_->publish(leg_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegNode>());
    rclcpp::shutdown();
    return 0;
}