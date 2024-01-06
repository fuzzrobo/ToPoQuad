#include <string>

#include <ros/ros.h>
#include "leg_node.hpp"

#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelCmd.h>
#include <topoquad_master/QuadRobotStateLeg.h>
#include <topoquad_master/QuadRobotCmdLegAngle.h>
#include <topoquad_master/QuadRobotCmdLegPoint.h>

#include <cmath>
using std::isnan;

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

using std::ref;
using std::vector;
using geometry_msgs::Point;
using geometry_msgs::Pose2D;

//https://qiita.com/Ninagawa123/items/4ae058d819de1d5b698a
inline double two_link_ik_t1(const double x, const double y, const double l1, const double l2){
    return atan2(y, x) + acos(((x*x+y*y) + l1*l1 - l2*l2) / (2*l1*sqrt(x*x+y*y)));
}

inline double two_link_ik_t2(const double x, const double y, const double l1, const double l2){
    return - acos(((x*x+y*y) - l1*l1 - l2*l2 ) / (2*l1*l2));
}

inline double normalizeAngle(const double theta){
    return theta - (2*M_PI) * floor((theta + M_PI) / (2*M_PI));
}

class Leg {
    public:
        Leg(): Leg(0, 0, 0) {}
        Leg(double x, double y, double theta): 
            is_updated_(true), 
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
        void SetJointAngles(const std::vector<double>& angles) {
            if (angles.size() != 3) {
                ROS_ERROR("The size of angles must be 3");
                return;
            }
            hip_yaw_.SetJointAngle(angles[0]);
            hip_pitch_.SetJointAngle(angles[1]);
            knee_pitch_.SetJointAngle(angles[2]);
            is_updated_ = true;
        }

        void SetJointTorques(const std::vector<double>& torques) {
            if (torques.size() != 3) {
                ROS_ERROR("The size of torques must be 3");
                return;
            }
            hip_yaw_.SetJointAngle(torques[0]);
            hip_pitch_.SetJointAngle(torques[1]);
            knee_pitch_.SetJointAngle(torques[2]);
            is_updated_ = true;
        }

        bool operator==(const Leg& leg) const {
            return ( fabs(hip_yaw_.joint_angle_ - leg.hip_yaw_.joint_angle_) < 5e-3 
                    && fabs(hip_pitch_.joint_angle_ - leg.hip_pitch_.joint_angle_) < 5e-3 
                    && fabs(knee_pitch_.joint_angle_ - leg.knee_pitch_.joint_angle_) < 5e-3 );
        }
        bool operator!=(const Leg& leg) const {
            return !(*this == leg);
        }
        
        bool is_updated_; // 関節角が更新されたかどうか
        Joint hip_yaw_;
        Joint hip_pitch_;
        Joint knee_pitch_;
        Pose2D fixed_pose_;
};

#define ANGLE_FR (M_PI_4 + M_PI_2 * 0)
#define ANGLE_FL (M_PI_4 + M_PI_2 * 1)
#define ANGLE_BR (M_PI_4 + M_PI_2 * 2)
#define ANGLE_BL (M_PI_4 + M_PI_2 * 3)

#define LENGTH_BASE 0.052
#define LENGTH_HIP_YAW 0.0445
#define LENGTH_HIP_PITCH 0.0445
#define LENGTH_KNEE_PITCH 0.0715

Leg target_leg_FR(LENGTH_BASE*cos(ANGLE_FR), LENGTH_BASE*sin(ANGLE_FR), ANGLE_FR), goal_leg_FR, present_leg_FR; 
Leg target_leg_FL(LENGTH_BASE*cos(ANGLE_FL), LENGTH_BASE*sin(ANGLE_FL), ANGLE_FL), goal_leg_FL, present_leg_FL; 
Leg target_leg_BR(LENGTH_BASE*cos(ANGLE_BR), LENGTH_BASE*sin(ANGLE_BR), ANGLE_BR), goal_leg_BR, present_leg_BR; 
Leg target_leg_BL(LENGTH_BASE*cos(ANGLE_BL), LENGTH_BASE*sin(ANGLE_BL), ANGLE_BL), goal_leg_BL, present_leg_BL; 


vector<double> leg_ik(const Point& tp, const Pose2D& fp){
    double dx = tp.x - fp.x;
    double dy = tp.y - fp.y;
    double x = hypot(dx, dy) - LENGTH_HIP_YAW;
    vector<double> angles(3);
    angles[0] = normalizeAngle(atan2(dy, dx) - fp.theta);
    angles[1] = -two_link_ik_t1(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    angles[2] = -two_link_ik_t2(x, tp.z, LENGTH_HIP_PITCH, LENGTH_KNEE_PITCH);
    return angles;
}

void CallBackOfLegAngle(const topoquad_master::QuadRobotCmdLegAngle::ConstPtr& msg) {
    if(msg->angles_FR.size() > 1) target_leg_FR.SetJointAngles(msg->angles_FR);
    if(msg->angles_FL.size() > 1) target_leg_FL.SetJointAngles(msg->angles_FL);
    if(msg->angles_BR.size() > 1) target_leg_BR.SetJointAngles(msg->angles_BR);
    if(msg->angles_BL.size() > 1) target_leg_BL.SetJointAngles(msg->angles_BL);
}

void CallBackOfLegPoint(const topoquad_master::QuadRobotCmdLegPoint::ConstPtr& msg){
    auto angle_FR = leg_ik(msg->leg_FR, target_leg_FR.fixed_pose_);
    if (!isnan(angle_FR[0]) && !isnan(angle_FR[1]) && !isnan(angle_FR[2])) target_leg_FR.SetJointAngles(angle_FR);
    auto angle_FL = leg_ik(msg->leg_FL, target_leg_FL.fixed_pose_);
    if (!isnan(angle_FL[0]) && !isnan(angle_FL[1]) && !isnan(angle_FL[2])) target_leg_FL.SetJointAngles(angle_FL);
    auto angle_BR = leg_ik(msg->leg_BR, target_leg_BR.fixed_pose_);
    if (!isnan(angle_BR[0]) && !isnan(angle_BR[1]) && !isnan(angle_BR[2])) target_leg_BR.SetJointAngles(angle_BR);
    auto angle_BL = leg_ik(msg->leg_BL, target_leg_BL.fixed_pose_);
    if (!isnan(angle_BL[0]) && !isnan(angle_BL[1]) && !isnan(angle_BL[2])) target_leg_BL.SetJointAngles(angle_BL);
}

void CallBackOfDynamixelState(const dynamixel_handler::DynamixelState::ConstPtr& msg) {
    for ( auto& leg : {ref(present_leg_FR), ref(present_leg_FL), ref(present_leg_BR), ref(present_leg_BL)}) {
        for (int i=0; i<msg->ids.size(); i++) {
            if(msg->ids[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(msg->present_angles[i]);
            if(msg->ids[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(msg->present_currents[i]);
            if(msg->ids[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(msg->present_angles[i]);
            if(msg->ids[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(msg->present_currents[i]);
            if(msg->ids[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(msg->present_angles[i]);
            if(msg->ids[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(msg->present_currents[i]);
        }
        leg.get().is_updated_ = true;
    }
    for ( auto& leg : {ref(goal_leg_FR), ref(goal_leg_FL), ref(goal_leg_BR), ref(goal_leg_BL)}) {
        for (int i=0; i<msg->ids.size(); i++) {
            if(msg->ids[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoAngle(msg->goal_angles[i]);
            if(msg->ids[i] == leg.get().hip_yaw_.id_) leg.get().hip_yaw_.SetServoCurrent(msg->goal_currents[i]);
            if(msg->ids[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoAngle(msg->goal_angles[i]);
            if(msg->ids[i] == leg.get().hip_pitch_.id_) leg.get().hip_pitch_.SetServoCurrent(msg->goal_currents[i]);
            if(msg->ids[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoAngle(msg->goal_angles[i]);
            if(msg->ids[i] == leg.get().knee_pitch_.id_) leg.get().knee_pitch_.SetServoCurrent(msg->goal_currents[i]);
        }
        leg.get().is_updated_ = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "leg_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::vector<int> ids_BR, ids_FR, ids_FL, ids_BL;
    if (!nh_p.getParam("BR_leg_dynamixel_ID",   ids_BR)) ids_BR = { 4, 3, 2};
    if (!nh_p.getParam("FR_leg_dynamixel_ID",   ids_FR)) ids_FR = {14,13,12};
    if (!nh_p.getParam("FL_leg_dynamixel_ID",   ids_FL)) ids_FL = {24,23,22};
    if (!nh_p.getParam("BL_leg_dynamixel_ID",   ids_BL)) ids_BL = {34,33,32};

    target_leg_BR.initialize( Joint{ ids_BR[0], -1.0,   0.0 /*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_BR[1], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_BR[2], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/}  );
    target_leg_FR.initialize( Joint{ ids_FR[0], -1.0,   0.0 /*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_FR[1], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_FR[2], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/}  ); 
    target_leg_FL.initialize( Joint{ ids_FL[0], +1.0,   0.0 /*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_FL[1], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_FL[2], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/}  );
    target_leg_BL.initialize( Joint{ ids_BL[0], +1.0,   0.0 /*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_BL[1], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/},
                              Joint{ ids_BL[2], +1.0, M_PI/4/*rad*/, +0.92/800/*Nm/mA*/, 0.2/*Nm*/}  );

    present_leg_FR = target_leg_FR;
    present_leg_FL = target_leg_FL;
    present_leg_BR = target_leg_BR;
    present_leg_BL = target_leg_BL;
    
    goal_leg_FR = target_leg_FR;
    goal_leg_FL = target_leg_FL;
    goal_leg_BR = target_leg_BR;
    goal_leg_BL = target_leg_BL;

    ros::Subscriber sub_leg_point = nh.subscribe("/legs/point", 10, CallBackOfLegPoint);
    ros::Subscriber sub_leg_angle = nh.subscribe("/legs/angle", 10, CallBackOfLegAngle);
    ros::Publisher  pub_dyn_cmd   = nh.advertise<dynamixel_handler::DynamixelCmd>("/dynamixel/cmd", 10);
    
    ros::Subscriber sub_dyn_state   = nh.subscribe("/dynamixel/state",   10, CallBackOfDynamixelState);  // サーボの角度をsubscribe
    ros::Publisher  pub_leg_state_p = nh.advertise<topoquad_master::QuadRobotStateLeg>("/legs/state/present", 10); // サーボの角度を関節の状態に変換してpublish
    ros::Publisher  pub_leg_state_g = nh.advertise<topoquad_master::QuadRobotStateLeg>("/legs/state/goal", 10);    // サーボの角度を関節の状態に変換してpublish

    vector<std::reference_wrapper<Leg>> targets = {ref(target_leg_FR), ref(target_leg_FL), ref(target_leg_BR), ref(target_leg_BL)};
    vector<std::reference_wrapper<Leg>> goals =   {ref(goal_leg_FR  ), ref(goal_leg_FL  ), ref(goal_leg_BR  ), ref(goal_leg_BL  )};
    ros::Rate rate(200);
    while(ros::ok()) {
        ros::spinOnce();
        dynamixel_handler::DynamixelCmd dyn_msg;
        dyn_msg.command = "write";
        bool is_diff = false;
        for ( int i=0; i<4; i++) {
            auto& tleg = targets[i];
            auto& gleg = goals[i];
            if( !(tleg.get().is_updated_ || gleg.get() != tleg.get()) ) continue; //更新されたときだけpublishする
            dyn_msg.ids.push_back(tleg.get().hip_yaw_.id_);
            dyn_msg.ids.push_back(tleg.get().hip_pitch_.id_);
            dyn_msg.ids.push_back(tleg.get().knee_pitch_.id_);
            dyn_msg.goal_angles.push_back(tleg.get().hip_yaw_.servo_angle_);
            dyn_msg.goal_angles.push_back(tleg.get().hip_pitch_.servo_angle_);
            dyn_msg.goal_angles.push_back(tleg.get().knee_pitch_.servo_angle_);
            dyn_msg.goal_currents.push_back(tleg.get().hip_yaw_.servo_current_);
            dyn_msg.goal_currents.push_back(tleg.get().hip_pitch_.servo_current_);
            dyn_msg.goal_currents.push_back(tleg.get().knee_pitch_.servo_current_);
            tleg.get().is_updated_ = false;
        }
        if (dyn_msg.ids.size() != 0) pub_dyn_cmd.publish(dyn_msg);

        topoquad_master::QuadRobotStateLeg leg_msg_p;
        bool is_any_updated_p = false;
        if (present_leg_FR.is_updated_) {
            leg_msg_p.angles_FR.push_back(present_leg_FR.hip_yaw_.joint_angle_);
            leg_msg_p.angles_FR.push_back(present_leg_FR.hip_pitch_.joint_angle_);
            leg_msg_p.angles_FR.push_back(present_leg_FR.knee_pitch_.joint_angle_);
            leg_msg_p.torques_FR.push_back(present_leg_FR.hip_yaw_.joint_torque_);
            leg_msg_p.torques_FR.push_back(present_leg_FR.hip_pitch_.joint_torque_);
            leg_msg_p.torques_FR.push_back(present_leg_FR.knee_pitch_.joint_torque_);
            present_leg_FR.is_updated_ = false;
            is_any_updated_p = true;
        }
        if (present_leg_FL.is_updated_) {
            leg_msg_p.angles_FL.push_back(present_leg_FL.hip_yaw_.joint_angle_);
            leg_msg_p.angles_FL.push_back(present_leg_FL.hip_pitch_.joint_angle_);
            leg_msg_p.angles_FL.push_back(present_leg_FL.knee_pitch_.joint_angle_);
            leg_msg_p.torques_FL.push_back(present_leg_FL.hip_yaw_.joint_torque_);
            leg_msg_p.torques_FL.push_back(present_leg_FL.hip_pitch_.joint_torque_);
            leg_msg_p.torques_FL.push_back(present_leg_FL.knee_pitch_.joint_torque_);
            present_leg_FL.is_updated_ = false;
            is_any_updated_p = true;
        }
        if (present_leg_BR.is_updated_) {
            leg_msg_p.angles_BR.push_back(present_leg_BR.hip_yaw_.joint_angle_);
            leg_msg_p.angles_BR.push_back(present_leg_BR.hip_pitch_.joint_angle_);
            leg_msg_p.angles_BR.push_back(present_leg_BR.knee_pitch_.joint_angle_);
            leg_msg_p.torques_BR.push_back(present_leg_BR.hip_yaw_.joint_torque_);
            leg_msg_p.torques_BR.push_back(present_leg_BR.hip_pitch_.joint_torque_);
            leg_msg_p.torques_BR.push_back(present_leg_BR.knee_pitch_.joint_torque_);
            present_leg_BR.is_updated_ = false;
            is_any_updated_p = true;
        }
        if (present_leg_BL.is_updated_) {
            leg_msg_p.angles_BL.push_back(present_leg_BL.hip_yaw_.joint_angle_);
            leg_msg_p.angles_BL.push_back(present_leg_BL.hip_pitch_.joint_angle_);
            leg_msg_p.angles_BL.push_back(present_leg_BL.knee_pitch_.joint_angle_);
            leg_msg_p.torques_BL.push_back(present_leg_BL.hip_yaw_.joint_torque_);
            leg_msg_p.torques_BL.push_back(present_leg_BL.hip_pitch_.joint_torque_);
            leg_msg_p.torques_BL.push_back(present_leg_BL.knee_pitch_.joint_torque_);
            present_leg_BL.is_updated_ = false;
            is_any_updated_p = true;
        }

        if (is_any_updated_p) pub_leg_state_p.publish(leg_msg_p);

        topoquad_master::QuadRobotStateLeg leg_msg_g;
        bool is_any_updated_g = false;
        if (goal_leg_FR.is_updated_) {
            leg_msg_g.angles_FR.push_back(goal_leg_FR.hip_yaw_.joint_angle_);
            leg_msg_g.angles_FR.push_back(goal_leg_FR.hip_pitch_.joint_angle_);
            leg_msg_g.angles_FR.push_back(goal_leg_FR.knee_pitch_.joint_angle_);
            leg_msg_g.torques_FR.push_back(goal_leg_FR.hip_yaw_.joint_torque_);
            leg_msg_g.torques_FR.push_back(goal_leg_FR.hip_pitch_.joint_torque_);
            leg_msg_g.torques_FR.push_back(goal_leg_FR.knee_pitch_.joint_torque_);
            goal_leg_FR.is_updated_ = false;
            is_any_updated_g = true;
        }
        if (goal_leg_FL.is_updated_) {
            leg_msg_g.angles_FL.push_back(goal_leg_FL.hip_yaw_.joint_angle_);
            leg_msg_g.angles_FL.push_back(goal_leg_FL.hip_pitch_.joint_angle_);
            leg_msg_g.angles_FL.push_back(goal_leg_FL.knee_pitch_.joint_angle_);
            leg_msg_g.torques_FL.push_back(goal_leg_FL.hip_yaw_.joint_torque_);
            leg_msg_g.torques_FL.push_back(goal_leg_FL.hip_pitch_.joint_torque_);
            leg_msg_g.torques_FL.push_back(goal_leg_FL.knee_pitch_.joint_torque_);
            goal_leg_FL.is_updated_ = false;
            is_any_updated_g = true;
        }
        if (goal_leg_BR.is_updated_) {
            leg_msg_g.angles_BR.push_back(goal_leg_BR.hip_yaw_.joint_angle_);
            leg_msg_g.angles_BR.push_back(goal_leg_BR.hip_pitch_.joint_angle_);
            leg_msg_g.angles_BR.push_back(goal_leg_BR.knee_pitch_.joint_angle_);
            leg_msg_g.torques_BR.push_back(goal_leg_BR.hip_yaw_.joint_torque_);
            leg_msg_g.torques_BR.push_back(goal_leg_BR.hip_pitch_.joint_torque_);
            leg_msg_g.torques_BR.push_back(goal_leg_BR.knee_pitch_.joint_torque_);
            goal_leg_BR.is_updated_ = false;
            is_any_updated_g = true;
        }
        if (goal_leg_BL.is_updated_) {
            leg_msg_g.angles_BL.push_back(goal_leg_BL.hip_yaw_.joint_angle_);
            leg_msg_g.angles_BL.push_back(goal_leg_BL.hip_pitch_.joint_angle_);
            leg_msg_g.angles_BL.push_back(goal_leg_BL.knee_pitch_.joint_angle_);
            leg_msg_g.torques_BL.push_back(goal_leg_BL.hip_yaw_.joint_torque_);
            leg_msg_g.torques_BL.push_back(goal_leg_BL.hip_pitch_.joint_torque_);
            leg_msg_g.torques_BL.push_back(goal_leg_BL.knee_pitch_.joint_torque_);
            goal_leg_BL.is_updated_ = false;
            is_any_updated_g = true;
        }

        if (is_any_updated_g) pub_leg_state_g.publish(leg_msg_g);
        rate.sleep();
    }
}