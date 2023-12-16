#include <string>
#include <Eigen/Core>
using Eigen::Vector3d;

#include <ros/ros.h>
#include "neck_node.hpp"

#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelCmd.h>
#include <topoquad_master/QuadRobotStateNeck.h>
#include <topoquad_master/QuadRobotCmdNeckAngle.h>

using std::vector;

class Neck {
    public:
        Neck(): is_updated_(true), pan_(Joint(1)), tilt_(Joint(2)) {}
        void initialize(const Joint& pan, const Joint& tilt) {
            pan_ = pan;
            tilt_ = tilt;
            is_updated_ = true;
        }

        void SetJointAngles(const double& pan_angle, const double& tilt_angle) {
            pan_.SetJointAngle(pan_angle);
            tilt_.SetJointAngle(tilt_angle);
            is_updated_ = true;
        }
       
        bool operator==(const Neck& neck) const {
            return ( fabs(pan_.joint_angle_ - neck.pan_.joint_angle_) < 5e-3 
                    && fabs(tilt_.joint_angle_ - neck.tilt_.joint_angle_) < 5e-3 );
        }
        bool operator!=(const Neck& neck) const {
            return !(*this == neck);
        }

        bool is_updated_; // 関節角が更新されたかどうか
        Joint pan_;
        Joint tilt_;
};

Neck target_neck, goal_neck, present_neck;

void CallBackOfLegAngle(const topoquad_master::QuadRobotCmdNeckAngle::ConstPtr& msg) {
    target_neck.SetJointAngles(msg->angle_pan, msg->angle_tilt);
}

void CallBackOfDynamixelState(const dynamixel_handler::DynamixelState::ConstPtr& msg) {
     for (int i=0; i<msg->ids.size(); i++) {
        if(msg->ids[i] == present_neck.pan_.id_) present_neck.pan_.SetServoAngle(msg->present_angles[i]);
        if(msg->ids[i] == present_neck.tilt_.id_) present_neck.tilt_.SetServoAngle(msg->present_angles[i]);
        present_neck.is_updated_ = true;
        if(msg->ids[i] == goal_neck.pan_.id_) goal_neck.pan_.SetServoAngle(msg->goal_angles[i]);
        if(msg->ids[i] == goal_neck.tilt_.id_) goal_neck.tilt_.SetServoAngle(msg->goal_angles[i]);
        goal_neck.is_updated_ = true;
     }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neck_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::vector<int> ids_pantilt;
    if (!nh_p.getParam("pantilt_dynamixel_ID",   ids_pantilt)) ids_pantilt = { 43, 42};

    target_neck.initialize( Joint{ ids_pantilt[0], +1.0, 0.0/*rad*/, +0.92/800/*Nm/mA*/, 0.5/*Nm*/},
                            Joint{ ids_pantilt[1], +1.0, 0.0/*rad*/, +0.92/800/*Nm/mA*/, 0.5/*Nm*/} );
    goal_neck = target_neck;
    present_neck = target_neck;

    ros::Subscriber sub_neck_angle = nh.subscribe("/neck/angle", 10, CallBackOfLegAngle);
    ros::Publisher  pub_dyn_cmd   = nh.advertise<dynamixel_handler::DynamixelCmd>("/dynamixel/cmd", 10);
    
    ros::Subscriber sub_dyn_state   = nh.subscribe("/dynamixel/state",   10, CallBackOfDynamixelState);  // サーボの角度をsubscribe
    ros::Publisher  pub_neck_state_p   = nh.advertise<topoquad_master::QuadRobotStateNeck>("/neck/state/present", 10); // サーボの角度を関節の状態に変換してpublish
    ros::Publisher  pub_neck_state_g   = nh.advertise<topoquad_master::QuadRobotStateNeck>("/neck/state/goal", 10); // サーボの角度を関節の状態に変換してpublish

    ros::Rate rate(200);
    while(ros::ok()) {
        ros::spinOnce();  //

        if( target_neck.is_updated_ || target_neck != goal_neck ){
            dynamixel_handler::DynamixelCmd dyn_msg;
            dyn_msg.command = "write";
            dyn_msg.ids.push_back(target_neck.pan_.id_);
            dyn_msg.ids.push_back(target_neck.tilt_.id_);
            dyn_msg.goal_angles.push_back(target_neck.pan_.servo_angle_);
            dyn_msg.goal_angles.push_back(target_neck.tilt_.servo_angle_);
            pub_dyn_cmd.publish(dyn_msg);
            target_neck.is_updated_ = false;
        }

        if (present_neck.is_updated_) {
            topoquad_master::QuadRobotStateNeck neck_msg_p;
            neck_msg_p.angle_pan = present_neck.pan_.joint_angle_;
            neck_msg_p.angle_tilt = present_neck.tilt_.joint_angle_;
            pub_neck_state_p.publish(neck_msg_p);
            present_neck.is_updated_ = false;
        }

        if (goal_neck.is_updated_) {
            topoquad_master::QuadRobotStateNeck neck_msg_g;
            neck_msg_g.angle_pan = goal_neck.pan_.joint_angle_;
            neck_msg_g.angle_tilt = goal_neck.tilt_.joint_angle_;
            pub_neck_state_g.publish(neck_msg_g);
            goal_neck.is_updated_ = false;
        }

        rate.sleep();
    }
}