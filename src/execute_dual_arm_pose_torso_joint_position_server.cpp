//
// Created by ray on 8/9/22.
//

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include "roport/ExecuteGroupPose.h"
#include <thread>
#include "roport/ExecuteJointPosition.h"
#include "roport/ExecuteDualArmPoseTorsoJointPosition.h"

class MultiClientCallServer {
public:
    MultiClientCallServer() {
        num_of_torso_joints_ = 3;

        left_arm_client_ = nh_.serviceClient<roport::ExecuteGroupPose>(
                "/panda_left/execute_create_ptp_cartesian_trajectory");
        right_arm_client_ = nh_.serviceClient<roport::ExecuteGroupPose>(
                "/panda_right/execute_create_ptp_cartesian_trajectory");
        torso_client_ = nh_.serviceClient<roport::ExecuteJointPosition>(
                "/curi/torso/execute_joint_position_srv_control");


        ros::service::waitForService("/panda_left/execute_create_ptp_cartesian_trajectory");
        ros::service::waitForService("/panda_right/execute_create_ptp_cartesian_trajectory");
        ros::service::waitForService("/curi/torso/execute_joint_position_srv_control");

        start_calling_flag_ = new bool(false);
        finish_calling_flag_ = new bool(false);
        left_arm_execute_finished_flag_ = new bool(false);
        right_arm_execute_finished_flag_ = new bool(false);
        torso_execute_finished_flag_ = new bool(false);

        left_arm_msg_ = new roport::ExecuteGroupPose();
        right_arm_msg_ = new roport::ExecuteGroupPose();
        torso_msg_ = new roport::ExecuteJointPosition();


        execute_curi_joint_server_ = nh_.advertiseService("/execute_dual_arm_pose_torso_joint_position",
                                                          &MultiClientCallServer::ExecuteCallingMultiClientCB, this);
        ROS_INFO("/execute_dual_arm_pose_torso_joint_position service server startup!");
    }

    ~MultiClientCallServer() {
        delete start_calling_flag_;
        delete finish_calling_flag_;
        delete left_arm_execute_finished_flag_;
        delete right_arm_execute_finished_flag_;
        delete torso_execute_finished_flag_;
        delete left_arm_msg_;
        delete right_arm_msg_;
        delete torso_msg_;
    }


    bool ExecuteCallingMultiClientCB(roport::ExecuteDualArmPoseTorsoJointPosition::Request &req,
                                     roport::ExecuteDualArmPoseTorsoJointPosition::Response &res);

    void CallingThread1();

    void CallingThread2();

    void CallingThread3();

    void ServiceThread();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient left_arm_client_;
    ros::ServiceClient right_arm_client_;
    ros::ServiceClient torso_client_;
    ros::ServiceServer execute_curi_joint_server_;
    roport::ExecuteGroupPose *left_arm_msg_;
    roport::ExecuteGroupPose *right_arm_msg_;
    roport::ExecuteJointPosition *torso_msg_;
    int num_of_torso_joints_;


    bool *left_arm_execute_finished_flag_;
    bool *right_arm_execute_finished_flag_;
    bool *torso_execute_finished_flag_;
    bool *start_calling_flag_;
    bool *finish_calling_flag_;
};

void MultiClientCallServer::ServiceThread() {
    while (ros::ok()) {
        ros::spinOnce();
    }
}

void MultiClientCallServer::CallingThread1() {
    while (ros::ok()) {
        if (*start_calling_flag_ && !*left_arm_execute_finished_flag_ && !*finish_calling_flag_) {
            left_arm_client_.call(*left_arm_msg_);
            ROS_INFO("finish calling left!");
            *left_arm_execute_finished_flag_ = true;
        }
    }
}

void MultiClientCallServer::CallingThread2() {
    while (ros::ok()) {
        if (*start_calling_flag_ && !*right_arm_execute_finished_flag_ && !*finish_calling_flag_) {
            right_arm_client_.call(*right_arm_msg_);
            ROS_INFO("finish calling right!");
            *right_arm_execute_finished_flag_ = true;
        }
    }
}

void MultiClientCallServer::CallingThread3() {
    while (ros::ok()) {
        if (*start_calling_flag_ && !*torso_execute_finished_flag_ && !*finish_calling_flag_) {
            torso_client_.call(*torso_msg_);
            ROS_INFO("finish calling torso!");
            *torso_execute_finished_flag_ = true;
        }
    }
}


bool MultiClientCallServer::ExecuteCallingMultiClientCB(roport::ExecuteDualArmPoseTorsoJointPosition::Request &req,
                                                        roport::ExecuteDualArmPoseTorsoJointPosition::Response &res) {
    ROS_INFO("Start Running multi client calling service!!");
    left_arm_msg_->request.goal.position.x = req.left_goal.position.x;
    left_arm_msg_->request.goal.position.y = req.left_goal.position.y;
    left_arm_msg_->request.goal.position.z = req.left_goal.position.z;
    left_arm_msg_->request.goal.orientation.x = req.left_goal.orientation.x;
    left_arm_msg_->request.goal.orientation.y = req.left_goal.orientation.y;
    left_arm_msg_->request.goal.orientation.z = req.left_goal.orientation.z;
    left_arm_msg_->request.goal.orientation.w = req.left_goal.orientation.w;
    left_arm_msg_->request.tolerance = req.run_time;

    right_arm_msg_->request.goal.position.x = req.right_goal.position.x;
    right_arm_msg_->request.goal.position.y = req.right_goal.position.y;
    right_arm_msg_->request.goal.position.z = req.right_goal.position.z;
    right_arm_msg_->request.goal.orientation.x = req.right_goal.orientation.x;
    right_arm_msg_->request.goal.orientation.y = req.right_goal.orientation.y;
    right_arm_msg_->request.goal.orientation.z = req.right_goal.orientation.z;
    right_arm_msg_->request.goal.orientation.w = req.right_goal.orientation.w;
    right_arm_msg_->request.tolerance = req.run_time;

    torso_msg_->request.goal_state.position.resize(num_of_torso_joints_);
    for (int i = 0; i < num_of_torso_joints_; ++i) {
        torso_msg_->request.goal_state.position[i] = req.torso_goal_state.position[i];
    }
    torso_msg_->request.speed_ratio = req.torso_speed_ratio;

    *left_arm_execute_finished_flag_ = false;
    *right_arm_execute_finished_flag_ = false;
    *torso_execute_finished_flag_ = false;
    *start_calling_flag_ = true;

    while (!*finish_calling_flag_) {
        if (*left_arm_execute_finished_flag_ && *right_arm_execute_finished_flag_ && *torso_execute_finished_flag_) {
            *finish_calling_flag_ = true;
            *left_arm_execute_finished_flag_ = false;
            *right_arm_execute_finished_flag_ = false;
            *torso_execute_finished_flag_ = false;
            *start_calling_flag_ = false;
        }
        ros::Duration(0.1).sleep();
    }
    *finish_calling_flag_ = false;
    res.result_status = res.SUCCEEDED;
    ROS_INFO("Finished Running!!");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CURI_Dual_Arm_Pose_Torso_Joint_Position_Control_Service_Server");
    auto *MultiClientExecutor = new MultiClientCallServer();
    std::thread service_thread(&MultiClientCallServer::ServiceThread, MultiClientExecutor);
    std::thread calling_thread1(&MultiClientCallServer::CallingThread1, MultiClientExecutor);
    std::thread calling_thread2(&MultiClientCallServer::CallingThread2, MultiClientExecutor);
    std::thread calling_thread3(&MultiClientCallServer::CallingThread3,MultiClientExecutor);
    service_thread.join();
    calling_thread1.join();
    calling_thread2.join();
    calling_thread3.join();
    return 0;
}