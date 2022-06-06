//
// Created by ray on 6/6/22.
//

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include "roport/ExecuteJointPosition.h"
#include <thread>

class MultiClientCallServer {
public:
    MultiClientCallServer() {
        num_of_arm_joints_ = 7;
        left_arm_client_ = nh_.serviceClient<roport::ExecuteJointPosition>(
                "/panda_left/execute_joint_position_srv_control");
        right_arm_client_ = nh_.serviceClient<roport::ExecuteJointPosition>(
                "/panda_right/execute_joint_position_srv_control");

        ros::service::waitForService("/panda_left/execute_joint_position_srv_control");
        ros::service::waitForService("/panda_right/execute_joint_position_srv_control");
        start_calling_flag_ = new bool(false);
        finish_calling_flag_ = new bool(false);
        left_arm_execute_finished_flag_ = new bool(false);
        right_arm_execute_finished_flag_ = new bool(false);
        left_arm_msg_ = new roport::ExecuteJointPosition();
        right_arm_msg_ = new roport::ExecuteJointPosition();
        execute_dual_arm_joint_server_ = nh_.advertiseService("/execute_dual_arm_joint_position",
                                                          &MultiClientCallServer::ExecuteCallingMultiClientCB, this);
        ROS_INFO("Multi client calling server startup!");
    }

    ~MultiClientCallServer() {
        delete start_calling_flag_;
        delete finish_calling_flag_;
        delete left_arm_execute_finished_flag_;
        delete right_arm_execute_finished_flag_;
        delete left_arm_msg_;
        delete right_arm_msg_;
    }


    bool ExecuteCallingMultiClientCB(roport::ExecuteJointPosition::Request &req,
                                     roport::ExecuteJointPosition::Response &res);

    void CallingThread1();

    void CallingThread2();

    void ServiceThread();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient left_arm_client_;
    ros::ServiceClient right_arm_client_;
    ros::ServiceServer execute_dual_arm_joint_server_;
    roport::ExecuteJointPosition *left_arm_msg_;
    roport::ExecuteJointPosition *right_arm_msg_;
    int num_of_arm_joints_;

    bool *left_arm_execute_finished_flag_;
    bool *right_arm_execute_finished_flag_;
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



bool MultiClientCallServer::ExecuteCallingMultiClientCB(roport::ExecuteJointPosition::Request &req,
                                                        roport::ExecuteJointPosition::Response &res) {
    ROS_INFO("Start Running dual arm joint position calling service!!");
//    left_arm_msg_->request.speed_ratio
    left_arm_msg_->request.speed_ratio = req.speed_ratio;
    right_arm_msg_->request.speed_ratio = req.speed_ratio;
    left_arm_msg_->request.goal_state.position.resize(num_of_arm_joints_);
    right_arm_msg_->request.goal_state.position.resize(num_of_arm_joints_);
    for (int i = 0; i < num_of_arm_joints_; ++i) {
        left_arm_msg_->request.goal_state.position[i] = req.goal_state.position[i];
        right_arm_msg_->request.goal_state.position[i] = req.goal_state.position[i+num_of_arm_joints_];
    }
    *left_arm_execute_finished_flag_ = false;
    *right_arm_execute_finished_flag_ = false;
    *start_calling_flag_ = true;

    while (!*finish_calling_flag_) {
        if (*left_arm_execute_finished_flag_ && *right_arm_execute_finished_flag_) {
            *finish_calling_flag_ = true;
            *left_arm_execute_finished_flag_ = false;
            *right_arm_execute_finished_flag_ = false;
            *start_calling_flag_ = false;
        }
        ros::Duration(0.001).sleep();
    }
    *finish_calling_flag_ = false;
    res.result_status = res.SUCCEEDED;
    ROS_INFO("Finished Running!!");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CURI_Joint_Position_Control_Service_Server");
    auto *MultiClientExecutor = new MultiClientCallServer();
    std::thread service_thread(&MultiClientCallServer::ServiceThread, MultiClientExecutor);
    std::thread calling_thread1(&MultiClientCallServer::CallingThread1, MultiClientExecutor);
    std::thread calling_thread2(&MultiClientCallServer::CallingThread2, MultiClientExecutor);
    service_thread.join();
    calling_thread1.join();
    calling_thread2.join();
    return 0;
}