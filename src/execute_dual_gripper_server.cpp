//
// Created by ray on 5/9/22.
//
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <boost/thread.hpp>
#include <thread>
#include "roport/ExecuteFrankaGripperGrasp.h"

class DualGripperCallServer {
public:
    DualGripperCallServer() {
        left_gripper_client_ = nh_.serviceClient<roport::ExecuteFrankaGripperGrasp>(
                "/panda_left/execute_franka_gripper_grasp");
        right_gripper_client_ = nh_.serviceClient<roport::ExecuteFrankaGripperGrasp>(
                "/panda_right/execute_franka_gripper_grasp");

        ros::service::waitForService("/panda_left/execute_franka_gripper_grasp");
        ros::service::waitForService("/panda_right/execute_franka_gripper_grasp");

        start_calling_flag_ = new bool(false);
        finish_calling_flag_ = new bool(false);
        left_gripper_execute_finished_flag_ = new bool(false);
        right_gripper_execute_finished_flag_ = new bool(false);
        left_gripper_msg_ = new roport::ExecuteFrankaGripperGrasp();
        right_gripper_msg_ = new roport::ExecuteFrankaGripperGrasp();

        execute_curi_joint_server_ = nh_.advertiseService("/execute_curi_dual_gripper",
                                                          &DualGripperCallServer::ExecuteCallingMultiClientCB, this);
        ROS_INFO("Multi client calling grippers server startup!");
    }

    ~DualGripperCallServer() {
        delete start_calling_flag_;
        delete finish_calling_flag_;
        delete left_gripper_execute_finished_flag_;
        delete right_gripper_execute_finished_flag_;
        delete left_gripper_msg_;
        delete right_gripper_msg_;
    }


    bool ExecuteCallingMultiClientCB(roport::ExecuteFrankaGripperGrasp::Request &req,
                                     roport::ExecuteFrankaGripperGrasp::Response &res);

    void CallingThreadLeft();

    void CallingThreadRight();

    void ServiceThread();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient left_gripper_client_;
    ros::ServiceClient right_gripper_client_;
    ros::ServiceServer execute_curi_joint_server_;
    roport::ExecuteFrankaGripperGrasp *left_gripper_msg_;
    roport::ExecuteFrankaGripperGrasp *right_gripper_msg_;


    bool *left_gripper_execute_finished_flag_;
    bool *right_gripper_execute_finished_flag_;
    bool *start_calling_flag_;
    bool *finish_calling_flag_;
};

void DualGripperCallServer::ServiceThread() {
    while (ros::ok()) {
        ros::spinOnce();
    }
}

void DualGripperCallServer::CallingThreadLeft() {
    while (ros::ok()) {
        if (*start_calling_flag_ && !*left_gripper_execute_finished_flag_ && !*finish_calling_flag_) {
            left_gripper_client_.call(*left_gripper_msg_);
            ROS_INFO("finish calling left!");
            *left_gripper_execute_finished_flag_ = true;
        }
    }
}

void DualGripperCallServer::CallingThreadRight() {
    while (ros::ok()) {
        if (*start_calling_flag_ && !*right_gripper_execute_finished_flag_ && !*finish_calling_flag_) {
            right_gripper_client_.call(*right_gripper_msg_);
            ROS_INFO("finish calling right!");
            *right_gripper_execute_finished_flag_ = true;
        }
    }
}



bool DualGripperCallServer::ExecuteCallingMultiClientCB(roport::ExecuteFrankaGripperGrasp::Request &req,
                                                        roport::ExecuteFrankaGripperGrasp::Response &res) {
    ROS_INFO("Start Running multi gripper calling service!!");
    left_gripper_msg_->request.width = req.width;
    left_gripper_msg_->request.force = req.force;
    left_gripper_msg_->request.speed = req.speed;
    left_gripper_msg_->request.epsilon_inner = req.epsilon_inner;
    left_gripper_msg_->request.epsilon_outer = req.epsilon_outer;
    right_gripper_msg_->request.width = req.width;
    right_gripper_msg_->request.force = req.force;
    right_gripper_msg_->request.speed = req.speed;
    right_gripper_msg_->request.epsilon_inner = req.epsilon_inner;
    right_gripper_msg_->request.epsilon_outer = req.epsilon_outer;

    *left_gripper_execute_finished_flag_ = false;
    *right_gripper_execute_finished_flag_ = false;
    *start_calling_flag_ = true;

    while (!*finish_calling_flag_) {
        if (*left_gripper_execute_finished_flag_ && *right_gripper_execute_finished_flag_) {
            *finish_calling_flag_ = true;
            *left_gripper_execute_finished_flag_ = false;
            *right_gripper_execute_finished_flag_ = false;
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
    ros::init(argc, argv, "CURI_Dual_Gripper_Service_Server");
    auto *MultiClientExecutor = new DualGripperCallServer();
    std::thread service_thread(&DualGripperCallServer::ServiceThread, MultiClientExecutor);
    std::thread calling_thread_left(&DualGripperCallServer::CallingThreadLeft, MultiClientExecutor);
    std::thread calling_thread_right(&DualGripperCallServer::CallingThreadRight, MultiClientExecutor);
    service_thread.join();
    calling_thread_left.join();
    calling_thread_right.join();
    return 0;
}