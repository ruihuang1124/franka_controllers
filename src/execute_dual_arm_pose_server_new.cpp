//
// Created by ray on 8/5/22.
//

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include "roport/ExecuteGroupPose.h"
#include "roport/ExecuteDualArmPose.h"
#include <thread>

class MultiClientCallServer {
public:
    MultiClientCallServer() {
        left_arm_client_ = nh_.serviceClient<roport::ExecuteGroupPose>(
                "/panda_left/execute_create_ptp_cartesian_trajectory");
        right_arm_client_ = nh_.serviceClient<roport::ExecuteGroupPose>(
                "/panda_right/execute_create_ptp_cartesian_trajectory");

        ros::service::waitForService("/panda_left/execute_create_ptp_cartesian_trajectory");
        ros::service::waitForService("/panda_right/execute_create_ptp_cartesian_trajectory");

        start_calling_flag_ = new bool(false);
        finish_calling_flag_ = new bool(false);
        left_arm_execute_finished_flag_ = new bool(false);
        right_arm_execute_finished_flag_ = new bool(false);

        left_arm_msg_ = new roport::ExecuteGroupPose();
        right_arm_msg_ = new roport::ExecuteGroupPose();


        execute_curi_joint_server_ = nh_.advertiseService("/execute_dual_arm_pose",
                                                          &MultiClientCallServer::ExecuteCallingMultiClientCB, this);
        ROS_INFO("/execute_dual_arm_pose service server startup!");
    }

    ~MultiClientCallServer() {
        delete start_calling_flag_;
        delete finish_calling_flag_;
        delete left_arm_execute_finished_flag_;
        delete right_arm_execute_finished_flag_;
        delete left_arm_msg_;
        delete right_arm_msg_;
    }


    bool ExecuteCallingMultiClientCB(roport::ExecuteDualArmPose::Request &req,
                                     roport::ExecuteDualArmPose::Response &res);

    void CallingThread1();

    void CallingThread2();

    void ServiceThread();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient left_arm_client_;
    ros::ServiceClient right_arm_client_;
    ros::ServiceServer execute_curi_joint_server_;
    roport::ExecuteGroupPose *left_arm_msg_;
    roport::ExecuteGroupPose *right_arm_msg_;


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



bool MultiClientCallServer::ExecuteCallingMultiClientCB(roport::ExecuteDualArmPose::Request &req,
                                                        roport::ExecuteDualArmPose::Response &res) {
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
        ros::Duration(0.1).sleep();
    }
    *finish_calling_flag_ = false;
    res.result_status = res.SUCCEEDED;
    ROS_INFO("Finished Running!!");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CURI_Pose_Control_Service_Server_new");
    auto *MultiClientExecutor = new MultiClientCallServer();
    std::thread service_thread(&MultiClientCallServer::ServiceThread, MultiClientExecutor);
    std::thread calling_thread1(&MultiClientCallServer::CallingThread1, MultiClientExecutor);
    std::thread calling_thread2(&MultiClientCallServer::CallingThread2, MultiClientExecutor);
    service_thread.join();
    calling_thread1.join();
    calling_thread2.join();
    return 0;
}