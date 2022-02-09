//
// Created by ray on 1/21/22.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include "franka_controllers/createTrajectory.h"

class multiThreadService
{
public:
    multiThreadService()
    {
        left_home_joint_ = {0.562135426947945, 0.5743059311833298, 0.04049851962087446, -0.5328376126029071, 0.010031284674710173, 2.508450375456964, 0.8626112982928753};
        left_end_joint_ = {0.562135426947945, 0.5743059311833298, 0.04049851962087446, -0.5328376126029071, 0.010031284674710173, 2.508450375456964, 0.8626112982928753};
        right_end_joint_ = {-0.7780238883788124, 0.6200491727611475, 0.3834831916341185, -0.46359594027604656, -0.43295345889197445, 2.40979893658763, 0.8452322907588546};
        right_home_joint_ = {-0.7780238883788124, 0.6200491727611475, 0.3834831916341185, -0.46359594027604656, -0.43295345889197445, 2.40979893658763, 0.8452322907588546};
        nh.getParam("/left_home_rate",left_home_rate_);
        nh.getParam("/left_home_point_number",left_home_point_number_);
        nh.getParam("/left_end_rate",left_end_rate_);
        nh.getParam("/left_home_point_number",left_end_point_number_);
        nh.getParam("/right_home_rate",right_home_rate_);
        nh.getParam("/right_home_point_number",right_home_point_number_);
        nh.getParam("/right_end_rate",right_end_rate_);
        nh.getParam("/right_home_point_number",right_end_point_number_);
        nh.getParam("/left_command_quene_size",left_command_quene_size_);
        nh.getParam("/right_command_quene_size",right_command_quene_size_);

        //if the franka is install in a customized way:
        XmlRpc::XmlRpcValue left_home_joint_args, right_home_joint_args, left_end_joint_args, right_end_joint_args;
        if (!nh.getParam("/left_home_joint", left_home_joint_args)) {
            ROS_WARN_STREAM("Assuming the default home joint positon for left arm");
        }  else {
            if (left_home_joint_args.size() == left_home_joint_.size()) {
                ROS_WARN_STREAM("The home joint position for left arm is set as:" << left_home_joint_args);
                for (int i = 0; i < left_home_joint_args.size(); ++i) {
                    left_home_joint_[i] = left_home_joint_args[i];
                }
                left_home_go_ = true;
            }
        }

        if (!nh.getParam("/left_end_joint", left_end_joint_args)) {
            ROS_WARN_STREAM("Assuming the default end joint positon for left arm");
        }  else {
            if (left_end_joint_args.size() == left_end_joint_.size()) {
                ROS_WARN_STREAM("The end joint position for left arm is set as:" << left_end_joint_args);
                for (int i = 0; i < left_end_joint_args.size(); ++i) {
                    left_end_joint_[i] = left_end_joint_args[i];
                }
                left_end_go_ = true;
            }
        }

        if (!nh.getParam("/right_home_joint", right_home_joint_args)) {
            ROS_WARN_STREAM("Assuming the default home joint positon for right arm");
        }  else {
            if (right_home_joint_args.size() == right_home_joint_.size()) {
                ROS_WARN_STREAM("The home joint position for right arm is set as:" << right_home_joint_args);
                for (int i = 0; i < right_home_joint_args.size(); ++i) {
                    right_home_joint_[i] = right_home_joint_args[i];
                }
                right_home_go_ = true;
            }
        }

        if (!nh.getParam("/right_end_joint", right_end_joint_args)) {
            ROS_WARN_STREAM("Assuming the default end joint positon for right arm");
        }  else {
            if (right_end_joint_args.size() == right_end_joint_.size()) {
                ROS_WARN_STREAM("The end joint position for right arm is set as:" << right_end_joint_args);
                for (int i = 0; i < right_end_joint_args.size(); ++i) {
                    right_end_joint_[i] = right_end_joint_args[i];
                }
                right_end_go_ = true;
            }
        }
        right_execute_joint_.position = {-0.7780238883788124, 0.6200491727611475, 0.3834831916341185, -0.46359594027604656, -0.43295345889197445, 2.40979893658763, 0.8452322907588546};
        left_execute_joint_.position = {0.562135426947945, 0.5743059311833298, 0.04049851962087446, -0.5328376126029071, 0.010031284674710173, 2.508450375456964, 0.8626112982928753};
        right_initial_joint_ = {-0.7780238883788124, 0.6200491727611475, 0.3834831916341185, -0.46359594027604656, -0.43295345889197445, 2.40979893658763, 0.8452322907588546};
        left_initial_joint_ = {0.562135426947945, 0.5743059311833298, 0.04049851962087446, -0.5328376126029071, 0.010031284674710173, 2.508450375456964, 0.8626112982928753};
        pub_right_joint_ = nh.advertise<sensor_msgs::JointState>("/panda_right/joint_commands", left_command_quene_size_);
        pub_left_joint_ = nh.advertise<sensor_msgs::JointState>("/panda_left/joint_commands", right_command_quene_size_);
        pandaLeftSrv = nh.advertiseService("/panda_left/create_trajectory",&multiThreadService::pandaLeftCB, this);
        pandarightSrv = nh.advertiseService("/panda_right/create_trajectory",&multiThreadService::pandaRightCB, this);
    }


    bool pandaLeftCB(franka_controllers::createTrajectory::Request& req,
                     franka_controllers::createTrajectory::Response& res);
    bool pandaRightCB(franka_controllers::createTrajectory::Request& req,
                     franka_controllers::createTrajectory::Response& res);

private:
    ros::NodeHandle nh;
    ros::ServiceServer pandaLeftSrv;
    ros::ServiceServer pandarightSrv;
    ros::Publisher pub_right_joint_, pub_left_joint_;
    sensor_msgs::JointState left_execute_joint_, right_execute_joint_;
    std::array<double,7> right_initial_joint_, right_home_joint_, left_initial_joint_, left_home_joint_, left_end_joint_, right_end_joint_;
    int left_home_rate_, left_end_rate_, right_home_rate_, right_end_rate_, left_home_point_number_, left_end_point_number_, right_home_point_number_, right_end_point_number_;
    int left_command_quene_size_, right_command_quene_size_;
    bool left_home_go_, left_end_go_, right_home_go_, right_end_go_;
};

bool multiThreadService::pandaLeftCB(franka_controllers::createTrajectory::Request &req,
                                     franka_controllers::createTrajectory::Response &res) {
    if (strcmp(req.targetPoint.c_str(), "home_point") == 0){
        ROS_INFO("Start publishing a trajectory to: [%s] for Left arm!", req.targetPoint.c_str());
        for (int i = 0; i < 7; ++i) {
            left_initial_joint_[i] = req.currentJoint[i];
        }
        ros::Rate rate(left_home_rate_);
        //pub through loop
        for (int i = 0; i < left_home_point_number_; ++i) {
            for (int j = 0; j < 7; ++j) {
                left_execute_joint_.position[j] = left_initial_joint_[j]+ i * (left_home_joint_[j] - left_initial_joint_[j]) / left_home_point_number_;
            }
            pub_left_joint_.publish(left_execute_joint_);
            rate.sleep();
        }
        res.finishPublishCommand = true;
        return true;
    } else if (strcmp(req.targetPoint.c_str(), "end_point") == 0){
        ROS_INFO("Start publishing a trajectory to: [%s] for Left arm!", req.targetPoint.c_str());
        for (int i = 0; i < 7; ++i) {
            left_initial_joint_[i] = req.currentJoint[i];
        }
        ros::Rate rate(left_end_rate_);
        //pub through loop
        for (int i = 0; i < left_end_point_number_; ++i) {
            for (int j = 0; j < 7; ++j) {
                left_execute_joint_.position[j] = left_initial_joint_[j]+ i * (left_end_joint_[j] - left_initial_joint_[j]) / left_end_point_number_;
            }
            pub_left_joint_.publish(left_execute_joint_);
            rate.sleep();
        }
        res.finishPublishCommand = true;
        return true;
    } else{
        ROS_INFO("Receive target Point: [%s] for Left arm!", req.targetPoint.c_str());
        ROS_WARN("Invilid target position name, please check!!");
        return false;
    }
}

bool multiThreadService::pandaRightCB(franka_controllers::createTrajectory::Request &req,
                                      franka_controllers::createTrajectory::Response &res) {
    if (strcmp(req.targetPoint.c_str(), "home_point") == 0){
        ROS_INFO("Start publishing a trajectory to: [%s] for Right arm", req.targetPoint.c_str());
        for (int i = 0; i < 7; ++i) {
            right_initial_joint_[i] = req.currentJoint[i];
        }
        ros::Rate rate(right_home_rate_);
        //pub through loop
        for (int i = 0; i < right_home_point_number_; ++i) {
            for (int j = 0; j < 7; ++j) {
                right_execute_joint_.position[j] = right_initial_joint_[j]+ i * (right_home_joint_[j] - right_initial_joint_[j]) / right_home_point_number_;
            }
            pub_right_joint_.publish(right_execute_joint_);
            rate.sleep();
        }
        res.finishPublishCommand = true;
        return true;
    } else if (strcmp(req.targetPoint.c_str(), "end_point") == 0){
        ROS_INFO("Start publishing a trajectory to: [%s] for Right arm!", req.targetPoint.c_str());
        for (int i = 0; i < 7; ++i) {
            right_initial_joint_[i] = req.currentJoint[i];
        }
        ros::Rate rate(right_end_rate_);
        //pub through loop
        for (int i = 0; i < right_end_point_number_; ++i) {
            for (int j = 0; j < 7; ++j) {
                right_execute_joint_.position[j] = right_initial_joint_[j]+ i * (right_end_joint_[j] - right_initial_joint_[j]) / right_end_point_number_;
            }
            pub_right_joint_.publish(right_execute_joint_);
            rate.sleep();
        }
        res.finishPublishCommand = true;
        return true;
    } else{
        ROS_INFO("Receive target Point: [%s] for Left arm!", req.targetPoint.c_str());
        ROS_WARN("Invilid target position name, please check!!");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Panda_Trajectory_Creater");
    multiThreadService Service_obj;
    ros::MultiThreadedSpinner s(2);
    ros::spin(s);
    return 0;
}