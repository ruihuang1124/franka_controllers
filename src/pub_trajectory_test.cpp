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
#include <sstream>
//开启服务，服务会调用类里面的函数

class MoveCommander{

};

int main(int argc, char *argv[])
{
    bool left_go_ = true;
    bool right_go_ = true;
    const double PI = atan(1.)*4.;
    ROS_WARN_STREAM(PI);
    ROS_INFO("This is the msg publisher");
    setlocale(LC_ALL,"");
    //2 init node
    ros::init(argc,argv,"PubFrankaTarget");
    //3 create ros nodehandle
    ros::NodeHandle nh;
    //4  create pub object
    ros::Publisher pub_left_joint = nh.advertise<sensor_msgs::JointState>("/left/desire_joint", 10);
    ros::Publisher pub_right_joint = nh.advertise<sensor_msgs::JointState>("/right/desire_joint", 10);
    ros::Publisher pub_left_pose = nh.advertise<geometry_msgs::PoseStamped>("/left/desire_pose", 10);
    ros::Publisher pub_right_pose = nh.advertise<geometry_msgs::PoseStamped>("/right/desire_pose", 10);
    // create the pub msg
    sensor_msgs::JointState left_target_joint, right_target_joint;

    std::array<double,7> left_initial_joint_ = {-0.2984,0.6331,-0.4711,-0.5120,0.6123,2.4343,0.5819};
    std::array<double,7> right_initial_joint_ = {-0.2984,0.6331,-0.4711,-0.5120,0.6123,2.4343,0.5819};
    std::array<double,7> left_target_joint_ = {0.2,-0.6331,0.3,0.1,0.1,0,0.4};
    std::array<double,7> right_target_joint_ = {-0.2984,0.6331,-0.4711,-0.5120,0.6123,2.4343,0.5819};
    left_target_joint.position = {left_initial_joint_[0],left_initial_joint_[1],left_initial_joint_[2],left_initial_joint_[3],left_initial_joint_[4],left_initial_joint_[5],left_initial_joint_[6]};
    right_target_joint.position = {right_initial_joint_[0],right_initial_joint_[1],right_initial_joint_[2],right_initial_joint_[3],right_initial_joint_[4],right_initial_joint_[5],right_initial_joint_[6]};



    geometry_msgs::PoseStamped left_target_pose,right_target_pose;

    right_target_pose.pose.position.x = 0.7029;
    right_target_pose.pose.position.y = -0.3473;
    right_target_pose.pose.position.z = 0.8224;
    right_target_pose.pose.orientation.x = 0.7625;
    right_target_pose.pose.orientation.y = -0.1643;
    right_target_pose.pose.orientation.z = 0.6202;
    right_target_pose.pose.orientation.w = 0.0836;

    //set pub rate
    ros::Rate rate(2);
    double t = 0;
    int count_left = 0;
    bool left_finish= false;
    int left_point_number = 100;

    int count_right = 0;
    bool right_finish= false;
    int right_point_number = 100;

    //pub through loop
    while (ros::ok() && count_right <= right_point_number - 1 && count_left <= left_point_number - 1)
    {
//        if (left_go_){
//            for (int i = 0; i < left_point_number; ++i) {
//                for (int j = 0; j < 7; ++j) {
//                    left_target_joint.position[j] = left_initial_joint_[j]+i*(left_target_joint_[j]-left_initial_joint_[j])/left_point_number;
//                }
//                count_left = count_left + 1;
//                pub_left_joint.publish(left_target_joint);
//                rate.sleep();
//                ros::spinOnce();
//            }
//        }
        if (right_go_){
            for (int i = 0; i < right_point_number; ++i) {
                for (int j = 0; j < 7; ++j) {
                    right_target_joint.position[j] = right_initial_joint_[j]+i*(right_target_joint_[j]-right_initial_joint_[j])/right_point_number;
                }
                count_right = count_right + 1;
                pub_right_joint.publish(right_target_joint);
                rate.sleep();
                ros::spinOnce();
            }
        }
    }


    return 0;
}