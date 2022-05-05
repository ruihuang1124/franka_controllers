//
// Created by ray on 1/21/22.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>


int main(int argc, char *argv[])
{
    const double PI = atan(1.)*4.;
    ROS_WARN_STREAM(PI);
    ROS_INFO("This is the msg publisher");
    setlocale(LC_ALL,"");
    //2 init node
    ros::init(argc,argv,"TargetObjectPosePublisher");
    //3 create ros nodehandle
    ros::NodeHandle nh;
    //4  create pub object
//    ros::Publisher pub_left = nh.advertise<sensor_msgs::JointState>("/left/desire_joint",10);
//    ros::Publisher pub_right = nh.advertise<sensor_msgs::JointState>("/right/desire_joint",10);
//    ros::Publisher pub_w_left = nh.advertise<geometry_msgs::PoseStamped>("/left/desire_pose", 10);
//    ros::Publisher pub_w_right = nh.advertise<geometry_msgs::PoseStamped>("/right/desire_pose", 10);

    ros::Publisher pub_fake_object = nh.advertise<geometry_msgs::Pose>("/object/pose", 10);
    ros::Publisher pub_franka_gripper_target = nh.advertise<sensor_msgs::JointState>("/panda_left/gripper/joint_commands",10);
    sensor_msgs::JointState left_finger_cmd;
    left_finger_cmd.position.resize(2);
    left_finger_cmd.position[0] = 0.0;
    left_finger_cmd.position[1] = 0.0;
    // create the pub msg
    sensor_msgs::JointState left_target_joint, right_target_joint;

    // left_target_joint.position = {0,1,2,3,4,5,6};
    right_target_joint.position = {-0.2984,0.6331,-0.4711,-0.5120,0.6123,2.4343,0.5819};

//    left_target_joint.position[0] = 0;
//    left_target_joint.position[1] = 0;
//    left_target_joint.position[2] = 0;
//    left_target_joint.position[3] = 0;
//    left_target_joint.position[4] = 0;
//    left_target_joint.position[5] = 0;
//    left_target_joint.position[6] = 0;
//
//    right_target_joint.position[0] = 0;
//    right_target_joint.position[1] = 0;
//    right_target_joint.position[2] = 0;
//    right_target_joint.position[3] = 0;
//    right_target_joint.position[4] = 0;
//    right_target_joint.position[5] = 0;
//    right_target_joint.position[6] = 0;


    geometry_msgs::PoseStamped left_target_pose,right_target_pose;
    geometry_msgs::Pose target_object_pose;
    // left_target_pose.pose.position.x = 0.7029;
    // left_target_pose.pose.position.y = -0.3473;
    // left_target_pose.pose.position.z = 0.8224;
    // left_target_pose.pose.orientation.x = 0.7625;
    // left_target_pose.pose.orientation.y = -0.1643;
    // left_target_pose.pose.orientation.z = 0.6202;
    // left_target_pose.pose.orientation.w = 0.0836;

//    right_target_pose.pose.position.x = 0.7029;
//    right_target_pose.pose.position.y = -0.3473;
//    right_target_pose.pose.position.z = 0.8224;
//    right_target_pose.pose.orientation.x = 0.7625;
//    right_target_pose.pose.orientation.y = -0.1643;
//    right_target_pose.pose.orientation.z = 0.6202;
//    right_target_pose.pose.orientation.w = 0.0836;
    target_object_pose.position.x = -2.3999;
    target_object_pose.position.y = -0.5999;
    target_object_pose.position.z = 0.9625;
    target_object_pose.orientation.x = 0;
    target_object_pose.orientation.y = 0;
    target_object_pose.orientation.z = 0;
    target_object_pose.orientation.w = 1.0;

    //set pub rate
    ros::Rate rate(50);
    double t = 0;
    //pub through loop
    while (ros::ok())
    {

        // right_target_pose.pose.position.x = 0.5525 * sin(2*t);
        // right_target_pose.pose.position.y = 0.4208 * cos(t);

        // pub_left.publish(left_target_joint);
        pub_fake_object.publish(target_object_pose);
        pub_franka_gripper_target.publish(left_finger_cmd);
        // pub_w_left.publish(left_target_pose);
        // pub_w_right.publish(right_target_pose);
        // t += 1/200;
        //sleep
        rate.sleep();
        //suggess to add:
        ros::spinOnce();
        /* code */
    }


    return 0;
}