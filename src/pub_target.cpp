//
// Created by ray on 1/21/22.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
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
    ros::init(argc,argv,"PubFrankaTarget");
    //3 create ros nodehandle
    ros::NodeHandle nh;
    //4  create pub object
    ros::Publisher pub_left = nh.advertise<sensor_msgs::JointState>("/left/desire_joint",10);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::JointState>("/right/desire_joint",10);
    ros::Publisher pub_w_left = nh.advertise<geometry_msgs::PoseStamped>("/left/desire_pose", 10);
    ros::Publisher pub_w_right = nh.advertise<geometry_msgs::PoseStamped>("/right/desire_pose", 10);
    // create the pub msg
    sensor_msgs::JointState left_target_joint, right_target_joint;
    left_target_joint.position = {0,1,2,3,4,5,6};
    right_target_joint.position = {0,1,2,3,4,5,6};

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
    left_target_pose.pose.position.x = 0.5525;
    left_target_pose.pose.position.y = 0.4208;
    left_target_pose.pose.position.z = 0.7;
    left_target_pose.pose.orientation.x = 0.2263;
    left_target_pose.pose.orientation.y = -0.5168;
    left_target_pose.pose.orientation.z = -0.0198;
    left_target_pose.pose.orientation.w = 0.8254;

    right_target_pose.pose.position.x = 0.5525;
    right_target_pose.pose.position.y = 0.4208;
    right_target_pose.pose.position.z = 0.7;
    right_target_pose.pose.orientation.x = 0.2263;
    right_target_pose.pose.orientation.y = -0.5168;
    right_target_pose.pose.orientation.z = -0.0198;
    right_target_pose.pose.orientation.w = 0.8254;

    //set pub rate
    ros::Rate rate(10);
    double t = 0;
    //pub through loop
    while (ros::ok())
    {

        right_target_pose.pose.position.x = 0.5525 * sin(2*t);
        right_target_pose.pose.position.y = 0.4208 * cos(t);

        pub_left.publish(left_target_joint);
        pub_right.publish(right_target_joint);
        pub_w_left.publish(left_target_pose);
        pub_w_right.publish(right_target_pose);
        t += 1/200;
        //sleep
        rate.sleep();
        //suggess to add:
        ros::spinOnce();
        /* code */
    }


    return 0;
}