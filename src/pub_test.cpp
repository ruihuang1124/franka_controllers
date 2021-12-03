#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>


/*
    pub person msg
    1. include .h file
    2. init ros node
    3. create ros nodehandle
    4. create pub object
    5. write pub logict and pub msg
*/

int main(int argc, char *argv[])
{
    const double PI = atan(1.)*4.;
    ROS_WARN_STREAM(PI);
    ROS_INFO("This is the msg publisher");
    setlocale(LC_ALL,"");
    //2 init node
    ros::init(argc,argv,"PubTestNode");
    //3 create ros nodehandle
    ros::NodeHandle nh;
    //4  create pub object
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/equilibrium_pose",10);
    ros::Publisher pub_w = nh.advertise<geometry_msgs::PoseStamped>("/whole_body_controller/equilibrium_pose",10);
    ros::Publisher pub_mobile = nh.advertise<geometry_msgs::Twist>("/summit_xl/robotnik_base_control/cmd_vel", 10);
    //5 write pub logic and pub msg
    // create the pub msg
    geometry_msgs::PoseStamped person;
    // person.pose.position.x = 0.6327;
    // person.pose.position.y = 0.0085;
    // person.pose.position.z = 1.1152;
    // person.pose.orientation.x = 0.9310;
    // person.pose.orientatio n.y = -0.3644;
    // person.pose.orientation.z = -0.0175;
    // person.pose.orientation.w = 0.0142;
    person.pose.position.x = 0.5525;
    person.pose.position.y = 0.4208;
    person.pose.position.z = 0.7;
    person.pose.orientation.x = 0.2263;
    person.pose.orientation.y = -0.5168;
    person.pose.orientation.z = -0.0198;
    person.pose.orientation.w = 0.8254;
    //x:0.5525, y:0.4208, z:0.7215, QX:0.2263, QY:-0.5168, QZ:-0.0198, QW:0.8254
    //x:0.6054, y:0.0177, z:1.2426, QX:0.9193, QY:-0.3630, QZ:0.1513, QW:-0.0151
    //x:0.6605, y:0.0038, z:1.0788, QX:0.9223, QY:-0.3778, QZ:0.0800, QW:0.0160
    //x:0.3034, y:0.0045, z:0.5813, QX:0.9138, QY:-0.3930, QZ:0.1023, QW:0.0107
    // x:0.3077, y:0.0014, z:0.5880, QX:0.9247, QY:-0.3807, QZ:-0.0011, QW:0.0034
    // x:0.7206, y:-0.0031, z:1.1095, QX:0.9234, QY:-0.3839, QZ:0.0002, QW:0.0010
    //x:0.3123, y:-0.0088, z:0.5837, QX:0.9317, QY:-0.3613, QZ:-0.0311, QW:0.0196

    geometry_msgs::Twist mobile_move_test;
    mobile_move_test.linear.x = -0.05;
    mobile_move_test.linear.y = 0;
    mobile_move_test.linear.z = 0;
    mobile_move_test.angular.x = 0;
    mobile_move_test.angular.y = 0;
    mobile_move_test.angular.z = 0;
    //set pub rate
    ros::Rate rate(6);
    //pub through loop
    while (ros::ok())
    {
        //change the date
        // main task: pub the msg
        pub.publish(person);
        pub_w.publish(person);
        // pub_mobile.publish(mobile_move_test);
        //sleep
        rate.sleep();
        //suggess to add:
        ros::spinOnce();
        /* code */
    }
    

    return 0;
}