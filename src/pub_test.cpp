#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <cmath>


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

    geometry_msgs::Twist mobile_move_test;
    mobile_move_test.linear.x = -0.05;
    mobile_move_test.linear.y = 0;
    mobile_move_test.linear.z = 0;
    mobile_move_test.angular.x = 0;
    mobile_move_test.angular.y = 0;
    mobile_move_test.angular.z = 0;
    
    //Code about Calculating the rotation matrix and gravity direction when tilt mounting
    Eigen::MatrixXf rotation1(4, 4), rotation2(4, 4), rotation3(4, 4), rotation4(4, 4), rotation5(4, 4), rotationFinal(4, 4), rotation6(4, 4), rotation7(4, 4);
    Eigen::MatrixXf vet1(4, 1), vet2(4, 1), vet3_gravity_incustomized(4, 1), vetG(4, 1), vettemp(4, 1);
    vet1 << 0, 1, 0, 1;
    vetG << 0, 0, -9.81, 0;
    rotation1 << 0, 0, 1, 0,
        0, 1, 0, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1; //rotation around Y 90 degree;
    rotation2 << 1, 0, 0, 0,
        0, cos(PI / 4), -sin(PI / 4), 0,
        0, sin(PI / 4), cos(PI / 4), 0,
        0, 0, 0, 1; //rotation around X -45 degree;
    rotation3 << cos(PI / 6), -sin(PI / 6), 0, 0,
        sin(PI / 6), cos(PI / 6), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1; //rotation around Z -30 degree;
    rotation6 << cos(-10 * PI / 180), 0, sin(-10 * PI / 180), 0,
        0, 1, 0, 0,
        -sin(-10 * PI / 180), 0, cos(-10 * PI / 180), 0,
        0, 0, 0, 1; //rotation vector in original base frame, Y+ direction

    rotation7 = rotation6 * rotation1 * rotation2 * rotation3;

    rotation4 = rotation1 * rotation2 * rotation3;

    vet2 = rotation4.inverse() * vet1;
    vet2(3, 0) = 0;
    vet2.normalize();
    ROS_ERROR("tes value is:%.8f,%.8f,%.8f，%.8f", vet2(0, 0), vet2(1, 0), vet2(2, 0), vet2.norm());
    double a = vet2(0, 0);
    double b = vet2(1, 0);
    double c = vet2(2, 0);
    double xita = -10 * PI / 180;
    ROS_ERROR("tes value is:%.8f,%.8f,%.8f，%.8f", vet2(0, 0), vet2(1, 0), vet2(2, 0), vet2.norm());
    rotation5(0, 0) = a * a + (1 - a * a) * cos(xita);
    rotation5(0, 1) = a * b * (1 - cos(xita)) + c * sin(xita);
    rotation5(0, 2) = a * c * (1 - cos(xita)) - b * sin(xita);
    rotation5(0, 3) = 0;
    rotation5(1, 0) = a * b * (1 - cos(xita)) - c * sin(xita);
    rotation5(1, 1) = b * b + (1 - b * b) * cos(xita);
    rotation5(1, 2) = b * c * (1 - cos(xita)) + a * sin(xita);
    rotation5(1, 3) = 0;
    rotation5(2, 0) = a * c * (1 - cos(xita)) + b * sin(xita);
    rotation5(2, 1) = b * c * (1 - cos(xita)) - a * sin(xita);
    rotation5(2, 2) = c * c + (1 - c * c) * cos(xita);
    rotation5(2, 3) = 0;
    rotation5(3, 0) = 0;
    rotation5(3, 1) = 0;
    rotation5(3, 2) = 0;
    rotation5(3, 3) = 1;

    // rotation5(0, 0) = a * a * (1-cos(xita)) +  cos(xita);
    // rotation5(0, 1) = a * b * (1 - cos(xita)) + c * sin(xita);
    // rotation5(0, 2) = a * c * (1 - cos(xita)) - b * sin(xita);
    // rotation5(0, 3) = 0;
    // rotation5(1, 0) = a * b * (1 - cos(xita)) - c * sin(xita);
    // rotation5(1, 1) = b * b * (1 - cos(xita)) + cos(xita);
    // rotation5(1, 2) = b * c * (1 - cos(xita)) + a * sin(xita);
    // rotation5(1, 3) = 0;
    // rotation5(2, 0) = a * c * (1 - cos(xita)) + b * sin(xita);
    // rotation5(2, 1) = b * c * (1 - cos(xita)) - a * sin(xita);
    // rotation5(2, 2) = c * c * (1 - cos(xita)) + cos(xita);
    // rotation5(2, 3) = 0;
    // rotation5(3, 0) = 0;
    // rotation5(3, 1) = 0;
    // rotation5(3, 2) = 0;
    // rotation5(3, 3) = 1;

    rotationFinal = rotation5 * rotation4;
    ROS_WARN_STREAM(rotationFinal);
    vet3_gravity_incustomized = rotation7.inverse() * vetG;
    vet3_gravity_incustomized(3, 0) = 0;
    ROS_ERROR("The final value is:%.16f,%.16f,%.16f,%.16f", vet3_gravity_incustomized(0, 0), vet3_gravity_incustomized(1, 0), vet3_gravity_incustomized(2, 0), vet3_gravity_incustomized.norm());

    //
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