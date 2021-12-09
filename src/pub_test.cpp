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

    ros::Publisher pub_w_left = nh.advertise<geometry_msgs::PoseStamped>("/left/desire_pose", 10);
    ros::Publisher pub_w_right = nh.advertise<geometry_msgs::PoseStamped>("/right/desire_pose", 10);
    // create the pub msg
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

    geometry_msgs::Twist mobile_move_test;
    mobile_move_test.linear.x = -0.05;
    mobile_move_test.linear.y = 0;
    mobile_move_test.linear.z = 0;
    mobile_move_test.angular.x = 0;
    mobile_move_test.angular.y = 0;
    mobile_move_test.angular.z = 0;
    
    //Code about Calculating the rotation matrix and gravity direction when tilt mounting
    Eigen::MatrixXf rotation_left_1(4, 4), rotation_left_2(4, 4), rotation_left_3(4, 4), rotation4(4, 4), rotation5(4, 4), rotationFinal(4, 4), rotation_left_4(4, 4), rotation_left(4, 4), rotation_right_1(4, 4), rotation_right_2(4, 4), rotation_right_3(4, 4), rotation_right_4(4, 4), rotation_right(4, 4);
    Eigen::MatrixXf vet1(4, 1), vet2(4, 1), left_gravity_incustomized_direction(4, 1), gravity_earth_direction(4, 1), vettemp(4, 1), right_gravity_incustomized_direction(4, 1);
    Eigen::Matrix<double, 3, 3> rotationMatrix_left, rotationMatrix_right;
    vet1 << 0, 1, 0, 1;
    gravity_earth_direction << 0, 0, -9.81, 0;
    rotation_left_1 << 0, 0, 1, 0,
        0, 1, 0, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1; //rotation around Y 90 degree;
    rotation_left_2 << 1, 0, 0, 0,
        0, cos(- PI / 4), -sin(- PI / 4), 0,
        0, sin(- PI / 4), cos(- PI / 4), 0,
        0, 0, 0, 1; //rotation around X -45 degree;
    rotation_left_3 << cos(- PI / 6), -sin(- PI / 6), 0, 0,
        sin(- PI / 6), cos(- PI / 6), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1; //rotation around Z -30 degree;
    rotation_left_4 << cos(-10 * PI / 180), 0, sin(-10 * PI / 180), 0,
        0, 1, 0, 0,
        -sin(-10 * PI / 180), 0, cos(-10 * PI / 180), 0,
        0, 0, 0, 1; //rotation vector in original base frame, Y+ direction

    rotation_left = rotation_left_4 * rotation_left_1 * rotation_left_2 * rotation_left_3;
    ROS_ERROR("Left_arm rotataion matrix:");
    ROS_WARN_STREAM(rotation_left);
    left_gravity_incustomized_direction = rotation_left.inverse() * gravity_earth_direction;
    left_gravity_incustomized_direction(3, 0) = 0;
    ROS_ERROR("Light_arm gravity direction is:%.16f,%.16f,%.16f,%.16f", left_gravity_incustomized_direction(0, 0), left_gravity_incustomized_direction(1, 0), left_gravity_incustomized_direction(2, 0), left_gravity_incustomized_direction.norm());

    rotationMatrix_left(0, 0) = rotation_left(0, 0);
    rotationMatrix_left(0, 1) = rotation_left(0, 1);
    rotationMatrix_left(0, 2) = rotation_left(0, 2);
    rotationMatrix_left(1, 0) = rotation_left(1, 0);
    rotationMatrix_left(1, 1) = rotation_left(1, 1);
    rotationMatrix_left(1, 2) = rotation_left(1, 2);
    rotationMatrix_left(2, 0) = rotation_left(2, 0);
    rotationMatrix_left(2, 1) = rotation_left(2, 1);
    rotationMatrix_left(2, 2) = rotation_left(2, 2);

    Eigen::Quaterniond q_left(rotationMatrix_left);
    ROS_ERROR("left_arm rotation:x:%.16f,y:%.16f,z:%.16f,w:%.16f", q_left.coeffs().transpose().x(), q_left.coeffs().transpose().y(), q_left.coeffs().transpose().z(), q_left.coeffs().transpose().w());
    ROS_WARN_STREAM(q_left.coeffs().transpose());
    Eigen::Quaterniond left_install_rotation(0.747283, -0.436865, 0.49775, 0.0544277);
    Eigen::Matrix3d franka_install_rotation_matrix = left_install_rotation.toRotationMatrix();
    ROS_WARN_STREAM(franka_install_rotation_matrix);

    rotation_right_1 << 0, 0, 1, 0,
        0, 1, 0, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1; //rotation around Y 90 degree;
    rotation_right_2 << 1, 0, 0, 0,
        0, cos(PI / 4), -sin(PI / 4), 0,
        0, sin(PI / 4), cos(PI / 4), 0,
        0, 0, 0, 1; //rotation around X -45 degree;
    rotation_right_3 << cos(PI / 6), -sin(PI / 6), 0, 0,
        sin(PI / 6), cos(PI / 6), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1; //rotation around Z -30 degree;
    rotation_right_4 << cos(-10 * PI / 180), 0, sin(-10 * PI / 180), 0,
        0, 1, 0, 0,
        -sin(-10 * PI / 180), 0, cos(-10 * PI / 180), 0,
        0, 0, 0, 1; //rotation vector in original base frame, Y+ direction

    rotation_right = rotation_right_4 * rotation_right_1 * rotation_right_2 * rotation_right_3;
    ROS_ERROR("Right_arm rotataion matrix:");
    ROS_WARN_STREAM(rotation_right);
    right_gravity_incustomized_direction = rotation_right.inverse() * gravity_earth_direction;
    right_gravity_incustomized_direction(3, 0) = 0;
    ROS_ERROR("Right_arm gravity direction is:%.16f,%.16f,%.16f,%.16f", right_gravity_incustomized_direction(0, 0), right_gravity_incustomized_direction(1, 0), right_gravity_incustomized_direction(2, 0), right_gravity_incustomized_direction.norm());
    rotationMatrix_right(0, 0) = rotation_right(0, 0);
    rotationMatrix_right(0, 1) = rotation_right(0, 1);
    rotationMatrix_right(0, 2) = rotation_right(0, 2);
    rotationMatrix_right(1, 0) = rotation_right(1, 0);
    rotationMatrix_right(1, 1) = rotation_right(1, 1);
    rotationMatrix_right(1, 2) = rotation_right(1, 2);
    rotationMatrix_right(2, 0) = rotation_right(2, 0);
    rotationMatrix_right(2, 1) = rotation_right(2, 1);
    rotationMatrix_right(2, 2) = rotation_right(2, 2);

    Eigen::Quaterniond q_right(rotationMatrix_right);
    ROS_ERROR("right_arm rotation:x:%.16f,y:%.16f,z:%.16f,w:%.16f", q_right.coeffs().transpose().x(), q_right.coeffs().transpose().y(), q_right.coeffs().transpose().z(), q_right.coeffs().transpose().w());
    ROS_WARN_STREAM(q_right.coeffs().transpose());

    rotation4 = rotation_left_1 * rotation_left_2 * rotation_left_3;
    vet2 = rotation4.inverse() * vet1;
    vet2(3, 0) = 0;
    vet2.normalize();
    // ROS_ERROR("tes value is:%.8f,%.8f,%.8f，%.8f", vet2(0, 0), vet2(1, 0), vet2(2, 0), vet2.norm());
    double a = vet2(0, 0);
    double b = vet2(1, 0);
    double c = vet2(2, 0);
    double xita = -10 * PI / 180;
    // ROS_ERROR("tes value is:%.8f,%.8f,%.8f，%.8f", vet2(0, 0), vet2(1, 0), vet2(2, 0), vet2.norm());
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

    // Eigen::Quaterniond q_compare(0.747283, 0.436865, 0.49775, -0.0544277);
    // ROS_WARN_STREAM(q_compare.coeffs().transpose());
    // // ROS_ERROR("x:%.16f,y:%.16f,z:%.16f,w:%.16f", q.coeffs().transpose().x(), q.coeffs().transpose().y(), q.coeffs().transpose().z(), q.coeffs().transpose().w())

    // Eigen::Matrix3d rotationMatrix1 = q_left.toRotationMatrix();
    // ROS_WARN_STREAM(rotationMatrix1);
    // Eigen::Matrix3d rotationMatrix1_compare = q_compare.toRotationMatrix();
    // ROS_WARN_STREAM(rotationMatrix1_compare);
    // ROS_ERROR("The final value is:%.16f,%.16f,%.16f,%.16f",q.x(),q.y,q.z,q.w);

    //
    //set pub rate
    ros::Rate rate(10);
    //pub through loop
    while (ros::ok())
    {
        //change the date
        // main task: pub the msg
        // pub.publish(left_target_pose);
        // pub_w.publish(left_target_pose);
        // pub_mobile.publish(mobile_move_test);
        pub_w_left.publish(left_target_pose);
        pub_w_right.publish(right_target_pose);
        //sleep
        rate.sleep();
        //suggess to add:
        ros::spinOnce();
        /* code */
    }
    

    return 0;
}