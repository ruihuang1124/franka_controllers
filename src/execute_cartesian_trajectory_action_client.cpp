//
// Created by ray on 8/1/22.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "roport/GetGroupPose.h"
#include "actionlib/client/simple_action_client.h"
#include "cartesian_control_msgs/FollowCartesianTrajectoryAction.h"
#include "roport/ExecuteGroupPose.h"
#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include <chrono>

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> Arm_Client;

auto make_point(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

auto make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0) {
    geometry_msgs::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

auto make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q) {
    geometry_msgs::Pose pose;
    pose.position = p;
    pose.orientation = q;
    return pose;
}

void done_cb(const actionlib::SimpleClientGoalState &state,
             const cartesian_control_msgs::FollowCartesianTrajectoryResultConstPtr &result) {
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Succeed!");
    } else {
        ROS_INFO("Fail!!");
    }

}

void active_cb() {
    ROS_INFO("Robot start moving");
}

void feedback_cb(const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr &feedback) {
//    ROS_INFO("Under executing!!");
}


class multiThreadService {
public:
    multiThreadService() {
        default_time_step_ = 0.001;
//        left_home_pose_ = make_pose(make_point(0.632, 0.459, 0.783),
//                                    make_quaternion(0.6991598868218329, 0.4651891646565687, 0.5412705526514202,
//                                                    0.04235254932858261));
//        right_home_pose_ = make_pose(make_point(0.632, 0.459, 0.783),
//                                     make_quaternion(0.6991598868218329, 0.4651891646565687, 0.5412705526514202,
//                                                     0.04235254932858261));
        pandaLeftSrv = nh.advertiseService("/panda_left/create_cartesian_trajectory", &multiThreadService::pandaLeftCB,
                                           this);
        pandarightSrv = nh.advertiseService("/panda_right/create_cartesian_trajectory",
                                            &multiThreadService::pandaRightCB, this);

        action_client_left_ = new Arm_Client(nh, "/panda_left/execute_cartesian_trajectory_server_", true);
        action_client_right_ = new Arm_Client(nh, "/panda_right/execute_cartesian_trajectory_server_", true);
        action_client_left_->waitForServer();
        action_client_right_->waitForServer();

        getCurrentPoseClientLeft = nh.serviceClient<roport::GetGroupPose>("/panda_left/get_group_cartesian_pose");
        getCurrentPoseClientRight = nh.serviceClient<roport::GetGroupPose>("/panda_right/get_group_cartesian_pose");
        ros::service::waitForService("/panda_left/get_group_cartesian_pose");
        ros::service::waitForService("/panda_right/get_group_cartesian_pose");
    }

    ~multiThreadService() {
        delete action_client_left_;
        delete action_client_right_;
    }

private:
    ros::NodeHandle nh;
    ros::ServiceServer pandaLeftSrv;
    ros::ServiceServer pandarightSrv;
    ros::Publisher pub_right_joint_, pub_left_joint_;
//    geometry_msgs::Pose left_home_pose_, right_home_pose_;
    Eigen::Matrix<double, 7, 1> upper_joint_position_warning_limits_, lower_joint_position_warning_limits_;
    ros::ServiceClient getCurrentPoseClientLeft, getCurrentPoseClientRight;
    Arm_Client *action_client_left_;
    Arm_Client *action_client_right_;
    double default_time_step_;

    void get_min_jerk_pose_trajectory(double time_step, double run_time, geometry_msgs::Pose initial_pose,
                                      geometry_msgs::Pose desired_pose,
                                      cartesian_control_msgs::FollowCartesianTrajectoryGoal &goal_cartesian_trajectory);


    bool pandaLeftCB(roport::ExecuteGroupPose::Request &req,
                     roport::ExecuteGroupPose::Response &res);

    bool pandaRightCB(roport::ExecuteGroupPose::Request &req,
                      roport::ExecuteGroupPose::Response &res);

//    void done_cb(const actionlib::SimpleClientGoalState &state, const cartesian_control_msgs::FollowCartesianTrajectoryResultConstPtr &result);
//    void active_cb();
//    void feedback_cb(const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr &feedback);
};

void multiThreadService::get_min_jerk_pose_trajectory(double time_step, double run_time,
                                                      geometry_msgs::Pose initial_pose,
                                                      geometry_msgs::Pose desired_pose,
                                                      cartesian_control_msgs::FollowCartesianTrajectoryGoal &goal_cartesian_trajectory) {
    int trajectory_length = run_time / time_step;
    Eigen::Quaterniond initial_quaterniond(initial_pose.orientation.w, initial_pose.orientation.x,
                                           initial_pose.orientation.y, initial_pose.orientation.z);
    Eigen::Quaterniond desired_quaterniond(desired_pose.orientation.w, desired_pose.orientation.x,
                                           desired_pose.orientation.y, desired_pose.orientation.z);
    Eigen::Quaterniond trajectory_point_pose_quaternion;
    cartesian_control_msgs::CartesianTrajectoryPoint trajectory_point;
    for (int i = 0; i < trajectory_length; ++i) {
        double map_t = double(i)/ double(trajectory_length);
        double slerp_t = (10 * std::pow(map_t, 3) - 15 * std::pow(map_t, 4) + 6 * std::pow(map_t, 5));
        trajectory_point.pose.position.x =
                initial_pose.position.x + (desired_pose.position.x - initial_pose.position.x) * slerp_t;
        trajectory_point.pose.position.y =
                initial_pose.position.y + (desired_pose.position.y - initial_pose.position.y) * slerp_t;
        trajectory_point.pose.position.z =
                initial_pose.position.z + (desired_pose.position.z - initial_pose.position.z) * slerp_t;
        trajectory_point_pose_quaternion = initial_quaterniond.slerp(slerp_t, desired_quaterniond);
        trajectory_point.pose.orientation.x = trajectory_point_pose_quaternion.x();
        trajectory_point.pose.orientation.y = trajectory_point_pose_quaternion.y();
        trajectory_point.pose.orientation.z = trajectory_point_pose_quaternion.z();
        trajectory_point.pose.orientation.w = trajectory_point_pose_quaternion.w();
        goal_cartesian_trajectory.trajectory.points.push_back(trajectory_point);
    }
}

bool multiThreadService::pandaLeftCB(roport::ExecuteGroupPose::Request &req,
                                     roport::ExecuteGroupPose::Response &res) {
    roport::GetGroupPose get_current_pose;
    get_current_pose.request.group_name = "End-effector";
    getCurrentPoseClientLeft.call(get_current_pose);
    //TODO change the time_step and run_time with new define req value.
    double trajectory_run_time = req.tolerance;
    if (get_current_pose.response.result_status == res.SUCCEEDED) {
        cartesian_control_msgs::FollowCartesianTrajectoryGoal left_goal_cartesian_trajectory;
        multiThreadService::get_min_jerk_pose_trajectory(default_time_step_, trajectory_run_time,
                                                         get_current_pose.response.pose, req.goal,
                                                         left_goal_cartesian_trajectory);
        action_client_left_->sendGoal(left_goal_cartesian_trajectory, &done_cb, &active_cb, &feedback_cb);
    } else {
        ROS_WARN("Fail to get the current left arm pose!!!");
    }
}

bool multiThreadService::pandaRightCB(roport::ExecuteGroupPose::Request &req,
                                      roport::ExecuteGroupPose::Response &res) {
    roport::GetGroupPose get_current_pose;
    get_current_pose.request.group_name = "End-effector";
    getCurrentPoseClientRight.call(get_current_pose);
    //TODO change the time_step and run_time with new define req value.
    double trajectory_run_time = req.tolerance;
    if (get_current_pose.response.result_status == res.SUCCEEDED) {
        cartesian_control_msgs::FollowCartesianTrajectoryGoal right_goal_cartesian_trajectory;
        multiThreadService::get_min_jerk_pose_trajectory(default_time_step_, trajectory_run_time,
                                                         get_current_pose.response.pose, req.goal,
                                                         right_goal_cartesian_trajectory);
        action_client_right_->sendGoal(right_goal_cartesian_trajectory, &done_cb, &active_cb, &feedback_cb);
    } else {
        ROS_WARN("Fail to get the right arm pose!!!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Dual_Arm_Cartesian_Pose_Trajectory_Generator");
    ROS_INFO("execute_cartesian_trajectory_action_client node start.");
    multiThreadService Service_obj;
    ros::MultiThreadedSpinner s(4);
    ros::spin(s);
    return 0;
}