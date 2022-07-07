#!/usr/bin/env python3
"""franka_move_client
A ros node that
    - call the Follow Cartesian Trajectory action server of curi_dual arm
"""

import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal
from cartesian_control_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
import time, sys, math
import datetime
import math


def done_cb(state, result):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Finish Moving to the target point")


def active_cb():
    rospy.loginfo("Server is activated")


def fb_cb(fb):
    rospy.loginfo("Still Moving!")


if __name__ == "__main__":
    rospy.init_node("curi_dual_arm_move_client")
    client = actionlib.SimpleActionClient("/curi/torso/follow_cartesian_trajectory", FollowCartesianTrajectoryAction)
    client.wait_for_server()
    start_tra_point = CartesianTrajectoryPoint()
    Temp_tra_point1 = CartesianTrajectoryPoint()
    Temp_tra_point2 = CartesianTrajectoryPoint()
    final_tra_point = CartesianTrajectoryPoint()
    desired_trajectory = FollowCartesianTrajectoryGoal()
    start_tra_point.pose.position.x = 0.1;
    Temp_tra_point1.pose.position.x = 0.1;
    Temp_tra_point2.pose.position.x = 0.1;
    final_tra_point.pose.position.x = 0.2;
    desired_trajectory.trajectory.points = [start_tra_point, Temp_tra_point1, Temp_tra_point2, final_tra_point]
    client.send_goal(desired_trajectory, done_cb, active_cb, fb_cb)
    rospy.spin()
