#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty, termios
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState

cmd_left = JointState()
cmd_right = JointState()
pub_left = rospy.Publisher('/panda_left/joint_velocities_commands', JointState, queue_size=1)
pub_right = rospy.Publisher('/panda_right/joint_velocities_commands', JointState, queue_size=1)


def keyboardLoop():
    rospy.init_node('curi_dual_arm_teleop_keyboard')
    rate = rospy.Rate(rospy.get_param('~hz', 200))

    joint1_vel_ = rospy.get_param('joint1_vel', 0.01)
    joint2_vel_ = rospy.get_param('joint2_vel', 0.01)
    joint3_vel_ = rospy.get_param('joint3_vel', 0.01)
    joint4_vel_ = rospy.get_param('joint4_vel', 0.01)
    joint5_vel_ = rospy.get_param('joint5_vel', 0.01)
    joint6_vel_ = rospy.get_param('joint6_vel', 0.01)
    joint7_vel_ = rospy.get_param('joint7_vel', 0.01)


    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == '1':
            left_vel_1 = joint1_vel_
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0
        elif ch == '2':
            left_vel_1 = 0
            left_vel_2 = joint2_vel_
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0
        elif ch == '3':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = joint3_vel_
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == '4':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = joint4_vel_
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == '5':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = joint5_vel_
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0


        elif ch == '6':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = joint6_vel_
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == '7':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = joint7_vel_
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == 'Q':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = joint1_vel_
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0
        elif ch == 'W':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = joint2_vel_
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0
        elif ch == 'E':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = joint3_vel_
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == 'R':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = joint4_vel_
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        elif ch == 'T':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = joint5_vel_
            right_vel_6 = 0
            right_vel_7 = 0


        elif ch == 'Y':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = joint6_vel_
            right_vel_7 = 0

        elif ch == 'U':
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = joint7_vel_


        elif ch == 'm':
            exit()
        else:
            left_vel_1 = 0
            left_vel_2 = 0
            left_vel_3 = 0
            left_vel_4 = 0
            left_vel_5 = 0
            left_vel_6 = 0
            left_vel_7 = 0
            right_vel_1 = 0
            right_vel_2 = 0
            right_vel_3 = 0
            right_vel_4 = 0
            right_vel_5 = 0
            right_vel_6 = 0
            right_vel_7 = 0

        cmd_left.velocity = [left_vel_1, left_vel_2, left_vel_3, left_vel_4, left_vel_5, left_vel_6, left_vel_7]
        cmd_right.velocity = [right_vel_1, right_vel_2, right_vel_3,right_vel_4,right_vel_5,right_vel_6,right_vel_7]
        pub_left.publish(cmd_left)
        pub_right.publish(cmd_right)
        rate.sleep()

        if ch == 'o':
            stop_robot()


def stop_robot():
    cmd_left.velocity = [0, 0, 0, 0, 0, 0, 0]
    cmd_right.velocity = [0, 0, 0, 0, 0, 0, 0]
    pub_left.publish(cmd_left)
    pub_right.publish(cmd_right)


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
