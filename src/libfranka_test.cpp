//
// Created by ray on 1/21/22.
//
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>

#include <franka/exception.h>
#include <franka/model.h>

/**
 * @example print_joint_poses.cpp
 * An example showing how to use the model library that prints the transformation
 * matrix of each joint with respect to the base frame.
 */

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try {
        franka::Robot robot(argv[1]);

        franka::RobotState state = robot.readOnce();

        franka::Model model(robot.loadModel());
        for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
             frame++) {
            std::cout << model.pose(frame, state) << std::endl;
        }
    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

