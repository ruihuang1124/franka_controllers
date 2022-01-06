//
// Created by ray on 1/6/22.
//

#include "franka_controllers/dual_arm_joint_position_controller.h"

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace franka_controllers
{

    bool DualArmJointPositionController::initArm(
            hardware_interface::RobotHW *robot_hw,
            const std::string &arm_id,
            const std::vector<std::string> &joint_names)
    {
        FrankaDataContainerJointCommand arm_data;
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Error getting model interface from hardware");
            return false;
        }
        try
        {
            arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Exception getting model handle from "
                    "interface: "
                            << ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Error getting state interface from hardware");
            return false;
        }
        try
        {
            arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Exception getting state handle from "
                    "interface: "
                            << ex.what());
            return false;
        }

        auto *position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Error getting position joint interface from "
                    "hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                arm_data.position_joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM(
                        "DualArmJointPositionController: Exception getting joint handles: "
                                << ex.what());
                return false;
            }
        }
        arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));
        return true;
    }

    bool DualArmJointPositionController::init(hardware_interface::RobotHW *robot_hw,
                                                   ros::NodeHandle &node_handle)
    {

        sub_desired_joint_state_left_ = node_handle.subscribe("/left/desire_joint", 20, &DualArmJointPositionController::leftJointCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay());

        sub_desired_joint_state_right_ = node_handle.subscribe("/right/desire_joint", 20, &DualArmJointPositionController::rightJointCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay());

        if (!node_handle.getParam("left/arm_id", left_arm_id_))
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Could not read parameter left_arm_id_");
            return false;
        }
        std::vector<std::string> left_joint_names;
        if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7)
        {
            ROS_ERROR(
                    "DualArmJointPositionController: Invalid or no left_joint_names parameters "
                    "provided, "
                    "aborting controller init!");
            return false;
        }

        if (!node_handle.getParam("right/arm_id", right_arm_id_))
        {
            ROS_ERROR_STREAM(
                    "DualArmJointPositionController: Could not read parameter right_arm_id_");
            return false;
        }


        std::vector<std::string> right_joint_names;
        if (!node_handle.getParam("right/joint_names", right_joint_names) ||
            right_joint_names.size() != 7) {
            ROS_ERROR(
                    "DualArmJointPositionController: Invalid or no right_joint_names parameters "
                    "provided, "
                    "aborting controller init!");
            return false;
        }

        bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
        bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);


        // Get the transformation from right_O_frame to left_O_frame
        tf::StampedTransform transform;
        tf::TransformListener listener;
        try
        {
            if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                          ros::Duration(4.0)))
            {
                listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                         transform);
            }
            else
            {
                ROS_ERROR(
                        "DualArmJointPositionController: Failed to read transform from %s to %s. "
                        "Aborting init!",
                        (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
                return false;
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("DualArmJointPositionController: %s", ex.what());
            return false;
        }

        return left_success && right_success;
    }

    void DualArmJointPositionController::startingArm(FrankaDataContainerJointCommand &arm_data)
    {
        // set target point to current state
        for (size_t i = 0; i < 7; ++i) {
            arm_data.initial_pose_[i] = arm_data.position_joint_handles_[i].getPosition();
            arm_data.desired_pose_[i] = arm_data.position_joint_handles_[i].getPosition();
            arm_data.desired_pose_target_[i] = arm_data.position_joint_handles_[i].getPosition();
        }
    }
    void DualArmJointPositionController::starting(const ros::Time & /*time*/)
    {
        for (auto &arm_data : arms_data_)
        {
            startingArm(arm_data.second);
        }
     }
    void DualArmJointPositionController::update(const ros::Time & /*time*/,
                                                     const ros::Duration & /*period*/)
    {
        for (auto &arm_data : arms_data_)
        {
            updateArm(arm_data.second);
        }
    }

    void DualArmJointPositionController::updateArm(FrankaDataContainerJointCommand &arm_data)
    {
        for (size_t i = 0; i < 7; ++i)
        {
            arm_data.position_joint_handles_[i].setCommand(arm_data.desired_pose_[i]);
        }

        for (size_t i = 0; i < 7; ++i)
        {
            arm_data.desired_pose_[i] = arm_data.filter_params_ * arm_data.desired_pose_target_[i]
                    + (1.0 - arm_data.filter_params_) * arm_data.desired_pose_[i];
        }

    }

    void DualArmJointPositionController::leftJointCommandCallback(
            const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg) {
        auto &left_arm_data = arms_data_.at(left_arm_id_);
        if (msg->position.size() != 7)
        {
            ROS_ERROR_STREAM("Franka Joint Command Interface: Position values in the given msg is not 7 " << msg->position.size());
            return;
        }
        for (int i = 0; i < msg->position.size(); ++i) {
            left_arm_data.desired_pose_target_[i] = msg->position[i];
        }
    }

    void DualArmJointPositionController::rightJointCommandCallback(
            const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg){
        auto &right_arm_data = arms_data_.at(right_arm_id_);
        if (msg->position.size() != 7)
        {
            ROS_ERROR_STREAM("Franka Joint Command Interface: Position values in the given msg is not 7 " << msg->position.size());
            return;
        }
        for (int i = 0; i < msg->position.size(); ++i) {
            right_arm_data.desired_pose_target_[i] = msg->position[i];
        }
    }
}

PLUGINLIB_EXPORT_CLASS(franka_controllers::DualArmJointPositionController,
                       controller_interface::ControllerBase)