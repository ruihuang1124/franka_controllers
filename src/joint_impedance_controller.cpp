// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_controllers/joint_impedance_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_controllers {

bool JointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    std::string arm_id;

    // If the franka is install in a customized way:
    XmlRpc::XmlRpcValue customized_gravity_direction_args, tool_vector_args;
    if (!node_handle.getParam("/customized_gravity_direction", customized_gravity_direction_args)) {
        ROS_WARN_STREAM("Assuming the default robot install pose");
    } else {
        if (customized_gravity_direction_args.size() == customized_gravity_direction.size()) {
//            ROS_WARN_STREAM("Using customized gravity direction " << customized_gravity_direction_args);
            for (int i = 0; i < customized_gravity_direction_args.size(); ++i) {
                customized_gravity_direction[i] = customized_gravity_direction_args[i];
            }
            customized_install = true;
            customized_install_ = new bool(customized_install);
            customized_gravity_direction_ = new std::array<double, 3>();
            *customized_gravity_direction_ = customized_gravity_direction;
            //check if a tool is install
            if (!node_handle.getParam("/tool_mass",tool_mass) || !node_handle.getParam("/tool_vector",tool_vector_args)){
                ROS_WARN_STREAM("Assuming the default robot tool values");
                /* code */
            }else{
                if (tool_vector_args.size() == tool_vector.size()){
//                    ROS_WARN_STREAM("Using customized tool! " << tool_vector_args);
                    for (int j = 0; j < tool_vector_args.size(); ++j)
                    {
                        tool_vector[j] = tool_vector_args[j];
                    }
                    tool_install = true;
                    tool_install_ = new bool(tool_install);
                    tool_mass_ = tool_mass;
                    tool_vector_ = new std::array<double, 3>();
                    *tool_vector_ = tool_vector;
                }else{
                    ROS_ERROR_STREAM("Customized tool vector arg number "
                                             << tool_vector_args.size()
                                             << " does not match the requested number " << tool_vector.size()
                    );
                }
            }
        } else {
            ROS_ERROR_STREAM("Customized gravity direction arg number "
                                     << customized_gravity_direction_args.size()
                                     << " does not match the requested number " << customized_gravity_direction.size()
            );
        }
    }

    if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
    return false;
    }
    if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter radius, defaulting to: " << radius_);
    }
    if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void JointImpedanceController::starting(const ros::Time& /*time*/) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

void JointImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
  if (angle_ > 2 * M_PI) {
    angle_ -= 2 * M_PI;
  }

  double delta_y = radius_ * (1 - std::cos(angle_));
  double delta_z = radius_ * std::sin(angle_);

  std::array<double, 16> pose_desired = initial_pose_;
//  pose_desired[13] += delta_y;
//  pose_desired[14] += delta_z;
  cartesian_pose_handle_->setCommand(pose_desired);

  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
//  Eigen::VectorXd gravity_compensation(7);
  std::array<double, 7> gravity_compensation;

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

    //) check if customized install:
    // DO NOT FORGET THE * BEFORE customized_install_
    if (*customized_install_) {
        if (*tool_install_){
            // get the custimized gravity torque:
            std::array<double, 7> normal_gravity_array = model_handle_->getGravity(robot_state.q,tool_mass_,*tool_vector_);
            std::array<double, 7> customized_gravity_array = model_handle_->getGravity(robot_state.q,tool_mass_,*tool_vector_,customized_gravity_direction);
//            Eigen::Map<Eigen::Matrix<double, 7, 1>> ng(normal_gravity_array.data());
//            Eigen::Map<const Eigen::Matrix<double, 7, 1>> cg(customized_gravity_array.data());
            for (int i = 0; i < 7; ++i) {
                gravity_compensation[i] = customized_gravity_array[i] - normal_gravity_array[i];
            }
//            gravity_compensation = customized_gravity_array - normal_gravity_array;
            ROS_INFO_THROTTLE(2,"customized_install and tool_install!");
        }else{
            std::array<double, 7> normal_gravity_array = model_handle_->getGravity();
            std::array<double, 7> customized_gravity_array = model_handle_->getGravity(customized_gravity_direction);
            for (int i = 0; i < 7; ++i) {
                gravity_compensation[i] = customized_gravity_array[i] - normal_gravity_array[i];
            }
        }
    } else {
        for (int i = 0; i < 7; ++i) {
            gravity_compensation[i] = 0;
        }
    }
    ROS_WARN_STREAM_THROTTLE(3, gravity_compensation[0]);

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]) + gravity_compensation[i];
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> JointImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::JointImpedanceController,
                       controller_interface::ControllerBase)