#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/motion/motion_joint.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <ruckig/ruckig.hpp>
#include <utility>

namespace frankx {
using namespace movex;

template <class RobotType>
struct JointMotionGenerator : public MotionGenerator {
  ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator{RobotType::control_rate};

  ruckig::InputParameter<RobotType::degrees_of_freedoms> input_para;
  ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_para;
  ruckig::Result result;

  std::array<double, RobotType::degrees_of_freedoms> joint_positions;
  double time{0.0};
  RobotType* robot;

  JointMotion motion;
  MotionData& data;

  explicit JointMotionGenerator(RobotType* robot, JointMotion motion, MotionData& data)
      : robot(robot), motion(std::move(motion)), data(data) {}

  void init(const franka::RobotState& robot_state, franka::Duration period) {
    input_para.current_position = robot_state.q_d;
    input_para.current_velocity = toStd(Vector7d::Zero());
    input_para.current_acceleration = toStd(Vector7d::Zero());

    input_para.target_position = toStd(motion.target);
    input_para.target_velocity = toStd(Vector7d::Zero());
    input_para.target_acceleration = toStd(Vector7d::Zero());

    for (size_t dof = 0; dof < RobotType::degrees_of_freedoms; dof += 1) {
      input_para.max_velocity[dof] = RobotType::max_joint_velocity[dof] * robot->velocity_rel * data.velocity_rel;
      input_para.max_acceleration[dof] =
          0.3 * RobotType::max_joint_acceleration[dof] * robot->acceleration_rel * data.acceleration_rel;
      input_para.max_jerk[dof] = 0.3 * RobotType::max_joint_jerk[dof] * robot->jerk_rel * data.jerk_rel;
    }
  }

  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time += period.toSec();
    if (time == 0.0) {
      init(robot_state, period);
    }

    const int steps = std::max<int>(period.toMSec(), 1);
    for (int i = 0; i < steps; i++) {
      result = trajectory_generator.update(input_para, output_para);
      joint_positions = output_para.new_position;

      if (result == ruckig::Result::Finished) {
        joint_positions = input_para.target_position;
        return franka::MotionFinished(franka::JointPositions(joint_positions));

      } else if (result == ruckig::Result::Error) {
        std::cout << "[frankx robot] Invalid inputs:" << std::endl;
        return franka::MotionFinished(franka::JointPositions(joint_positions));
      }

      input_para.current_position = output_para.new_position;
      input_para.current_velocity = output_para.new_velocity;
      input_para.current_acceleration = output_para.new_acceleration;
    }

    return franka::JointPositions(joint_positions);
  }
};

template <class RobotType>
struct JointVelocityMotionGenerator : public MotionGenerator {
  ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator{RobotType::control_rate};

  ruckig::InputParameter<RobotType::degrees_of_freedoms> input_param;
  ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_param;
  ruckig::Result result_;

  std::array<double, RobotType::degrees_of_freedoms> joint_position_;
  std::array<double, RobotType::degrees_of_freedoms> joint_velocity_;
  double time_{0.0};
  RobotType* robot_;

  JointVelocityMotion motion_;
  MotionData& data_;

  explicit JointVelocityMotionGenerator(RobotType* robot, JointVelocityMotion motion, MotionData& data)
      : robot_(robot), motion_(std::move(motion)), data_(data), result_(ruckig::Result::Working) {}

  void init(const franka::RobotState& robot_state, franka::Duration period) {
    input_param.current_position = robot_state.q_d;
    input_param.current_velocity = robot_state.dq_d;
    input_param.current_acceleration = toStd(Vector7d::Zero());

    input_param.target_position = toStd(motion_.target_position_);
    input_param.target_velocity = toStd(motion_.target_velocity_);
    input_param.target_acceleration = toStd(Vector7d::Zero());

    for (size_t dof = 0; dof < RobotType::degrees_of_freedoms; dof += 1) {
      input_param.max_velocity[dof] = RobotType::max_joint_velocity[dof] * robot_->velocity_rel * data_.velocity_rel;
      input_param.max_acceleration[dof] =
          0.3 * RobotType::max_joint_acceleration[dof] * robot_->acceleration_rel * data_.acceleration_rel;
      input_param.max_jerk[dof] = 0.3 * RobotType::max_joint_jerk[dof] * robot_->jerk_rel * data_.jerk_rel;
    }
  }

  franka::JointVelocities operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time_ += period.toSec();
    if (time_ == 0.0) {
      init(robot_state, period);
    }

    const int steps = std::max<int>(period.toMSec(), 1);
    for (int i = 0; i < steps; i++) {
      result_ = trajectory_generator.update(input_param, output_param);
      joint_position_ = output_param.new_position;
      joint_velocity_ = output_param.new_velocity;

      if (result_ == ruckig::Result::Finished) {
        joint_velocity_ = input_param.target_velocity;
        return franka::MotionFinished(franka::JointVelocities(joint_velocity_));
      } else if (result_ == ruckig::Result::Error) {
        std::cout << "[frankx robot] Invalid inputs:" << std::endl;
        return franka::MotionFinished(franka::JointVelocities(joint_velocity_));
      }

      input_param.current_position = output_param.new_position;
      input_param.current_velocity = output_param.new_velocity;
      input_param.current_acceleration = output_param.new_acceleration;
    }
    return franka::JointVelocities(joint_velocity_);
  }
};

}  // namespace frankx
