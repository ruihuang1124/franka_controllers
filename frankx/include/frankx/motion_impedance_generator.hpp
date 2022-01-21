#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/motion/motion_impedance.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <utility>

namespace frankx {
using namespace movex;

template <class RobotType>
struct ImpedanceMotionGenerator : public MotionGenerator {
  double time{0.0}, motion_init_time{0.0};
  RobotType* robot;

  Eigen::Matrix<double, 6, 6> stiffness, damping;
  Affine initial_affine;
  Eigen::Vector3d position_d;
  Eigen::Quaterniond orientation_d;

  franka::RobotState initial_state;
  franka::Model* model_;

  Affine frame_;
  ImpedanceMotion& motion_;
  MotionData& data;

  explicit ImpedanceMotionGenerator(RobotType* robot, Affine frame, ImpedanceMotion& motion, MotionData& data)
      : robot(robot), frame_(std::move(frame)), motion_(motion), data(data) {
    if (motion.type == ImpedanceMotion::Type::Joint) {
      throw std::runtime_error("joint impedance is not implemented yet.");
    }

    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << motion.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << motion.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(motion.translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(motion.rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    initial_state = robot->readOnce();
    model_ = new franka::Model(robot->loadModel());

    initial_affine = Affine(initial_state.O_T_EE);
    position_d = Eigen::Vector3d(initial_affine.translation());
    orientation_d = Eigen::Quaterniond(initial_affine.quaternion());
  }

  explicit ImpedanceMotionGenerator(RobotType* robot, ImpedanceMotion& motion, MotionData& data)
      : robot(robot), motion_(motion), data(data) {
    if (motion.type == ImpedanceMotion::Type::Joint) {
      throw std::runtime_error("joint impedance is not implemented yet.");
    }

    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << motion.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << motion.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(motion.translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(motion.rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    initial_state = robot->readOnce();
    model_ = new franka::Model(robot->loadModel());

    initial_affine = Affine(initial_state.O_T_EE);
    position_d = Eigen::Vector3d(initial_affine.translation());
    orientation_d = Eigen::Quaterniond(initial_affine.quaternion());
  }

  void init(const franka::RobotState& robot_state, franka::Duration period) {
    // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    // motion_.target = Affine(transform);
    motion_.is_active = true;
  }

  franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time += period.toSec();
    if (time == 0.0) {
      init(robot_state, period);
    }

    std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
    std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }

    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -transform.linear() * error.tail(3);

    Eigen::VectorXd wrench_cartesian(6), tau_task(7), tau_d(7);
    wrench_cartesian = -stiffness * error - damping * (jacobian * dq);

    // Force constraints
    for (const auto force_constraint : motion_.force_constraints) {
      switch (force_constraint.first) {
        case ImpedanceMotion::Axis::X: {
          wrench_cartesian(0) = force_constraint.second;
        } break;
        case ImpedanceMotion::Axis::Y: {
          wrench_cartesian(1) = force_constraint.second;
        } break;
        case ImpedanceMotion::Axis::Z: {
          wrench_cartesian(2) = force_constraint.second;
        } break;
      }
    }

    tau_task << jacobian.transpose() * wrench_cartesian;
    tau_d << tau_task + coriolis;

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    if (motion_.should_finish) {
      motion_.is_active = false;
      return franka::MotionFinished(franka::Torques(tau_d_array));
    }

    // Update target with target motion
    switch (motion_.target_motion) {
      case ImpedanceMotion::TargetMotion::Exponential: {
        position_d = motion_.exponential_decay * motion_.target.translation() + (1.0 - motion_.exponential_decay) * position_d;
        orientation_d = orientation_d.slerp(motion_.exponential_decay, motion_.target.quaternion());
      } break;
      case ImpedanceMotion::TargetMotion::Linear: {
        if (!motion_.linear_motion.initialized) {
          initial_affine = Affine(Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())));
          motion_init_time = time;
          motion_.linear_motion.initialized = true;
        }
        double transition_parameter = (time - motion_init_time) / (motion_.linear_motion.duration);
        if (transition_parameter <= 1.0) {  // [ms] to [s]
          motion_.target = initial_affine.slerp(motion_.linear_motion.relative_target * initial_affine, transition_parameter);
          position_d = motion_.target.translation();
          orientation_d = motion_.target.quaternion();
        } else if (motion_.linear_motion.finish_after &&
                   transition_parameter > motion_.linear_motion.finish_wait_factor) {  // Wait a bit longer to stop
          motion_.should_finish = true;
        }
      } break;
      case ImpedanceMotion::TargetMotion::Spiral: {
        if (!motion_.spiral_motion.initialized) {
          motion_init_time = time;
          motion_.spiral_motion.initialized = true;
        }

        double time_diff = motion_init_time - time;
        // target = spiral_motion.center * Affine(0, 0, 0, 2 * pi * spiral_motion.revolutions_per_second * time) *
        // Affine(spiral_motion.radius_increment_per_revolution * time); position_d = target.translation(); orientation_d =
        // target.quaternion();
      } break;
    }

    return franka::Torques(tau_d_array);
  }
};

}  // namespace frankx
