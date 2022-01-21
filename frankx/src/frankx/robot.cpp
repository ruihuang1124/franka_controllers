#include <frankx/robot.hpp>
#include <ruckig/ruckig.hpp>
#include <utility>

namespace frankx {

Robot::Robot(std::string fci_ip, double dynamic_rel, bool repeat_on_error, bool stop_at_python_signal)
    : franka::Robot(fci_ip),
      fci_ip(fci_ip),
      velocity_rel(dynamic_rel),
      acceleration_rel(dynamic_rel),
      jerk_rel(dynamic_rel),
      repeat_on_error(repeat_on_error),
      stop_at_python_signal(stop_at_python_signal) {}

void Robot::setDefaultBehavior() { setBehavior(100., 80., 20.); }

void Robot::setBehavior(double major_torque_limit, double minor_torque_limit, double force_limit) {
  std::array<double, 7> torque_limits = {major_torque_limit, major_torque_limit, major_torque_limit, major_torque_limit,
                                         minor_torque_limit, minor_torque_limit, minor_torque_limit};
  std::array<double, 6> force_limits = {force_limit, force_limit, force_limit, force_limit, force_limit, force_limit};
  setCollisionBehavior(torque_limits, torque_limits, torque_limits, torque_limits, force_limits, force_limits, force_limits,
                       force_limits);

  setEE({1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1});
}

void Robot::setDynamicRel(double dynamic_rel) {
  velocity_rel = dynamic_rel;
  acceleration_rel = dynamic_rel;
  jerk_rel = dynamic_rel;
}

bool Robot::hasErrors() { return bool(readOnce().current_errors); }

bool Robot::recoverFromErrors() {
  automaticErrorRecovery();
  return !hasErrors();
}

Affine Robot::currentPose(const Affine& frame) {
  auto state = readOnce();
  return Affine(state.O_T_EE) * frame;
}

std::array<double, 7> Robot::currentJointPositions() {
  auto state = readOnce();
  return state.q;
}

Affine Robot::forwardKinematics(const std::array<double, 7>& q) {
  const Eigen::Matrix<double, 7, 1> q_current = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data(), q.size());
  return Affine(Kinematics::forward(q_current));
}

std::array<double, 7> Robot::inverseKinematics(const Affine& target, const std::array<double, 7>& q0) {
  std::array<double, 7> result;

  Eigen::Matrix<double, 6, 1> x_target = target.vector();
  const Eigen::Matrix<double, 7, 1> q0_current = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q0.data(), q0.size());

  Eigen::Matrix<double, 7, 1>::Map(result.data()) = Kinematics::inverse(x_target, q0_current);
  return result;
}

bool Robot::move(ImpedanceMotion& motion) { return move(Affine(), motion); }

bool Robot::move(ImpedanceMotion& motion, MotionData& data) { return move(Affine(), motion, data); }

bool Robot::move(const Affine& frame, ImpedanceMotion& motion) {
  auto data = MotionData();
  return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, ImpedanceMotion& motion, MotionData& data) {
  ImpedanceMotionGenerator<Robot> mg{this, frame, motion, data};

  try {
    control(stateful<franka::Torques>(mg));
    motion.is_active = false;

  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    motion.is_active = false;
    return false;
  }
  return true;
}

bool Robot::move(ImpedanceMotion& i_motion, JointMotion p_motion) {
  auto data = MotionData();
  return move(std::move(p_motion), i_motion, data);
}

bool Robot::move(ImpedanceMotion& i_motion, JointVelocityMotion v_motion) {
  auto data = MotionData();
  return move(std::move(v_motion), i_motion, data);
}

bool Robot::move(JointMotion p_motion, ImpedanceMotion& i_motion, MotionData& data) {
  JointMotionGenerator<Robot> pmg{this, std::move(p_motion), data};
  ImpedanceMotionGenerator<Robot> img{this, i_motion, data};

  try {
    control(img, pmg);
    i_motion.is_active = false;
  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    i_motion.is_active = false;
    return false;
  }
  return true;
}

bool Robot::move(JointVelocityMotion v_motion, ImpedanceMotion& i_motion, MotionData& data) {
  JointVelocityMotionGenerator<Robot> vmg{this, std::move(v_motion), data};
  ImpedanceMotionGenerator<Robot> img{this, i_motion, data};

  try {
    control(img, vmg);
    i_motion.is_active = false;

  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    i_motion.is_active = false;
    return false;
  }
  return true;
}

bool Robot::move(JointVelocityMotion motion) {
  auto data = MotionData();
  return move(std::move(motion), data);
}

bool Robot::move(JointVelocityMotion motion, MotionData& data) {
  JointVelocityMotionGenerator<Robot> mg{this, std::move(motion), data};
  try {
    control(stateful<franka::JointVelocities>(mg), franka::ControllerMode::kJointImpedance, false, 1000);
  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    return false;
  }
  return true;
}

bool Robot::move(JointMotion motion) {
  auto data = MotionData();
  return move(std::move(motion), data);
}

bool Robot::move(JointMotion motion, MotionData& data) {
  JointMotionGenerator<Robot> mg{this, std::move(motion), data};
  try {
    control(stateful<franka::JointPositions>(mg));
  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    return false;
  }
  return true;
}

bool Robot::move(PathMotion motion) { return move(Affine(), std::move(motion)); }

bool Robot::move(PathMotion motion, MotionData& data) { return move(Affine(), std::move(motion), data); }

bool Robot::move(const Affine& frame, PathMotion motion) {
  auto data = MotionData();
  return move(frame, std::move(motion), data);
}

bool Robot::move(const Affine& frame, PathMotion motion, MotionData& data) {
  PathMotionGenerator<Robot> mg{this, frame, std::move(motion), data};

  try {
    control(stateful<franka::CartesianPose>(mg), controller_mode);

  } catch (const franka::Exception& exception) {
    std::cout << exception.what() << std::endl;
    return false;
  }
  return true;
}

bool Robot::move(WaypointMotion& motion) { return move(Affine(), motion); }

bool Robot::move(WaypointMotion& motion, MotionData& data) { return move(Affine(), motion, data); }

bool Robot::move(const Affine& frame, WaypointMotion& motion) {
  auto data = MotionData();
  return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion& motion, MotionData& data) {
  WaypointMotionGenerator<Robot> mg{this, frame, motion, data};
  mg.input_para.target_position[0] = 0.01;

  try {
    control(stateful<franka::CartesianPose>(mg), controller_mode);

  } catch (const franka::Exception& exception) {
    auto errors = readOnce().last_motion_errors;
    if (repeat_on_error
        // && (errors.cartesian_motion_generator_joint_acceleration_discontinuity
        // || errors.cartesian_motion_generator_joint_velocity_discontinuity
        // || errors.cartesian_motion_generator_velocity_discontinuity
        // || errors.cartesian_motion_generator_acceleration_discontinuity)
    ) {
      std::cout << "[frankx robot] continue motion after exception: " << exception.what() << std::endl;
      automaticErrorRecovery();

      data.velocity_rel *= 0.5;
      data.acceleration_rel *= 0.5;
      data.jerk_rel *= 0.5;
      mg.reset();

      bool success{false};

      try {
        control(stateful<franka::CartesianPose>(mg), controller_mode);
        success = true;

      } catch (const franka::Exception& exception) {
        std::cout << exception.what() << std::endl;
      }
      data.velocity_rel *= 2;
      data.acceleration_rel *= 2;
      data.jerk_rel *= 2;

      return success;

    } else {
      std::cout << exception.what() << std::endl;
    }

    return false;
  }
  return true;
}

}  // namespace frankx
