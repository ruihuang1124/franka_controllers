#include "franka_controllers/cartesian_impedance_controller.h"
#include "ros/ros.h"
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "franka_controllers/pseudo_inversion.h"
#include <tf/tf.h>

namespace franka_controllers {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  //if the franka is install in a customized way:
  XmlRpc::XmlRpcValue customized_gravity_direction_args, tool_vector_args;
  if (!node_handle.getParam("/customized_gravity_direction", customized_gravity_direction_args)) {
      ROS_WARN_STREAM("Assuming the default robot install pose");
  } else {
      if (customized_gravity_direction_args.size() == customized_gravity_direction.size()) {
        ROS_WARN_STREAM("Using customized gravity direction " << customized_gravity_direction_args);
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
            ROS_WARN_STREAM("Using customized tool! " << tool_vector_args);
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

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_joint_state_ = node_handle.subscribe("desire_joint", 20, &CartesianImpedanceController::jointCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  // print the current position and orientation under world frame
  ROS_INFO_THROTTLE(2,"Current positon is: x:%.4f, y:%.4f, z:%.4f, QX:%.4f, QY:%.4f, QZ:%.4f, QW:%.4f", position(0),position(1),position(2),orientation.x(),orientation.y(),orientation.z(),orientation.w());

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7),gravity_compensation(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  
  //) check if customized install:
  // DO NOT FORGET THE * BEFORE customized_install_
  if (*customized_install_) {
    if (*tool_install_){
      // get the custimized gravity torque:
      std::array<double, 7> normal_gravity_array = model_handle_->getGravity(robot_state.q,tool_mass_,*tool_vector_);
      std::array<double, 7> customized_gravity_array = model_handle_->getGravity(robot_state.q,tool_mass_,*tool_vector_,customized_gravity_direction);
      Eigen::Map<Eigen::Matrix<double, 7, 1>> ng(normal_gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> cg(customized_gravity_array.data());
      gravity_compensation = cg - ng;
      ROS_INFO_THROTTLE(2,"customized_install and tool_install!");
      }else{
      std::array<double, 7> normal_gravity_array = model_handle_->getGravity();
      std::array<double, 7> customized_gravity_array = model_handle_->getGravity(customized_gravity_direction);
      Eigen::Map<Eigen::Matrix<double, 7, 1>> ng(normal_gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> cg(customized_gravity_array.data());
      gravity_compensation = cg - ng;}
  } else {
      gravity_compensation.setZero();
  }
  ROS_WARN_STREAM_THROTTLE(3, gravity_compensation);

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + gravity_compensation;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::complianceParamCallback(
    franka_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  //kEndEffector
  ROS_ERROR("receive target equilibriumPose");
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianImpedanceController::jointCommandCallback(
    const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg)
{
  std::array<double, 7> joint_position_cmd{};
  std::array<double, 7> joint_velocity_cmd{};
  if (msg->position.size() != 7)
  {
    ROS_ERROR_STREAM("Franka Joint Command Interface: Position values in the given msg is not 7 " << msg->position.size());
    return;
  }
  bool have_vel = (msg->velocity.size() == 7);
  for (size_t i = 0; i < msg->position.size(); ++i)
  {
    q_d_nullspace_(i) = msg->position[i];
    joint_position_cmd[i] = msg->position[i];
    if (have_vel)
    {
      joint_velocity_cmd[i] = msg->velocity[i];
    }
    else
    {
      joint_velocity_cmd[i] = 0.;
    }
  }

  std::array<double, 16> F_T_EE = {0.7071,0.7071,0,0,-0.7071,0.7071,0,0,0,0,1,0.1034,0,0,0,1};  // NOLINT(readability-identifier-naming)
  std::array<double, 16> EE_T_K = {0};  // NOLINT(readability-identifier-naming)
  // F_T_EE.zero();
  std::array<double, 16> EEPose = model_handle_->getPose(franka::Frame::kEndEffector, joint_position_cmd, F_T_EE, EE_T_K);
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << EEPose[3], EEPose[7], EEPose[11];

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  Eigen::Matrix<double, 3, 3> rotationMatrix;
  rotationMatrix << EEPose[0], EEPose[1], EEPose[2], EEPose[4], EEPose[5], EEPose[6], EEPose[8], EEPose[9], EEPose[10];
  Eigen::Quaterniond new_orientation_target(rotationMatrix);
  orientation_d_target_.coeffs() << new_orientation_target.coeffs().transpose().x(), new_orientation_target.coeffs().transpose().y(),
      new_orientation_target.coeffs().transpose().z(), new_orientation_target.coeffs().transpose().w();
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
  {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerBase)