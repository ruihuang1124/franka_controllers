//
// Created by Ray in 2021/12/06
//
#include "franka_controllers/dual_arm_cartesian_impedance_controller.h"

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include "franka_controllers/pseudo_inversion.h"
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <utility>

namespace franka_controllers
{

    bool DualArmCartesianImpedanceController::initArm(
        hardware_interface::RobotHW *robot_hw,
        const std::string &arm_id,
        const std::vector<std::string> &joint_names, std::array<double, 3> customize_gravity_direction,
        double tool_mass,
        std::array<double, 3> tool_vector)
    {
        FrankaDataContainer arm_data;
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "DualArmCartesianImpedanceController: Error getting model interface from hardware");
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
                "DualArmCartesianImpedanceController: Exception getting model handle from "
                "interface: "
                << ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "DualArmCartesianImpedanceController: Error getting state interface from hardware");
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
                "DualArmCartesianImpedanceController: Exception getting state handle from "
                "interface: "
                << ex.what());
            return false;
        }

        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "DualArmCartesianImpedanceController: Error getting effort joint interface from "
                "hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM(
                    "DualArmCartesianImpedanceController: Exception getting joint handles: "
                    << ex.what());
                return false;
            }
        }

        arm_data.position_d_.setZero();
        arm_data.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        arm_data.position_d_target_.setZero();
        arm_data.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        arm_data.cartesian_stiffness_.setZero();
        arm_data.cartesian_damping_.setZero();

        arm_data.k_gains_.setZero();
        arm_data.d_gains_.setZero();
        arm_data.tool_mass_ = tool_mass;
        arm_data.tool_vector_ = tool_vector;
        arm_data.tool_vector_pointer_ = new std::array<double, 3>();
        *arm_data.tool_vector_pointer_ = arm_data.tool_vector_;
        arm_data.customize_gravity_direction_ = customize_gravity_direction;
        arm_data.impedance_motion_.is_active = true;


        arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));

        return true;
    }

    bool DualArmCartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw,
                                                          ros::NodeHandle &node_handle)
    {
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        sub_desired_joint_state_left_ = node_handle.subscribe("/panda_left/joint_commands", 20, &DualArmCartesianImpedanceController::leftJointCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay());

        sub_desired_joint_state_right_ = node_handle.subscribe("/panda_right/joint_commands", 20, &DualArmCartesianImpedanceController::rightJointCommandCallback, this, ros::TransportHints().reliable().tcpNoDelay());

        sub_desired_pose_left_ = node_handle.subscribe("/left/desire_pose", 20, &DualArmCartesianImpedanceController::leftPoseCommandCallback,
                                                 this, ros::TransportHints().reliable().tcpNoDelay());

        sub_desired_pose_right_ =
        node_handle.subscribe("/right/desire_pose", 20, &DualArmCartesianImpedanceController::rightPoseCommandCallback, this,
                            ros::TransportHints().reliable().tcpNoDelay());
        
        if (!node_handle.getParam("left/arm_id", left_arm_id_))
        {
            ROS_ERROR_STREAM(
                "DualArmCartesianImpedanceController: Could not read parameter left_arm_id_");
            return false;
        }
        std::vector<std::string> left_joint_names;
        if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7)
        {
            ROS_ERROR(
                "DualArmCartesianImpedanceController: Invalid or no left_joint_names parameters "
                "provided, "
                "aborting controller init!");
            return false;
        }

        if (!node_handle.getParam("right/arm_id", right_arm_id_))
        {
            ROS_ERROR_STREAM(
                "DualArmCartesianImpedanceController: Could not read parameter right_arm_id_");
            return false;
        }

        boost::function<void(const geometry_msgs::PoseStamped::ConstPtr &)> callback =
            boost::bind(&DualArmCartesianImpedanceController::targetPoseCallback, this, _1);

        ros::SubscribeOptions subscribe_options;
        subscribe_options.init("centering_frame_target_pose", 1, callback);
        subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
        sub_target_pose_left_ = node_handle.subscribe(subscribe_options);

        std::vector<std::string> right_joint_names;
        if (!node_handle.getParam("right/joint_names", right_joint_names) ||
            right_joint_names.size() != 7)
        {
            ROS_ERROR(
                "DualArmCartesianImpedanceController: Invalid or no right_joint_names parameters "
                "provided, "
                "aborting controller init!");
            return false;
        }
        upper_joint_position_warning_limits_ << 2.7, 1.6, 2.7, 0, 2.7, 3.6, 2.7;
        lower_joint_position_warning_limits_ << -2.7,-1.6, -2.7, -2.9, -2.7, 0, -2.7;


        //TODO read these paramaters from rosparam:
        std::array<double, 3> left_customize_gravity_direction = {7.5439190864562988,6.2554783821105957,-0.4408954679965973};
        std::array<double, 3> right_customize_gravity_direction = {7.5439190864562988,-6.2554783821105957,-0.4408954679965973};

        double left_tool_mass = 0.73, right_tool_mass = 0.73;
        std::array<double, 3> left_tool_vector = {-0.01, 0.0, 0.03};
        std::array<double, 3> right_tool_vector = {-0.01, 0.0, 0.03};

        bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names, left_customize_gravity_direction, left_tool_mass, left_tool_vector);
        bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names, right_customize_gravity_direction, right_tool_mass, right_tool_vector);

        dynamic_reconfigure_compliance_param_node_ =
            ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<
            franka_controllers::dual_arm_compliance_paramConfig>>(
            dynamic_reconfigure_compliance_param_node_);

        dynamic_server_compliance_param_->setCallback(boost::bind(
            &DualArmCartesianImpedanceController::complianceParamCallback, this, _1, _2));

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
                    "DualArmCartesianImpedanceController: Failed to read transform from %s to %s. "
                    "Aborting init!",
                    (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
                return false;
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("DualArmCartesianImpedanceController: %s", ex.what());
            return false;
        }
        tf::transformTFToEigen(transform, Ol_T_Or_); // NOLINT (readability-identifier-naming)

        // Setup publisher for the centering frame.
        publish_rate_ = franka_hw::TriggerRate(10000.0);
        center_frame_pub_.init(node_handle, "centering_frame", 1, true);
        // ROS_ERROR("something wrong  after iniArm");

        //Initialize service to achieve curi stop control
        switchController_ = node_handle.advertiseService("/dual_panda/switch_controller",
                                                         &DualArmCartesianImpedanceController::executeswitchControllerCb, this);
        ROS_INFO_STREAM("Advertising service /dual_panda/switch_controller");

        //Initialize service to achieve curi arm swith function
        movetoHome_ = node_handle.advertiseService("/dual_panda/movetoHome", &DualArmCartesianImpedanceController::executemovetoHomeCB, this);
        ROS_INFO_STREAM("Advertising service /dual_panda/movetoHome");

        movetoTarget_left_ = node_handle.advertiseService("/dual_panda/left/movetoTarget", &DualArmCartesianImpedanceController::executemovetoTargetLeftCB, this);
        ROS_INFO_STREAM("Advertising service /dual_panda/left/movetoTarget");

        movetoTarget_right_ = node_handle.advertiseService("/dual_panda/right/movetoTarget", &DualArmCartesianImpedanceController::executemovetoTargetRightCB, this);
        ROS_INFO_STREAM("Advertising service /dual_panda/left/movetoTarget");
        controller_type_ = 1;
        left_createTrajectoryclient_ = node_handle.serviceClient<franka_controllers::createTrajectory>("/panda_left/create_trajectory");
        right_createTrajectoryclient_ = node_handle.serviceClient<franka_controllers::createTrajectory>("/panda_right/create_trajectory");
        return left_success && right_success;
    }

    void DualArmCartesianImpedanceController::startingArm(FrankaDataContainer &arm_data)
    {
        // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
        // to initial configuration
        franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
        // get jacobian
        std::array<double, 42> jacobian_array =
            arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        // convert to eigen
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

        // set target point to current state
        arm_data.position_d_ = initial_transform.translation();
        arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
        arm_data.position_d_target_ = initial_transform.translation();
        arm_data.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

        // set nullspace target configuration to initial q
        arm_data.q_d_nullspace_ = q_initial;
        arm_data.dq_d_nullspace_.setZero();
        arm_data.q_d_nullspace_target_ = q_initial;
    }

    void DualArmCartesianImpedanceController::starting(const ros::Time & /*time*/)
    {
        for (auto &arm_data : arms_data_)
        {
            startingArm(arm_data.second);
        }
        franka::RobotState robot_state_right =
            arms_data_.at(right_arm_id_).state_handle_->getRobotState();
        franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
        Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map( // NOLINT (readability-identifier-naming)
            robot_state_left.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
        Eigen::Affine3d Or_T_EEr(Eigen::Matrix4d::Map( // NOLINT (readability-identifier-naming)
            robot_state_right.O_T_EE.data()));         // NOLINT (readability-identifier-naming)
        EEr_T_EEl_ =
            Or_T_EEr.inverse() * Ol_T_Or_.inverse() * Ol_T_EEl; // NOLINT (readability-identifier-naming)
        EEl_T_C_.setIdentity();
        Eigen::Vector3d EEr_r_EEr_EEl = // NOLINT (readability-identifier-naming)
            EEr_T_EEl_.translation();   // NOLINT (readability-identifier-naming)
        EEl_T_C_.translation() = -0.5 * EEr_T_EEl_.inverse().rotation() * EEr_r_EEr_EEl;
    }

    void DualArmCartesianImpedanceController::update(const ros::Time & /*time*/,
                                                            const ros::Duration & /*period*/)
    {
        for (auto &arm_data : arms_data_)
        {
            updateArm(arm_data.second);
        }
        if (publish_rate_())
        {
            publishCenteringPose();
        }
    }

    void DualArmCartesianImpedanceController::updateArm(FrankaDataContainer &arm_data)
    {
        // get state variables
        franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
        std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
        std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array =
            arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        // convert to Eigen
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
            robot_state.tau_J_d.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        // compute error to desired pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - arm_data.position_d_;

        // orientation error
        if (arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation * arm_data.orientation_d_.inverse());
        // convert to axis angle
        Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
        // compute "orientation error"
        error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

        // compute control
        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), gravity_compensation(7), tau_joint_impedance(7);

        // pseudoinverse for nullspace handling
        // kinematic pseuoinverse
        Eigen::MatrixXd jacobian_transpose_pinv;
        franka_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        // Cartesian PD control with damping ratio = 1
        tau_task << jacobian.transpose() * (-arm_data.cartesian_stiffness_ * error -
                                            arm_data.cartesian_damping_ * (jacobian * dq));
        // nullspace PD control with damping ratio = 1
//        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
//                          jacobian.transpose() * jacobian_transpose_pinv) *
//                             (arm_data.nullspace_stiffness_ * (arm_data.q_d_nullspace_ - q) -
//                              (2.0 * sqrt(arm_data.nullspace_stiffness_)) * dq);

        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                          jacobian.transpose() * jacobian_transpose_pinv) *
                         (arm_data.k_gains_ * (arm_data.q_d_nullspace_ - q) -
                          arm_data.d_gains_ * dq);

        // get the customized gravity torque:

        std::array<double, 7> normal_gravity_array = arm_data.model_handle_->getGravity(robot_state.q, arm_data.tool_mass_, *arm_data.tool_vector_pointer_);
        std::array<double, 7> customized_gravity_array = arm_data.model_handle_->getGravity(
            robot_state.q, arm_data.tool_mass_, *arm_data.tool_vector_pointer_, arm_data.customize_gravity_direction_);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> ng(normal_gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> cg(customized_gravity_array.data());
        gravity_compensation = cg - ng;

//         if (arm_data.customize_gravity_direction_[1]<0)
//         {
//             for (int i = 0; i < 7; i++) {
//                 if (robot_state.q[i] < lower_joint_position_warning_limits_(i) || robot_state.q[i] > upper_joint_position_warning_limits_(i)){
//                     ROS_ERROR_THROTTLE(1,"Joint %d of right arm is about to reach the limit!!",i);
//                 }
//             }
//             ROS_INFO_THROTTLE(2, "Current Joint states of right arm are: Joint1:%.4f, Joint2:%.4f, Joint3:%.4f, Joint4:%.4f, "
//                                  "Joint5:%.4f, Joint6:%.4f, Joint7:%.4f",q[0],q[1],q[2],q[3],q[4],q[5],q[6]);
// //            ROS_INFO_THROTTLE(2,
// //                              "Current end-effector for position and orientation for Right arm under world frame is: x:%.4f, y:%.4f, z:%.4f, QX:%.4f, "
// //                              "QY:%.4f, QZ:%.4f, QW:%.4f",
// //                              position(0), position(1), position(2),
// //                              orientation.x(), orientation.y(), orientation.z(),
// //                              orientation.w());
// //            ROS_INFO_THROTTLE(2,
// //                              "The target ee pose is set as: x:%.4f, y:%.4f, z:%.4f, QX:%.4f, "
// //                              "QY:%.4f, QZ:%.4f, QW:%.4f",
// //                              arm_data.position_d_target_(0), arm_data.position_d_target_(1), arm_data.position_d_target_(2),
// //                              arm_data.orientation_d_target_.x(), arm_data.orientation_d_target_.y(), arm_data.orientation_d_target_.z(),
// //                              arm_data.orientation_d_target_.w());
//         } else{
//             for (int i = 0; i < 7; i++) {
//                 if (robot_state.q[i] < lower_joint_position_warning_limits_(i) || robot_state.q[i] > upper_joint_position_warning_limits_(i)){
//                     ROS_ERROR_THROTTLE(1,"Joint %d of right arm is about to reach the limit!!",i);
//                 }
//             }
//             ROS_INFO_THROTTLE(2, "Current Joint states of Left arm are: Joint1:%.4f, Joint2:%.4f, Joint3:%.4f, Joint4:%.4f, "
//                                  "Joint5:%.4f, Joint6:%.4f, Joint7:%.4f",q[0],q[1],q[2],q[3],q[4],q[5],q[6]);
//         }



//        ROS_WARN_STREAM_THROTTLE(3, gravity_compensation);
        // Desired torque
//        std::array<double, 7> tau_d_calculated;
//        for (size_t i = 0; i < 7; ++i) {
//            tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
//                                  k_gains_target_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
//                                  d_gains_target_[i] * (robot_state.dq_d[i] - dq_filtered_[i]) + gravity_compensation[i];
//        }
        arm_data.dq_d_nullspace_ = (1-alpha) * arm_data.dq_d_nullspace_ + alpha*dq_d;

//        tau_d << tau_task + tau_nullspace + coriolis + gravity_compensation;
//        tau_joint_impedance << coriolis + arm_data.k_gains_*(arm_data.q_d_nullspace_ - q) + arm_data.d_gains_*(dq_d-arm_data.dq_d_nullspace_)+gravity_compensation;
        // Saturate torque rate to avoid discontinuities

        if (controller_type_ == 1)
        {
            tau_d << coriolis + arm_data.k_gains_*(arm_data.q_d_nullspace_ - q) + arm_data.d_gains_*(dq_d-arm_data.dq_d_nullspace_)+gravity_compensation;
            ROS_INFO_THROTTLE(3,"Joint impedance controller!");
        }else{
            tau_d << tau_task + tau_nullspace + coriolis + gravity_compensation;
            ROS_INFO_THROTTLE(3,"Cartesian impedance controller!");
        }
        tau_d << saturateTorqueRate(arm_data, tau_d, tau_J_d);
        for (size_t i = 0; i < 7; ++i)
        {
            arm_data.joint_handles_[i].setCommand(tau_d(i));
        }

        // update parameters changed online either through dynamic reconfigure or through the interactive
        // target by filtering
        arm_data.cartesian_stiffness_ = arm_data.filter_params_ * arm_data.cartesian_stiffness_target_ +
                                        (1.0 - arm_data.filter_params_) * arm_data.cartesian_stiffness_;
        arm_data.cartesian_damping_ = arm_data.filter_params_ * arm_data.cartesian_damping_target_ +
                                      (1.0 - arm_data.filter_params_) * arm_data.cartesian_damping_;
        arm_data.nullspace_stiffness_ = arm_data.filter_params_ * arm_data.nullspace_stiffness_target_ +
                                        (1.0 - arm_data.filter_params_) * arm_data.nullspace_stiffness_;
        arm_data.position_d_ = arm_data.filter_params_ * arm_data.position_d_target_ +
                               (1.0 - arm_data.filter_params_) * arm_data.position_d_;
        arm_data.orientation_d_ =
            arm_data.orientation_d_.slerp(arm_data.filter_params_, arm_data.orientation_d_target_);
        
        arm_data.q_d_nullspace_ = arm_data.filter_params_ * arm_data.q_d_nullspace_target_ +
                               (1.0 - arm_data.filter_params_) * arm_data.q_d_nullspace_;


        arm_data.k_gains_ = arm_data.filter_params_ * arm_data.k_gains_target_ +
                                        (1.0 - arm_data.filter_params_) * arm_data.k_gains_target_;
        arm_data.d_gains_ = arm_data.filter_params_ * arm_data.d_gains_target_ +
                            (1.0 - arm_data.filter_params_) * arm_data.d_gains_target_;


        // ROS_WARN_STREAM_THROTTLE(3, arm_data.position_d_target_);
        
    }

    Eigen::Matrix<double, 7, 1> DualArmCartesianImpedanceController::saturateTorqueRate(
        const FrankaDataContainer &arm_data,
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d)
    { // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++)
        {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                                       -arm_data.delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void DualArmCartesianImpedanceController::complianceParamCallback(
        franka_controllers::dual_arm_compliance_paramConfig &config,
        uint32_t /*level*/)
    {
        auto &left_arm_data = arms_data_.at(left_arm_id_);
        left_arm_data.cartesian_stiffness_target_.setIdentity();
        left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
            << config.left_translational_stiffness * Eigen::Matrix3d::Identity();
        left_arm_data.cartesian_stiffness_target_.bottomRightCorner(3, 3)
            << config.left_rotational_stiffness * Eigen::Matrix3d::Identity();
        left_arm_data.cartesian_damping_target_.setIdentity();

        left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
            << 2 * sqrt(config.left_translational_stiffness) * Eigen::Matrix3d::Identity();
        left_arm_data.cartesian_damping_target_.bottomRightCorner(3, 3)
            << 2 * sqrt(config.left_rotational_stiffness) * Eigen::Matrix3d::Identity();
        left_arm_data.nullspace_stiffness_target_ = config.left_nullspace_stiffness;

        left_arm_data.k_gains_target_.setIdentity();
        left_arm_data.k_gains_target_.topLeftCorner(4, 4) << config.left_k_gains_1234 * Eigen::Matrix4d::Identity();
        left_arm_data.k_gains_target_(4, 4) = config.left_k_gains_5;
        left_arm_data.k_gains_target_(5, 5) = config.left_k_gains_6;
        left_arm_data.k_gains_target_(6, 6) = config.left_k_gains_7;

        left_arm_data.d_gains_target_.setIdentity();
        left_arm_data.d_gains_target_.topLeftCorner(4, 4) << config.left_d_gains_1234 * Eigen::Matrix4d::Identity();
        left_arm_data.d_gains_target_(4, 4) = config.left_d_gains_5;
        left_arm_data.d_gains_target_(5, 5) = config.left_d_gains_6;
        left_arm_data.d_gains_target_(6, 6) = config.left_d_gains_7;


        auto &right_arm_data = arms_data_.at(right_arm_id_);
        right_arm_data.cartesian_stiffness_target_.setIdentity();
        right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
            << config.right_translational_stiffness * Eigen::Matrix3d::Identity();
        right_arm_data.cartesian_stiffness_target_.bottomRightCorner(3, 3)
            << config.right_rotational_stiffness * Eigen::Matrix3d::Identity();
        right_arm_data.cartesian_damping_target_.setIdentity();

        right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
            << 2 * sqrt(config.right_translational_stiffness) * Eigen::Matrix3d::Identity();
        right_arm_data.cartesian_damping_target_.bottomRightCorner(3, 3)
            << 2 * sqrt(config.right_rotational_stiffness) * Eigen::Matrix3d::Identity();
        right_arm_data.nullspace_stiffness_target_ = config.right_nullspace_stiffness;

        right_arm_data.k_gains_target_.setIdentity();
        right_arm_data.k_gains_target_.topLeftCorner(4, 4) << config.right_k_gains_1234 * Eigen::Matrix4d::Identity();
        right_arm_data.k_gains_target_(4, 4) = config.right_k_gains_5;
        right_arm_data.k_gains_target_(5, 5) = config.right_k_gains_6;
        right_arm_data.k_gains_target_(6, 6) = config.right_k_gains_7;

        right_arm_data.d_gains_target_.setIdentity();
        right_arm_data.d_gains_target_.topLeftCorner(4, 4) << config.right_d_gains_1234 * Eigen::Matrix4d::Identity();
        right_arm_data.d_gains_target_(4, 4) = config.right_d_gains_5;
        right_arm_data.d_gains_target_(5, 5) = config.right_d_gains_6;
        right_arm_data.d_gains_target_(6, 6) = config.right_d_gains_7;

    }

    void DualArmCartesianImpedanceController::targetPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        try
        {
            if (msg->header.frame_id != left_arm_id_ + "_link0")
            { // NOLINT
                ROS_ERROR_STREAM(
                    "DualArmCartesianImpedanceController: Got pose target with invalid"
                    " frame_id "
                    << msg->header.frame_id << ". Expected " << left_arm_id_ + "_link0");
                return;
            }

            // Set target for the left robot.
            auto &left_arm_data = arms_data_.at(left_arm_id_);
            Eigen::Affine3d Ol_T_C{}; // NOLINT (readability-identifier-naming)
            tf::poseMsgToEigen(msg->pose, Ol_T_C);
            Eigen::Affine3d Ol_T_EEl_d =     // NOLINT (readability-identifier-naming)
                Ol_T_C * EEl_T_C_.inverse(); // NOLINT (readability-identifier-naming)
            left_arm_data.position_d_target_ = Ol_T_EEl_d.translation();
            Eigen::Quaterniond last_orientation_d_target(left_arm_data.orientation_d_target_);
            Eigen::Quaterniond new_orientation_target(Ol_T_EEl_d.linear()); // NOLINT
            if (last_orientation_d_target.coeffs().dot(new_orientation_target.coeffs()) < 0.0)
            {
                new_orientation_target.coeffs() << -new_orientation_target.coeffs();
            }
            Ol_T_EEl_d.linear() = new_orientation_target.matrix();
            left_arm_data.orientation_d_target_ = Ol_T_EEl_d.linear();

            // Compute target for the right endeffector given the static desired transform from left to
            // right endeffector.
            Eigen::Affine3d Or_T_EEr_d = Ol_T_Or_.inverse()    // NOLINT (readability-identifier-naming)
                                         * Ol_T_EEl_d *        // NOLINT (readability-identifier-naming)
                                         EEr_T_EEl_.inverse(); // NOLINT (readability-identifier-naming)
            auto &right_arm_data = arms_data_.at(right_arm_id_);
            right_arm_data.position_d_target_ =
                Or_T_EEr_d.translation(); // NOLINT (readability-identifier-naming)
            last_orientation_d_target = Eigen::Quaterniond(right_arm_data.orientation_d_target_);
            right_arm_data.orientation_d_target_ =
                Or_T_EEr_d.linear(); // NOLINT (readability-identifier-naming)
            if (last_orientation_d_target.coeffs().dot(right_arm_data.orientation_d_target_.coeffs()) <
                0.0)
            {
                right_arm_data.orientation_d_target_.coeffs()
                    << -right_arm_data.orientation_d_target_.coeffs();
            }
        }
        catch (std::out_of_range &ex)
        {
            ROS_ERROR_STREAM("DualArmCartesianImpedanceController: Exception setting target poses.");
        }
    }

    void DualArmCartesianImpedanceController::publishCenteringPose()
    {
        if (center_frame_pub_.trylock())
        {
            franka::RobotState robot_state_left =
                arms_data_.at(left_arm_id_).state_handle_->getRobotState();
            Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map( // NOLINT (readability-identifier-naming)
                robot_state_left.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
            Eigen::Affine3d Ol_T_C = Ol_T_EEl * EEl_T_C_;  // NOLINT (readability-identifier-naming)
            tf::poseEigenToMsg(Ol_T_C, center_frame_pub_.msg_.pose);
            center_frame_pub_.msg_.header.frame_id = left_arm_id_ + "_link0";
            center_frame_pub_.unlockAndPublish();
        }
    }

    void DualArmCartesianImpedanceController::leftJointCommandCallback(
        const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg)
    {
        auto msg_left = msg;
        auto &left_arm_data = arms_data_.at(left_arm_id_);
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
            left_arm_data.q_d_nullspace_target_(i) = msg->position[i];
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
//        Eigen::Map<Eigen::Matrix<double, 16, 1>> F_T_EE(left_arm_data.state_handle_->getRobotState().F_T_EE.data());
        std::array<double, 16> F_T_EE = left_arm_data.state_handle_->getRobotState().F_T_EE;
        std::array<double, 16> EE_T_K = left_arm_data.state_handle_->getRobotState().EE_T_K;
//        std::array<double, 16> F_T_EE = {0.7071, 0.7071, 0, 0, -0.7071, 0.7071, 0, 0, 0, 0, 1, 0.1034, 0, 0, 0, 1}; // NOLINT(readability-identifier-naming)
//        std::array<double, 16> EE_T_K = {}; // NOLINT(readability-identifier-naming)
        // F_T_EE.zero();
        std::array<double, 16> EEPose = left_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, joint_position_cmd, F_T_EE, EE_T_K);
        // std::lock_guard<std::mutex> position_d_target_mutex_lock(left_arm_data.position_and_orientation_d_target_mutex_);
//        left_arm_data.position_d_target_ << EEPose[3], EEPose[7], EEPose[11];


        Eigen::Quaterniond last_orientation_d_target(left_arm_data.orientation_d_target_);
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        rotationMatrix << EEPose[0], EEPose[1], EEPose[2], EEPose[4], EEPose[5], EEPose[6], EEPose[8], EEPose[9], EEPose[10];
        Eigen::Quaterniond new_orientation_target(rotationMatrix);
        left_arm_data.orientation_d_target_.coeffs() << new_orientation_target.coeffs().transpose().x(), new_orientation_target.coeffs().transpose().y(),
            new_orientation_target.coeffs().transpose().z(), new_orientation_target.coeffs().transpose().w();
        if (last_orientation_d_target.coeffs().dot(left_arm_data.orientation_d_target_.coeffs()) < 0.0)
        {
            left_arm_data.orientation_d_target_.coeffs() << -left_arm_data.orientation_d_target_.coeffs();
        }

        auto ee_pose = left_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, joint_position_cmd, F_T_EE, EE_T_K);
        auto ee_affine = affx::Affine(ee_pose);
        left_arm_data.impedance_motion_.setTarget(ee_affine);
        left_arm_data.position_d_target_  << left_arm_data.impedance_motion_.target.translation();
    }

    void DualArmCartesianImpedanceController::rightJointCommandCallback(
        const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg)
    {
        auto &right_arm_data = arms_data_.at(right_arm_id_);
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
            right_arm_data.q_d_nullspace_target_(i) = msg->position[i];
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
        std::array<double, 16> F_T_EE = right_arm_data.state_handle_->getRobotState().F_T_EE;
        std::array<double, 16> EE_T_K = right_arm_data.state_handle_->getRobotState().EE_T_K;
        // F_T_EE.zero();
        std::array<double, 16> EEPose = right_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, joint_position_cmd, F_T_EE, EE_T_K);
        // std::lock_guard<std::mutex> position_d_target_mutex_lock(right_arm_data.position_and_orientation_d_target_mutex_);
//        right_arm_data.position_d_target_ << EEPose[3], EEPose[7], EEPose[11];

        Eigen::Quaterniond last_orientation_d_target(right_arm_data.orientation_d_target_);
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        rotationMatrix << EEPose[0], EEPose[1], EEPose[2], EEPose[4], EEPose[5], EEPose[6], EEPose[8], EEPose[9], EEPose[10];
        Eigen::Quaterniond new_orientation_target(rotationMatrix);
        right_arm_data.orientation_d_target_.coeffs() << new_orientation_target.coeffs().transpose().x(), new_orientation_target.coeffs().transpose().y(),
            new_orientation_target.coeffs().transpose().z(), new_orientation_target.coeffs().transpose().w();
        if (last_orientation_d_target.coeffs().dot(right_arm_data.orientation_d_target_.coeffs()) < 0.0)
        {
            right_arm_data.orientation_d_target_.coeffs() << -right_arm_data.orientation_d_target_.coeffs();
        }

        auto ee_pose = right_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, joint_position_cmd, F_T_EE, EE_T_K);
        auto ee_affine = affx::Affine(ee_pose);
        right_arm_data.impedance_motion_.setTarget(ee_affine);
        right_arm_data.position_d_target_  << right_arm_data.impedance_motion_.target.translation();
    }

    void DualArmCartesianImpedanceController::leftPoseCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS_WARN_STREAM("Received left arm target pose! Attention please! left will move to the target pose!!");
    auto& left_arm_data = arms_data_.at(left_arm_id_);
    // std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
    left_arm_data.position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(left_arm_data.orientation_d_target_);
    left_arm_data.orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(left_arm_data.orientation_d_target_.coeffs()) < 0.0) {
      left_arm_data.orientation_d_target_.coeffs() << -left_arm_data.orientation_d_target_.coeffs();
        }
    }

    void DualArmCartesianImpedanceController::rightPoseCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS_WARN_STREAM("Received right arm target pose! Attention please! right will move to the target pose!!");
    auto& right_arm_data = arms_data_.at(right_arm_id_);
    // std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
    right_arm_data.position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(right_arm_data.orientation_d_target_);
    right_arm_data.orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(right_arm_data.orientation_d_target_.coeffs()) < 0.0) {
      right_arm_data.orientation_d_target_.coeffs() << -right_arm_data.orientation_d_target_.coeffs();
        }
    }

    bool DualArmCartesianImpedanceController::executeswitchControllerCb(franka_controllers::switchController::Request &req,
                                                                        franka_controllers::switchController::Response &res){

        auto& left_arm_data = arms_data_.at(left_arm_id_);
        auto& right_arm_data = arms_data_.at(right_arm_id_);
        if (strcmp(req.controllerType.c_str(), "joint_impedance") == 0)
        {
            for (int i = 0; i < 7; ++i) {
                left_arm_data.q_d_nullspace_target_[i] = left_arm_data.state_handle_->getRobotState().q[i];
                right_arm_data.q_d_nullspace_target_[i] = right_arm_data.state_handle_->getRobotState().q[i];
            }
            controller_type_ = 1;
            ROS_WARN_STREAM("Switching to Joint Impedance Controller");
            res.finishSwitchController = true;
        }else if(strcmp(req.controllerType.c_str(), "cartesian_impedance") == 0){
            std::array<double, 7> left_joint_position_cmd{}, right_joint_position_cmd{};
            for (int i = 0; i < 7; ++i) {
                left_joint_position_cmd[i] = left_arm_data.state_handle_->getRobotState().q[i];
                right_joint_position_cmd[i] = right_arm_data.state_handle_->getRobotState().q[i];
            }
            std::array<double, 16> left_F_T_EE = left_arm_data.state_handle_->getRobotState().F_T_EE;
            std::array<double, 16> left_EE_T_K = left_arm_data.state_handle_->getRobotState().EE_T_K;

            std::array<double, 16> right_F_T_EE = right_arm_data.state_handle_->getRobotState().F_T_EE;
            std::array<double, 16> right_EE_T_K = right_arm_data.state_handle_->getRobotState().EE_T_K;

            auto left_ee_pose = left_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, left_joint_position_cmd, left_F_T_EE, left_EE_T_K);
            auto left_ee_affine = affx::Affine(left_ee_pose);
            left_arm_data.impedance_motion_.setTarget(left_ee_affine);
            left_arm_data.position_d_target_  << left_arm_data.impedance_motion_.target.translation();

            auto right_ee_pose = right_arm_data.model_handle_->getPose(franka::Frame::kEndEffector, right_joint_position_cmd, right_F_T_EE, right_EE_T_K);
            auto right_ee_affine = affx::Affine(right_ee_pose);
            right_arm_data.impedance_motion_.setTarget(right_ee_affine);
            right_arm_data.position_d_target_  << right_arm_data.impedance_motion_.target.translation();

            controller_type_ = 2;
            res.finishSwitchController = true;
            ROS_WARN_STREAM("Switching to Cartesian Impedance Controller");
        }else{
            res.finishSwitchController = false;
            ROS_WARN_STREAM("Invalid Controller Type!!!");
        }
        return true;
    }

    bool DualArmCartesianImpedanceController::executemovetoHomeCB(franka_controllers::movetoHome::Request &req,
                                                                  franka_controllers::movetoHome::Response &res) {
        auto& left_arm_data = arms_data_.at(left_arm_id_);
        auto& right_arm_data = arms_data_.at(right_arm_id_);
        if (strcmp(req.arm.c_str(), "left") == 0){
            franka_controllers::createTrajectory left_srv;
            for (int i = 0; i < 7; ++i) {
                left_srv.request.currentJoint[i] = left_arm_data.state_handle_->getRobotState().q[i];
            }
            left_srv.request.targetPoint = req.targetPoint;
            if (left_createTrajectoryclient_.call(left_srv)){
                ROS_INFO("Succeed call the left arm trajectory create service");
                res.finishPublishCommand = true;
            } else{
                ROS_ERROR("/panda_left/create_trajectory srv is not exit!!");
                res.finishPublishCommand = false;
            }
        } else if (strcmp(req.arm.c_str(), "right") == 0){
            franka_controllers::createTrajectory right_srv;
            for (int i = 0; i < 7; ++i) {
                right_srv.request.currentJoint[i] = right_arm_data.state_handle_->getRobotState().q[i];
            }
            right_srv.request.targetPoint = req.targetPoint;
            if (right_createTrajectoryclient_.call(right_srv)){
                ROS_INFO("Succeed call the right arm trajectory create service");
                res.finishPublishCommand = true;
            } else{
                ROS_ERROR("/panda_right/create_trajectory srv is not exit!!");
                res.finishPublishCommand = false;
            }
        } else if (strcmp(req.arm.c_str(), "dual") == 0){
            franka_controllers::createTrajectory left_srv;
            franka_controllers::createTrajectory right_srv;
            for (int i = 0; i < 7; ++i) {
                left_srv.request.currentJoint[i] = left_arm_data.state_handle_->getRobotState().q[i];
            }
            left_srv.request.targetPoint = req.targetPoint;
            for (int i = 0; i < 7; ++i) {
                right_srv.request.currentJoint[i] = right_arm_data.state_handle_->getRobotState().q[i];
            }
            right_srv.request.targetPoint = req.targetPoint;

            if (left_createTrajectoryclient_.call(left_srv)&&right_createTrajectoryclient_.call(right_srv)){
                ROS_INFO("Succeed call the dual arm trajectory create service");
                res.finishPublishCommand = true;
            } else{
                res.finishPublishCommand = false;
                ROS_ERROR("/panda_left/create_trajectory or /panda_right/create_trajectory srv is not exit!!");
            }

//            if (left_createTrajectoryclient_.call(left_srv)){
//                ROS_INFO("Succeed call the left arm trajectory create service");
//            } else{
//                ROS_ERROR("/panda_left/create_trajectory srv is not exit!!");
//            }
//            if (right_createTrajectoryclient_.call(right_srv)){
//                ROS_INFO("Succeed call the right arm trajectory create service");
//            } else{
//                ROS_ERROR("/panda_right/create_trajectory srv is not exit!!");
//            }

        } else{
            res.finishPublishCommand = false;
            ROS_WARN("Invilad arm name!");
        }
        return true;
    }

    bool DualArmCartesianImpedanceController::executemovetoTargetLeftCB(franka_controllers::movetoHome::Request &req,
                                                                  franka_controllers::movetoHome::Response &res) {
        auto& left_arm_data = arms_data_.at(left_arm_id_);
        franka_controllers::createTrajectory left_srv;
        for (int i = 0; i < 7; ++i) {
            left_srv.request.currentJoint[i] = left_arm_data.state_handle_->getRobotState().q[i];
            left_srv.request.desireJoint[i] = req.desireJoint[i];
        }
        left_srv.request.targetPoint = req.targetPoint;
        if (left_createTrajectoryclient_.call(left_srv)){
            ROS_INFO("Succeed call the left arm trajectory create service");
            res.finishPublishCommand = true;
        } else{
            ROS_ERROR("/panda_left/create_trajectory srv is not exit!!");
            res.finishPublishCommand = false;
        }
    return true;
    }

    
    bool DualArmCartesianImpedanceController::executemovetoTargetRightCB(franka_controllers::movetoHome::Request &req,
                                                                  franka_controllers::movetoHome::Response &res) {
        auto& right_arm_data = arms_data_.at(right_arm_id_);
        franka_controllers::createTrajectory right_srv;
        for (int i = 0; i < 7; ++i) {
            right_srv.request.currentJoint[i] = right_arm_data.state_handle_->getRobotState().q[i];
            right_srv.request.desireJoint[i] = req.desireJoint[i];
        }
        right_srv.request.targetPoint = req.targetPoint;
        if (right_createTrajectoryclient_.call(right_srv)){
            ROS_INFO("Succeed call the right arm trajectory create service");
            res.finishPublishCommand = true;
        } else{
            ROS_ERROR("/panda_right/create_trajectory srv is not exit!!");
            res.finishPublishCommand = false;
        }
    return true;
    }

} // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::DualArmCartesianImpedanceController,
                       controller_interface::ControllerBase)