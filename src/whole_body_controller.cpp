//
// Created by Ray in SMART Lab on 2021/11/02.
//

#include "franka_controllers/whole_body_controller.h"
#include "ros/ros.h"
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "franka_controllers/pseudo_inversion.h"
#include <tf/tf.h>

namespace franka_controllers{

    void TestClass::run(){
        ROS_INFO("run function in src file");
    }

    bool WholeBodyController::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& node_handle){
        // ROS_INFO("This means the init function is ok!!");
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        sub_equilibrium_pose_ = node_handle.subscribe("equilibrium_pose", 1, &WholeBodyController::equilibriumPoseCallback, this,ros::TransportHints().reliable().tcpNoDelay());
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("WholeBodyController: Could not read parameter arm_id");
            return false;
            }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR("WholeBodyController: Invalid or no joint_names parameters provided, " "aborting controller init!");
            return false;
            }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("WholeBodyController: Error getting model interface from hardware");
            return false;
            }
        try {model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
            } 
        catch (hardware_interface::HardwareInterfaceException& ex){
            ROS_ERROR_STREAM("WholeBodyController: Exception getting model handle from interface: "<< ex.what());
            return false;
            }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM("WholeBodyController: Error getting state interface from hardware");
            return false;
            }
        try {state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
            } 
        catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("WholeBodyController: Exception getting state handle from interface: "<< ex.what());
            return false;
            }
        
        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("WholeBodyController: Error getting effort joint interface from hardware");
        return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } 
            catch (const hardware_interface::HardwareInterfaceException& ex) {ROS_ERROR_STREAM("WholeBodyController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }

        dynamic_reconfigure_compliance_param_node_ =ros::NodeHandle(node_handle.getNamespace() +"dynamic_reconfigure_compliance_param_node");
        dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_controllers::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
        dynamic_server_compliance_param_->setCallback(boost::bind(&WholeBodyController::complianceParamCallback, this, _1, _2));

        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        cartesian_stiffness_.setZero();
        cartesian_damping_.setZero();

        //initializing some new add parameters:
        mobile_cmd_vel_.setZero();
        mobile_cmd_vel_old.setZero();
        mobile_fed_position_.setZero();
        mobile_fed_vel_.setZero();
        // mobile control topic publisher is initialized here: 
        pub_mobile_vol_ = node_handle.advertise<geometry_msgs::Twist>("/summit_xl/robotnik_base_control/cmd_vel", 1);
        // mobile platform message with topic name "/mobile_platform/odom" subscriber is initialized here, with callback function MobileCallback()
        sub_mobile_posevol_ = node_handle.subscribe("/summit_xl/robotnik_base_control/odom", 1, &WholeBodyController::MobileCallback,this,
                                               ros::TransportHints().reliable().tcpNoDelay());  
        // setting the initial status (both position and velocity) of mobile platform are 0, which means it is the same as world coordinate
        r_x = 0,r_y = 0,r_z = 0;
        // franka end-effector under base franme 
        x_q = 0,y_q = 0,z_q = 0;
        //Then calculate the initial Jocabian J_mobile for mobile platform:
        //TODO

        W_rz <<
        cos(r_z),-sin(r_z),0,0,0,0,
        sin(r_z),cos(r_z),0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;

        J_mobile <<
        1,0,0,
        0,1,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,1;
        return true;
    }

    void WholeBodyController::starting(const ros::Time& /*time*/) {
        //This function mainliy finish some parameters(equilibrium point, etc.) for every control loop to update before update the control command for controlling;

        ROS_INFO("This means the starting function is ok!");
        while (mobile_callback == false)
        {
            ros::Duration(0.005).sleep();//delay 5ms
            /* code */
        }
        if (mobile_callback == true){
        ROS_INFO("Finish delaying!!");
        // q_d_null_full_.head(3) << mobile_fed_position_;
        // mobile_initial_position << mobile_fed_position_;
        r_x = mobile_fed_position_[0];
        r_y = mobile_fed_position_[1];
        r_z = mobile_fed_position_[2];
        }

        //Calculating the Transforn from end-effector to the world coordinate:
        //TODO: 1) first calculate the transform matrix from franka base to mobile platform base, and 2) then calculate the transform matrix from mobile platform base to the world base, 3) Then use the Eigen::Affine3d to calculte the fangshe translation, 4) Finally get the initial_transform_full
        //get the dq joint velocity:
        //1) Homogeneous transform: the transform matrix from franka base to mobile platform base:
        transform_mtfb_temp <<
                   1,0,0,x_0,
                   0,1,0,y_0,
                   0,0,1,z_0,  
                   0,0,0,1;
        //2) Homogeneous transform: the transform matrix from mobile platform base to the world coordinate:
        transform_wtm_temp << 
                   cos(r_z),-sin(r_z),0,r_x,
                   sin(r_z),cos(r_z),0,r_y,
                   0,0,1,0,
                   0,0,0,1;
        // Homogeneous transform: the franka_base to world:
        franka::RobotState initial_state = state_handle_->getRobotState();

        //d(q) joint velocity
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        transform_wtfb_temp = transform_wtm_temp*transform_mtfb_temp;
        //get the transform from end-effector to franka base:
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Affine3d transform_wtfb(Eigen::Matrix4d::Map(transform_wtfb_temp.data()));
        //4) Finally get the initial_transform_full: the Homogeneous transform: the franka end-effector to the world:
        Eigen::Affine3d initial_transform_full = transform_wtfb*initial_transform;
        //5) Also get the W_rz matrix:
        W_rz <<
        cos(r_z),-sin(r_z),0,0,0,0,
        sin(r_z),cos(r_z),0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;

        //TODO
        //Then set the initial equilibrium point th current state with the initial_transform_full transform:
        // set equilibrium point to current state, set the position_d_ as the initial equilibrium status:
        position_d_ = initial_transform_full.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform_full.linear());
        position_d_target_ = initial_transform_full.translation();
        orientation_d_target_ = Eigen::Quaterniond(initial_transform_full.linear());

        // set nullspace equilibrium configuration to initial q
        q_d_nullspace_arm_ = q_initial; 
        q_d_null_full_.tail(7) = q_d_nullspace_arm_;
        q_d_nullspace_mobile_ << r_x,r_y,r_z;
        q_d_null_full_.head(3) = q_d_nullspace_mobile_;
        
        // setting the initial desired M and D matrix for mobile robot;
        virtual_initial_mobile_ <<
        mob_tran_Mparams_,0,0,
        0,mob_tran_Mparams_,0,
        0,0,mob_tran_Mparams_,
        // ration=2 dlr
        mob_tran_Dparams_ = 1.5* mob_tran_Mparams_;
        virtual_damping_mobile_ <<  
        mob_tran_Dparams_,0,0,
        0,mob_tran_Dparams_,0,
        0,0,mob_tran_Dparams_;

        //initialize the scale matrix:
        scale_matrix=Eigen::MatrixXd::Identity(10, 10);

    }
    void WholeBodyController::update(const ros::Time& /*time*/,const ros::Duration& period) {

        //To calculate the control torque, first to update the latest information, including the feedback information from mobile platform, and the information from franka,

        //(1)updating the information from mobile platform:
        r_x = mobile_fed_position_[0];
        r_y = mobile_fed_position_[1];
        r_z = mobile_fed_position_[2];

        //(2)Then calculating the current transformation from end-effector to the world coordination, as what starting() function has done
        //2.1) Homogeneous transform: the transform matrix from franka base to mobile platform base:
        transform_mtfb_temp <<
                   1,0,0,x_0,
                   0,1,0,y_0,
                   0,0,1,z_0,  
                   0,0,0,1;
        //2.2) Homogeneous transform: the transform matrix from mobile platform base to the world coordinate:
        transform_wtm_temp << 
                   cos(r_z),-sin(r_z),0,r_x,
                   sin(r_z),cos(r_z),0,r_y,
                   0,0,1,0,
                   0,0,0,1;
        transform_wtfb_temp = transform_wtm_temp*transform_mtfb_temp;
        //get the transform from end-effector to franka base:
        Eigen::Affine3d transform_wtfb(Eigen::Matrix4d::Map(transform_wtfb_temp.data()));
        //the transform from end-effector to franka base
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Affine3d transform_fbe(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        //the external Force
        Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(robot_state.K_F_ext_hat_K.data());
        //Finally the end-effector position and orientation in world coordinate:
        // Eigen::Vector3d position_0(transform_fbe.translation());
        // x_q=position_0[0];
        // y_q=position_0[1];
        // z_q=position_0[2];
        Eigen::Affine3d transform_full = transform_wtfb*transform_fbe;
        Eigen::Vector3d position(transform_full.translation());
        Eigen::Quaterniond orientation(transform_full.linear()); 

        // q_d_nullspace_arm_ = q_initial; 
        // q_d_null_full_.tail(7) = q_d_nullspace_arm_;
        // q_d_nullspace_mobile_ << r_x,r_y,r_z;
        // q_d_null_full_.head(3) = q_d_nullspace_mobile_;

        //(3) get some franka status to calculate the contro torque
        //3.1) get state variables
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 49> mass_array = model_handle_->getMass();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        // get the custimized gravity torque:
        std::array<double, 7> gravity_array = model_handle_->getGravity();

//      std::array<double, 7> getGravity(
//      const std::array<double, 7>& q,
//      double total_mass,
//      const std::array<double, 3>& F_x_Ctotal,  // NOLINT (readability-identifier-naming)
//      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const {
//      return model_->gravity(q, total_mass, F_x_Ctotal, gravity_earth);
//      }

        //3.2) convert to Eigen
        Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());

        // (4)compute control
        // 4.1) allocate variables
        Eigen::Matrix<double, 10, 10> M_full;
        // M_full.setZero();

        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_d_r(7), tau_d_w(10),tau_imp_w(10),tau_null_w(10),tau_d_m(3), tau_full_franka(7),q_full(10),dq_full(10),dq_full_move_base(3),dq_full_franka(7);

        dq_full.head(3) << mobile_fed_vel_;
        dq_full.tail(7) << dq;
        q_full.head(3)  << mobile_fed_position_;
        q_full.tail(7)  << q;


        //Then updating the Jocabian matrix
        Eigen::MatrixXd Jacobian_temp = W_rz*jacobian;
        Eigen::Matrix<double, 6, 10> Jacobian_full;
        Jacobian_full.setZero();
        Jacobian_full.block<6,3>(0,0) = J_mobile;  
        Jacobian_full.block<6,7> (0,3) = Jacobian_temp;

        M_full.block<3,3>(0,0) = virtual_initial_mobile_; 
        M_full.block<7,7>(3,3) = mass; 

        // weight_matrix=M_full.inverse()*scale_matrix;
        //calculate the inverse of Cartesian_inertia matrix of whole-body dynamics here so don't have to use the inverse of Jacobian^T
        Cartesian_inertia_inverse = Jacobian_full*M_full.inverse()*Jacobian_full.transpose();

        //weight_Cartesian_inertia_inverse
        // weight_Cartesian_inertia_inverse = Jacobian_full*M_full.inverse()*weight_matrix.inverse()*M_full.inverse()*Jacobian_full.transpose();

        //compute error to desired pose
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
        // Transform to world frame
        error.tail(3) << -transform_full.linear() * error.tail(3);

        // print the current position and orientation under world frame
        ROS_INFO_THROTTLE(3,"Current positon is: x:%.4f, y:%.4f, z:%.4f, QX:%.4f, QY:%.4f, QZ:%.4f, QW:%.4f", position(0),position(1),position(2),orientation.x(),orientation.y(),orientation.z(),orientation.w());

        tau_imp_w << Jacobian_full.transpose() *(-cartesian_stiffness_ * error - cartesian_damping_ * (Jacobian_full * dq_full));
        tau_null_w << (Eigen::MatrixXd::Identity(10, 10) - Jacobian_full.transpose() *Cartesian_inertia_inverse.inverse()*Jacobian_full*M_full.inverse()) *
                       (nullspace_stiffness_ * (q_d_null_full_ - q_full) - (2.0 * sqrt(nullspace_stiffness_)) * dq_full);

        // Then updating the torque vector;
        // ROS_INFO("For test!!!");
        // ROS_INFO("This means the update function is ok!!");       
        //1) To finish the whole-body controller, to command the torque here, 
        //step zero to ommit the original cartesian controller:
        // Desired torque
       //1.1) first need to write down the final torque for robot arm here as:
        tau_d_w << tau_imp_w + tau_null_w;
        tau_d_r << tau_d_w.tail(7) + coriolis;
        // tau_d_r << tau_task + tau_nullspace + coriolis;
        // Saturate torque rate to avoid discontinuities
        tau_d_r << saturateTorqueRate(tau_d_r, tau_J_d);
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d_r(i));
            }

        //1.2) Then need to write down the final velocity command for summit_xl steel here as(publish the velocity command as a topic):
        //1.2.1) first calculate the desired velocity with the admittance control:
        tau_d_m << tau_d_w.head(3);
        // Integrate the acceleration to mobile velocity
        mobile_cmd_vel_=((virtual_initial_mobile_+period.toSec()*virtual_damping_mobile_).inverse())*(virtual_initial_mobile_*mobile_cmd_vel_old+period.toSec()*tau_d_m);//Admittance Control for mobile robot;
        //mobile_cmd_acc_ = M_adm.inverse()*(tau_m - D_adm*mobile_cmd_vel_);
        //mobile_cmd_vel_ = mobile_cmd_vel_ + period.toSec()*mobile_cmd_acc_;
        mobile_cmd_vel_old=mobile_cmd_vel_;
        //1.2.2) used the desired vel to initial the rostopic mes wheelCommand and then pub:
        /* initial the rostopic:
        TO DO
        */
        for (size_t p=0;p<2;p++){
            if (std::isnan(mobile_cmd_vel_[p]))
            mobile_cmd_vel_[p] = 0.0;
            // if (std::abs(mobile_cmd_vel_[p])<=0.0
            // mobile_cmd_vel_[p] = 0.0;
            if (mobile_cmd_vel_[p]>0.08)
            mobile_cmd_vel_[p] = 0.08;
            if(mobile_cmd_vel_[p]<-0.08)
            mobile_cmd_vel_[p] = -0.08;
        }
        //pub the pub_mobile_vol_.publish(twist);
        wheelCommand.linear.x = mobile_cmd_vel_(0);
        wheelCommand.linear.y = mobile_cmd_vel_(1);
        wheelCommand.linear.z = 0;
        wheelCommand.angular.x = 0;
        wheelCommand.angular.y = 0;
        wheelCommand.angular.z = mobile_cmd_vel_(2);
        pub_mobile_vol_.publish(wheelCommand);
        // after publishing the control command for the mobile platform, an update loop for controlling is finished.
        // then update parameters changed online either through dynamic reconfigure or through the interactive
        // target by filtering
        cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
        cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
        nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        //update the virtual_damping_mobile matrix:
        // virtual_damping_mobile_ = filter_params_ * virtual_damping_mobile_target_ + (1.0 - filter_params_) * virtual_damping_mobile_;
        std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
    }

    Eigen::Matrix<double, 7, 1> WholeBodyController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void WholeBodyController::complianceParamCallback(
    franka_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
        cartesian_stiffness_target_.setIdentity();
        cartesian_stiffness_target_.topLeftCorner(3, 3) << config.translational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_stiffness_target_.bottomRightCorner(3, 3) << config.rotational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.setIdentity();
        // Damping ratio = 1
        cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
        nullspace_stiffness_target_ = config.nullspace_stiffness;
        // virtual inertial and virtual damping update for the mobile platform:
        virtual_initial_mobile_target_.setIdentity();
        virtual_initial_mobile_target_<<config.mobile_virtual_initial * Eigen::Matrix3d::Identity();
        virtual_damping_mobile_target_.setIdentity();
        virtual_damping_mobile_target_<<config.mobile_virtual_damping * Eigen::Matrix3d::Identity();
    }

    void WholeBodyController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        ROS_WARN_STREAM("Received the target equilibriumPose!");
        // std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
        position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
        orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
        if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();}
    }


    void WholeBodyController::MobileCallback(const nav_msgs::OdometryConstPtr& msg) {
        // pose
        mobile_fed_position_[0] = msg->pose.pose.position.x;
        mobile_fed_position_[1] = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        mobile_fed_position_[2] = yaw;
        // twist
        mobile_fed_vel_[0] = msg->twist.twist.linear.x;
        mobile_fed_vel_[1] = msg->twist.twist.linear.y;
        mobile_fed_vel_[2] = msg->twist.twist.angular.z;
        // mobile_fed_vel_=roation_wtm.transpose()*mobile_fed_vel_;
        mobile_callback = true;
    }

}// namespace franka_example_controllers


PLUGINLIB_EXPORT_CLASS(franka_controllers::WholeBodyController,
                       controller_interface::ControllerBase)