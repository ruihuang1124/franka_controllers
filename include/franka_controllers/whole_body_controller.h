//
// Created by Ray in SMART Lab on 2021/11/02.
//

#ifndef _FRANKA_CONTROLLERS_H
#define _FRANKA_CONTROLLERS_H

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "franka_controllers/compliance_paramConfig.h"
// matfile
// #include <matlogger2/matlogger2.h>
// #include <matlogger2/utils/mat_appender.h>
// #include <franka_hw/franka_hw.h>

/*
    声名namespace
            |--class
                |--run

*/
namespace franka_controllers{
    class TestClass
    {
    private:
        /* data */
    public:
        void run();
    }; 

    class WholeBodyController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,hardware_interface::EffortJointInterface,franka_hw::FrankaStateInterface>{
        public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

        private:
        double test1;
        //Saturation:
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<franka_controllers::compliance_paramConfig>>dynamic_server_compliance_param_;
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
        void complianceParamCallback(franka_controllers::compliance_paramConfig& config,uint32_t level);

        // Equilibrium pose subscriber
        ros::Subscriber sub_equilibrium_pose_;
        void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        // Other params:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        double filter_params_{0.005};
        double nullspace_stiffness_{20.0};
        double nullspace_stiffness_target_{20.0};
        const double delta_tau_max_{1.0};
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
        std::mutex position_and_orientation_d_target_mutex_;
    };
}

#endif