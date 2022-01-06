//
// Created by ray on 1/6/22.
//

#ifndef FRANKA_CONTROLLERS_DUAL_ARM_JOINT_POSITION_CONTROLLER_H
#define FRANKA_CONTROLLERS_DUAL_ARM_JOINT_POSITION_CONTROLLER_H

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include "franka_controllers/dual_arm_compliance_paramConfig.h"
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <sensor_msgs/JointState.h>

namespace franka_controllers
{
    /**
* This container holds all data and parameters used to control one panda arm with a initial joint
* impedance control law tracking a desired target joint position.
*/
    struct FrankaDataContainerJointCommand
    {
        std::unique_ptr<franka_hw::FrankaStateHandle>
                state_handle_; ///< To read to complete robot state.
        std::unique_ptr<franka_hw::FrankaModelHandle>
                model_handle_;                                           ///< To have access to e.g. jacobians.
        std::vector<hardware_interface::JointHandle> position_joint_handles_; ///< To command joint torques.
        double filter_params_{0.005};                                ///< [-] PT1-Filter constant to smooth target values set
        std::array<double, 7> initial_pose_{};
        std::array<double, 7> desired_pose_{};
        std::array<double, 7> desired_pose_target_{};
    };

    class DualArmJointPositionController
            : public controller_interface::MultiInterfaceController<
                    franka_hw::FrankaModelInterface,
                    hardware_interface::PositionJointInterface,
                    franka_hw::FrankaStateInterface> {
    public:
        /**
* Initializes the controller class to be ready to run.
*
* @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
* @param[in] node_handle Nodehandle that allows getting parameterizations from the server and
* starting subscribers.
* @return True if the controller was initialized successfully, false otherwise.
*/
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        /**
   * Prepares the controller for the real-time execution. This method is executed once every time
   * the controller is started and runs in real-time.
   */
        void starting(const ros::Time &) override;

        /**
   * Computes the control-law and commands the resulting joint torques to the robot.
   *
   * @param[in] period The control period (here 0.001s).
   */
        void update(const ros::Time &, const ros::Duration &period) override;
    private:
        std::map<std::string, FrankaDataContainerJointCommand>
                arms_data_;            ///< Holds all relevant data for both arms.
        std::string left_arm_id_;  ///< Name of the left arm, retrieved from the parameter server.
        std::string right_arm_id_; ///< Name of the right arm, retrieved from the parameter server.
        ///< Rate to trigger publishing the current pose of the centering frame.
        franka_hw::TriggerRate publish_rate_;

        ///< Transformation between base frames of the robots.
        bool initArm(hardware_interface::RobotHW *robot_hw,
                     const std::string &arm_id,
                     const std::vector<std::string> &joint_names);
        /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
        void updateArm(FrankaDataContainerJointCommand &arm_data);
        /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
        void startingArm(FrankaDataContainerJointCommand &arm_data);
        ///< Target pose subscriber
        ros::Subscriber sub_desired_joint_state_left_;
        ros::Subscriber sub_desired_joint_state_right_;
        void leftJointCommandCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg);
        void rightJointCommandCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg);

    };


}

#endif //FRANKA_CONTROLLERS_DUAL_ARM_JOINT_POSITION_CONTROLLER_H
