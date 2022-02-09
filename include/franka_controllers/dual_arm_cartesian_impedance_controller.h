//
// Created by Ray in 2021/12/06
//
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
#include <frankx/frankx.hpp>
#include <frankx/motion_generator.hpp>
#include <movex/motion/motion_impedance.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <franka_controllers/movetoHome.h>
#include <franka_controllers/switchController.h>
#include <franka_controllers/createTrajectory.h>

namespace franka_controllers
{

    /**
 * This container holds all data and parameters used to control one panda arm with a Cartesian
 * impedance control law tracking a desired target pose.
 */
    struct FrankaDataContainer
    {
        std::unique_ptr<franka_hw::FrankaStateHandle>
            state_handle_; ///< To read to complete robot state.
        std::unique_ptr<franka_hw::FrankaModelHandle>
            model_handle_;                                           ///< To have access to e.g. jacobians.
        std::vector<hardware_interface::JointHandle> joint_handles_; ///< To command joint torques.
        double filter_params_{0.005};                                ///< [-] PT1-Filter constant to smooth target values set
                                                                     ///< by dynamic reconfigure servers (stiffness/damping)
                                                                     ///< or interactive markers for the target poses.
        double nullspace_stiffness_{20.0};                           ///< [Nm/rad] To track the initial joint configuration in
                                                                     ///< the nullspace of the Cartesian motion.
        double nullspace_stiffness_target_{20.0};                    ///< [Nm/rad] Unfiltered raw value.
        const double delta_tau_max_{1.0};                            ///< [Nm/ms] Maximum difference in joint-torque per
                                                                     ///< timestep. Used to saturated torque rates to ensure
                                                                     ///< feasible commands.
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;            ///< To track the target pose.
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;     ///< Unfiltered raw value.
        Eigen::Matrix<double, 7, 7> k_gains_target_;            ///< To track the target pose.
        Eigen::Matrix<double, 7, 7> d_gains_target_;     ///< Unfiltered raw value.
        Eigen::Matrix<double, 7, 7> k_gains_;            ///< To track the target pose.
        Eigen::Matrix<double, 7, 7> d_gains_;     ///< Unfiltered raw value.
        Eigen::Matrix<double, 6, 6> cartesian_damping_;              ///< To damp cartesian motions.
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;       ///< Unfiltered raw value.
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;                  ///< Target joint pose for nullspace
        Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;                  ///< Target joint pose for nullspace
        Eigen::Matrix<double, 7, 1> dq_d_nullspace_;                  ///< Target joint velocity for nullspace
                                                                     ///< motion. For now we track the
                                                                     ///< initial joint pose.
        Eigen::Vector3d position_d_;                                 ///< Target position of the end effector.
        Eigen::Quaterniond orientation_d_;                           ///< Target orientation of the end effector.
        Eigen::Vector3d position_d_target_;                          ///< Unfiltered raw value.
        Eigen::Quaterniond orientation_d_target_;                    ///< Unfiltered raw value.

        std::array<double, 3> customize_gravity_direction_;
        double tool_mass_;
        std::array<double, 3> tool_vector_;
        std::array<double, 3> *tool_vector_pointer_;
        // std::mutex position_and_orientation_d_target_mutex_;
        movex::ImpedanceMotion impedance_motion_;
    };

    /**
 * Controller class for ros_control that renders two decoupled Cartesian impedances for the
 * tracking of two target poses for the two endeffectors. The controller can be reparameterized at
 * runtime via dynamic reconfigure servers.
 */
    class DualArmCartesianImpedanceController
        : public controller_interface::MultiInterfaceController<
              franka_hw::FrankaModelInterface,
              hardware_interface::EffortJointInterface,
              franka_hw::FrankaStateInterface>
    {
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
        std::map<std::string, FrankaDataContainer>
            arms_data_;            ///< Holds all relevant data for both arms.
        std::string left_arm_id_;  ///< Name of the left arm, retrieved from the parameter server.
        std::string right_arm_id_; ///< Name of the right arm, retrieved from the parameter server.

        ///< Transformation between base frames of the robots.
        Eigen::Affine3d Ol_T_Or_; // NOLINT (readability-identifier-naming)
        ///< Target transformation between the two endeffectors.
        Eigen::Affine3d EEr_T_EEl_; // NOLINT (readability-identifier-naming)
        ///< Transformation from the centering frame to the left end effector.
        Eigen::Affine3d EEl_T_C_{};

        ///< Publisher for the centering tracking frame of the coordinated motion.
        realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> center_frame_pub_;
        ///< Rate to trigger publishing the current pose of the centering frame.
        franka_hw::TriggerRate publish_rate_;

        Eigen::Matrix<double, 7, 1> upper_joint_position_warning_limits_, lower_joint_position_warning_limits_;

        /**
   * Saturates torque commands to ensure feasibility.
   *
   * @param[in] arm_data The data container of the arm.
   * @param[in] tau_d_calculated The raw command according to the control law.
   * @param[in] tau_J_d The current desired torque, read from the robot state.
   * @return The saturated torque command for the 7 joints of one arm.
   */
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
            const FrankaDataContainer &arm_data,
            const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
            const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

        /**
   * Initializes a single Panda robot arm.
   *
   * @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   * @param[in] arm_id The name of the panda arm.
   * @param[in] joint_names The names of all joints of the panda.
   * @return True if successful, false otherwise.
   */
        bool initArm(hardware_interface::RobotHW *robot_hw,
                     const std::string &arm_id,
                     const std::vector<std::string> &joint_names, std::array<double, 3> customize_gravity_direction,
                     double tool_mass,
                     std::array<double, 3> tool_vector);

        /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
        void updateArm(FrankaDataContainer &arm_data);

        /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
        void startingArm(FrankaDataContainer &arm_data);

        ///< Dynamic reconfigure server
        std::unique_ptr<dynamic_reconfigure::Server<
            franka_controllers::dual_arm_compliance_paramConfig>>
            dynamic_server_compliance_param_;

        ///< Nodehandle for the dynamic reconfigure namespace
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

        /**
   * Callback for updates on the parameterization of the controller in terms of stiffnesses.
   *
   * @param[in] config Data container for configuration updates.
   */
        void complianceParamCallback(
            franka_controllers::dual_arm_compliance_paramConfig &config,
            uint32_t /*level*/);

        ///< Target pose subscriber
        ros::Subscriber sub_target_pose_left_;

        ros::Subscriber sub_desired_joint_state_left_;
        ros::Subscriber sub_desired_joint_state_right_;

        void leftJointCommandCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg);
        void rightJointCommandCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg);

        ros::Subscriber sub_desired_pose_left_;
        ros::Subscriber sub_desired_pose_right_;

        void leftPoseCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void rightPoseCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
   * Callback method that handles updates of the target poses.
   *
   * @param[in] msg New target pose.
   */
        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /**
   * Publishes a Pose Stamped for visualization of the current centering pose.
   */
        void publishCenteringPose();
        ros::ServiceServer switchController_;
        bool executeswitchControllerCb(franka_controllers::switchController::Request& req,
                                       franka_controllers::switchController::Response& res);

        ros::ServiceServer movetoHome_;
        bool executemovetoHomeCB(franka_controllers::movetoHome::Request& req,
                                franka_controllers::movetoHome::Response& res);

        ros::Publisher left_arm_state_publisher_,right_arm_state_publisher_;

        double alpha = 0.99;
        int controller_type_;
        ros::ServiceClient left_createTrajectoryclient_;
        ros::ServiceClient right_createTrajectoryclient_;

    };

} // namespace franka_example_controllers