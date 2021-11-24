// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
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

//new////////////////////////////////////////////////////
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// matfile
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>

namespace franka_example_controllers {

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

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

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  //New///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 底盘位姿速度订阅，定义头文件，初始化init，实际应用update
  ros::Subscriber sub_mobile_posevol_;
  // 底盘控制速度（线速度，角速度）发布
  ros::Publisher pub_mobile_vol_;

  int numberOfwheel = 2;

  trajectory_msgs::JointTrajectory wheelCommand;
  trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

  //声明订阅底盘的位置，速度信息回调函数
  void MobileCallback(const nav_msgs::OdometryConstPtr& msg);

  bool mobile_callback = false;

  // 底盘反馈轴角向量
  Eigen::Vector2d mobile_cmd_vel_;
  Eigen::Vector2d mobile_cmd_vel_old;


  // 底盘反馈轴角向量
  double roll,pitch,yaw;

  // 定义底盘位置和速度向量
  Eigen::Vector3d mobile_fed_position_;
  Eigen::Vector3d mobile_fed_position_old;
  Eigen::Vector3d mobile_fed_vel_;


  // 机器人设定参数，定义偏置量及转换矩阵
  double r_x,r_y,r_z,x_q,y_q,z_q;
  // feankan base frame under move base frame
  double x_0=-0.059,y_0=0,z_0=0.755;
  // 移动底座L_1为两个轮子之间的距离，L_2为x方向偏差，L_3为y方向偏差，L_4为z方向偏差，L_5为轮子半径实际由于重量会压到12.5厘米
  double L_1=0.507, L_2=-0.059, L_3=0.0, L_4=0.7557, L_5=0.125;

//   // 定义计算用到的whole body变量
//   Eigen::Matrix<double, 9, 9> M_full;
//   // M_full.setZero();
//   Eigen::VectorXd tau_imp(9);
//   Eigen::VectorXd tau_null(9);
//   Eigen::VectorXd tau_d(9);
//   Eigen::VectorXd tau_full_franka(7);
//   Eigen::VectorXd tau_move_base(2);

//   Eigen::VectorXd q_full(9); //9
//   Eigen::VectorXd dq_full(9); //9
//   Eigen::VectorXd dq_full_move_base(2); //2
//   Eigen::VectorXd dq_full_franka(7); //9

  // 动力学参数 
  Eigen::Matrix<double, 9, 9> weight_matrix; //式子3.1
  Eigen::Matrix<double, 6, 6> Cartesian_inertia_inverse; //whole body笛卡尔空间惯量
  Eigen::Matrix<double, 6, 6> weight_Cartesian_inertia_inverse; //whole body加权笛卡尔空间惯量
  Eigen::Matrix<double, 6, 6> Cartesian_inertia_arm_inverse;//arm笛卡尔空间惯量

  // 零空间映射
  Eigen::Matrix<double, 9, 9> Null_space_whole_body;
  Eigen::Matrix<double, 7, 7> Null_space_arm;
                    
  // 底盘虚拟质量和阻尼  
  Eigen::Matrix<double, 2, 2> M_adm;
  Eigen::Matrix<double, 2, 2> D_adm;
  double mob_tran_Mparams_{80};
  //double mob_rota_Mparams_{20};
  double mob_tran_Dparams_;
  //double mob_rota_Dparams_;

  // 局部雅克比矩阵
  Eigen::Matrix<double, 6, 2> J_mobile_nonholo;
  Eigen::Matrix<double, 6, 6> J_mobile_nonholo2;
  Eigen::Matrix<double, 6, 6> W_rz;
  Eigen::Matrix<double, 6, 2> J_mobile;   

  // 正运动学齐次变换矩阵
  Eigen::Matrix<double, 4, 4> transform_mtfb_temp;
  Eigen::Matrix<double, 4, 4> transform_wtm_temp;
  Eigen::Matrix<double, 4, 4> transform_wtfb_temp;

  Eigen::Affine3d transform_wtfb;
  // 定义roation_wtm
  Eigen::Matrix<double, 3, 3> roation_wtm;

  // 零空间全自由度向量
  Eigen::Matrix<double, 9, 1> q_d_null_full_;

  //velocity error
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> cartkin_v;

  // 生成mat文件
  XBot::MatLogger2::Ptr logger;
  typedef std::shared_ptr<XBot::MatAppender> Ptr;
  Ptr appender;



//   // 表示笛卡尔空间的位置用vector，起点与终点
//   Eigen::Vector3d position_d_;
//   Eigen::Quaterniond orientation_d_;
//   Eigen::Vector3d position_d_target_;
//   Eigen::Quaterniond orientation_d_target_;

  // 轨迹规划
  ros::Duration elapsed_time_;
  double planTime;
  double r;
  Eigen::Matrix<double,6, 4> para_Matrix;

  //  Eigen::Matrix<double,6, 4> para_Matrix2;
  Eigen::Matrix<double, 4, 1> target_q;
  Eigen::Matrix<double, 4, 1> target_dq;
  Eigen::Matrix<double, 4, 1> target_d2q;

  // 轴角插值
  Eigen::Quaterniond target_quaternion;
  Eigen::AngleAxisd diff_quaternion_angle_axis;
  Eigen::AngleAxisd planed_quaternion_angle_axis;

  //初始位姿的旋转矩阵
  Eigen::Matrix<double, 3, 3> ini_rotation_matrix;
  //Eigen::Matrix<double, 3, 3> ini_rotation_matrix2;

  // 运动分配,权重矩阵
  Eigen::Matrix<double, 9, 9> scale_matrix;

  // 测试轴向量
  Eigen::Vector3d temparry;
  Eigen::Vector3d door_position;

  Eigen::Quaterniond  rpy2qua(const Eigen::Vector3d& eulerAngle);


  Eigen::Matrix<double, 7, 4> QuinticPolynomialPlanning(
    const Eigen::Quaterniond& original_quaternion,
    const Eigen::Quaterniond& desired_quaternion,
    const Eigen::Matrix<double, 4, 1>& original_position,
    const Eigen::Matrix<double, 4, 1>& desired_position,
    const double& planTime);
 

  double awayFromJointLimits(const Eigen::Matrix<double, 7, 1>& q);
  Eigen::Matrix<double, 7, 1> awayFromJointLimitsOptimization(const Eigen::Matrix<double, 7, 1>& q, const double k_0);


};

}  // namespace franka_example_controllers