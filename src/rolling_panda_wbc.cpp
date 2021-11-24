// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>
#include "rolling_panda_wbc.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// #include <franka_example_controllers/pseudo_inversion.h>

#include "pseudo_inversion.h"


//tf tree
#include <tf/tf.h>

namespace franka_example_controllers {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

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
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 订阅底盘的位置和速度信息
  sub_mobile_posevol_ = node_handle.subscribe("/mobile_platform/odom", 1, &CartesianImpedanceExampleController::MobileCallback,this,
                                               ros::TransportHints().reliable().tcpNoDelay());                                              
  // 发布底盘速度
  pub_mobile_vol_ = node_handle.advertise<trajectory_msgs::JointTrajectory>("/mobile_platform/joint_trajectory", 1);

  // mat file
  logger = XBot::MatLogger2::MakeLogger("/tmp/opendoorcase2.mat");
  appender = XBot::MatAppender::MakeInstance();
  appender->add_logger(logger);
  appender->start_flush_thread();

  // 正运动学部分 式子1.7这里non-holonomic地盘部分要修改
  // 移动底座L_1为两个轮子之间的距离，L_2为x方向偏差，L_3为y方向偏差，L_4为z方向偏差，L_5为轮子半径实际由于重量会压到12.5厘米
  J_mobile_nonholo <<
      (L_5/2) * (cos(r_z)+ 2/L_1 *(L_2*sin(r_z))) , (L_5/2) * (cos(r_z)- (2/L_1)*(L_2*sin(r_z))),
      (L_5/2) * (sin(r_z)+ 2/L_1 *(L_2*cos(r_z))) , (L_5/2) * (sin(r_z)- (2/L_1)*(L_2*cos(r_z))),
      0,0,
      0,0,
      0,0,
      -L_5/L_1,L_5/L_1;
  J_mobile_nonholo2 <<
      1,0,0,0,0,-(sin(r_z))*x_q-(cos(r_z))*y_q,
      0,1,0,0,0,(cos(r_z))*x_q-(sin(r_z))*y_q,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;
  J_mobile = J_mobile_nonholo2*J_mobile_nonholo;

  // 正运动学修正 中期报告中 式子1.6
  W_rz <<
      cos(r_z),-sin(r_z),0,0,0,0,
      sin(r_z),cos(r_z),0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,cos(r_z),-sin(r_z),0,
      0,0,0,sin(r_z),cos(r_z),0,
      0,0,0,0,0,1;
  // 正运动学修正 中期报告中 式子1.1
  roation_wtm << 
                   cos(r_z),-sin(r_z),0,
                   sin(r_z),cos(r_z),0,
                   0,0,1;
  // 底盘计算速度设置为0
  mobile_cmd_vel_.setZero();
  mobile_cmd_vel_old.setZero();
  mobile_fed_position_.setZero();
  mobile_fed_vel_.setZero();
 
  // 设定底盘初始状态的位置和速度均为0，即与世界系重合
  r_x = 0,r_y = 0,r_z = 0;
  // franka end-effector under base franme 
  x_q = 0,y_q = 0,z_q = 0;
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // 判断底盘回调函数开始订阅
  while (mobile_callback == false)
  {
      ros::Duration(0.005).sleep();//延时5ms
  }
  // 记录底盘第一次反馈的数据,初始位置Quaternion
  if (mobile_callback == true)
    {
        // 节点起始设置为0,mobile_fed_position_为底盘回调函数读取的信息
        // q_d_null_full_.head(3) << mobile_fed_position_;
        // mobile_initial_position << mobile_fed_position_;
        r_x = mobile_fed_position_[0];
        r_y = mobile_fed_position_[1];
        r_z = mobile_fed_position_[2];
    }

  // 齐次矩阵的转化(机械臂的基坐标相对于移动平台的基坐标的转换)
  transform_mtfb_temp <<
                   1,0,0,x_0,
                   0,1,0,y_0,
                   0,0,1,z_0,  
                   0,0,0,1;

  // 齐次矩阵的转化(移动平台的基坐标相对于世界坐标系的转换)
  transform_wtm_temp << 
                   cos(r_z),-sin(r_z),0,r_x,
                   sin(r_z),cos(r_z),0,r_y,
                   0,0,1,0,
                   0,0,0,1;
                
  transform_wtfb_temp = transform_wtm_temp*transform_mtfb_temp;

  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();

  //d(q)关节速度
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  //末端在基座中的齐次转换矩阵，从机械臂末端到世界坐标系的转化
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  Eigen::Affine3d transform_wtfb(Eigen::Matrix4d::Map(transform_wtfb_temp.data()));
  Eigen::Affine3d initial_transform_full = transform_wtfb*initial_transform;
  
  // set equilibrium point to current state 齐次变换矩阵中rotation部分和position部分; position_d_初始状态设为平衡状态
  position_d_ = initial_transform_full.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform_full.linear());
  position_d_target_ = initial_transform_full.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform_full.linear());

  // set nullspace equilibrium configuration to initial q
   q_d_nullspace_ = q_initial; 
   q_d_null_full_.tail(7) = q_d_nullspace_;

  // 设定底盘的期望M和D矩阵 
   M_adm <<
      mob_tran_Mparams_,0,
      0,mob_tran_Mparams_,

  // ration=3.2 dlr
  mob_tran_Dparams_ = 3.2 * mob_tran_Mparams_;
  D_adm <<  
      mob_tran_Dparams_,0,
      0,mob_tran_Dparams_,
  // 比例矩阵
  scale_matrix=Eigen::MatrixXd::Identity(9, 9);

  // 定义计算用到的whole body变量
  Eigen::Matrix<double, 9, 9> M_full;
  // M_full.setZero();
  Eigen::VectorXd tau_imp(9);
  Eigen::VectorXd tau_null(9);
  Eigen::VectorXd tau_d(9);
  Eigen::VectorXd tau_full_franka(7);  //TO DO
  Eigen::VectorXd tau_move_base(2); //TO DO
  Eigen::VectorXd tau_p(9); //TO DO
  Eigen::VectorXd tau_m(9); //TO DO

  Eigen::VectorXd q_full(9); //9
  Eigen::VectorXd dq_full(9); //9
  Eigen::VectorXd dq_full_move_base(2); //2
  Eigen::VectorXd dq_full_franka(7); //9

}

void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // 底盘运行时间长了，要开机重启
  // 回调命令必须有,需要每次更新的数据,必须在update函数中定义;
  // 设置权重矩阵
  // t=20
  elapsed_time_ += period;
  //地盘位置信息更新
  r_x = mobile_fed_position_[0];
  r_y = mobile_fed_position_[1];
  r_z = mobile_fed_position_[2];
  // 机器人设定参数，定义偏置量及转换矩阵
  transform_mtfb_temp <<
                   1,0,0,x_0,
                   0,1,0,y_0,
                   0,0,1,z_0,
                   0,0,0,1;
  transform_wtm_temp << 
                   cos(r_z),-sin(r_z),0,r_x,
                   sin(r_z),cos(r_z),0,r_y,
                   0,0,1,0,
                   0,0,0,1;
  transform_wtfb_temp = transform_wtm_temp*transform_mtfb_temp;
  Eigen::Affine3d transform_wtfb(Eigen::Matrix4d::Map(transform_wtfb_temp.data())); //三维仿射变换到四维矩阵

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());

  // 定义计算用到的whole body变量
  Eigen::Matrix<double, 9, 9> M_full;
  // M_full.setZero();
  Eigen::VectorXd tau_imp(9);
  Eigen::VectorXd tau_null(9);
  Eigen::VectorXd tau_d(9);
  Eigen::VectorXd tau_full_franka(7);
  Eigen::VectorXd tau_move_base(2);
  Eigen::VectorXd tau_p(9); //TO DO
  Eigen::VectorXd tau_m(9); //TO DO

  Eigen::VectorXd q_full(9); //9
  Eigen::VectorXd dq_full(9); //9
  Eigen::VectorXd dq_full_move_base(2); //2
  Eigen::VectorXd dq_full_franka(7); //9




  dq_full.head(2) << mobile_fed_vel_;
  dq_full.tail(7) << dq;
  q_full.head(2)  << mobile_fed_position_;
  q_full.tail(7)  << q;
  
  // whole body运动学更新////////////////////////////////////////////////////////////////////////////////////////////////
  // Affine3d T是一个4*4齐次矩阵变换，机械臂末端到link0
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); //三维仿射变换到四维矩阵
  // 获取外力robot_state是一个域名,下面有类,函数和值...
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(robot_state.K_F_ext_hat_K.data());
  // 机械臂末端在机械臂基座标系下的位置 https://stackoverflow.com/questions/51091957/apply-translation-to-eigen-vertices
  //https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
  Eigen::Vector3d position_0(transform.translation());
  x_q=position_0[0];
  y_q=position_0[1];
  z_q=position_0[2];
  // 机械臂末端在世界坐标系下的位姿
  Eigen::Affine3d transform_full = transform_wtfb*transform;
  Eigen::Vector3d position(transform_full.translation());
  Eigen::Quaterniond orientation(transform_full.linear()); 
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



  // whole body雅可比更新////////////////////////////////////////////////////////////////////////////////////////////////
  //Only consider manipulator
  Eigen::MatrixXd jacobian_temp = W_rz*jacobian; //式1.6
  Eigen::Matrix<double, 6, 9> jacobian_full;
  jacobian_full.setZero();
  jacobian_full.block<6,2>(0,0) = J_mobile;  //J_mobile这个变量的更新机制
  // 正运动学修正，因为速度指令是在移动底盘的坐标系下面发布的，要将其转化为世界坐标系下的描述。non-holonomic move base doesnt need this regulation 
  //J_mobile = J_mobile*roation_wtm;
  jacobian_full.block<6,7> (0,2) = jacobian_temp;
  // Eigen::Vector3d position(transform.translation()); 机械臂末端在机械臂基座标系下的位置
  // Eigen::Quaterniond orientation(transform.linear()); 机械臂末端在机械臂基座标系下的位置
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // M_full，未进行动力学补偿
  M_full.block<2,2>(0,0) = M_adm; 
  M_full.block<7,7>(2,2) = mass; 

  //weight_matrix=scale_matrix.transpose()*M_full.inverse()*scale_matrix;
  weight_matrix=M_full.inverse()*scale_matrix;
  
  // 尽量不要使用雅克比转置的伪逆(q,x)同一个，式2.2
  Cartesian_inertia_inverse = jacobian_full*M_full.inverse()*jacobian_full.transpose();

  //weight_Cartesian_inertia_inverse，式3.6
  weight_Cartesian_inertia_inverse = jacobian_full*M_full.inverse()*weight_matrix.inverse()*M_full.inverse()*jacobian_full.transpose();
 
  // N_Q(whole body零空间映射)，式2.6 // Eigen::Matrix<double, 9, 9> Null_space_whole_body;
  Null_space_whole_body = Eigen::MatrixXd::Identity(9, 9)- jacobian_full.transpose()*Cartesian_inertia_inverse.inverse()*jacobian_full*M_full.inverse();

  // 机械臂零空间N_Q_ARM, khatib matrix  // Eigen::Matrix<double, 6, 6> Cartesian_inertia_arm_inverse;//A_X
  Cartesian_inertia_arm_inverse = jacobian*mass.inverse()*jacobian.transpose();

  //arm 零空间映射
  Null_space_arm = Eigen::MatrixXd::Identity(7, 7)-jacobian.transpose()*Cartesian_inertia_arm_inverse.inverse()*jacobian*mass.inverse();

  // 关节限制优化 暂时不用
  // Eigen::Matrix<double, 7, 1> tau_jointlimit;
  // tau_jointlimit << awayFromJointLimitsOptimization(q,100);
  // tau_jointlimit << N_Q_ARM * tau_jointlimit ;

  tau_imp << weight_matrix.inverse()*M_full.inverse()*jacobian_full.transpose() *weight_Cartesian_inertia_inverse.inverse()*Cartesian_inertia_inverse*
               (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian_full * dq_full-cartkin_v));
   
  tau_null << (Eigen::MatrixXd::Identity(10, 10) - weight_matrix.inverse()*M_full.inverse()*jacobian_full.transpose() *weight_Cartesian_inertia_inverse.inverse()*jacobian_full*M_full.inverse()) *
                       (nullspace_stiffness_ * (q_d_null_full_ - q_full) - (2.0 * sqrt(nullspace_stiffness_)) * dq_full);

  // Desired torque
  tau_d << tau_imp + tau_null;

  tau_p << tau_d.tail(7) + coriolis;//+ tau_jointlimit;// 加了之后机械臂关节限位发生碰撞,没有加零空间力矩,超出工作空间限制，理论上没有操作空间限制。。。
  tau_p << saturateTorqueRate(tau_p, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_p(i));// 发送机械臂力矩指令
  }

/*  
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

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

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
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
*/
  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);


  //地盘速度发送/////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 计算并发布底盘cmd_vel
  // Eigen::Vector3d mobile_cmd_acc_;
  tau_m << tau_d.head(2);
  // 对加速度进行积分运算
  mobile_cmd_vel_=((M_adm+period.toSec()*D_adm).inverse())*(M_adm*mobile_cmd_vel_old+period.toSec()*tau_m);//导纳控制
  //   mobile_cmd_acc_ = M_adm.inverse()*(tau_m - D_adm*mobile_cmd_vel_);
  //   mobile_cmd_vel_ = mobile_cmd_vel_ + period.toSec()*mobile_cmd_acc_;
  mobile_cmd_vel_old=mobile_cmd_vel_;
  for (size_t p=0;p<2;p++)
  {
    if (std::isnan(mobile_cmd_vel_[p]))
       mobile_cmd_vel_[p] = 0.0;
    // if (std::abs(mobile_cmd_vel_[p])<=0.01)//造成运动不稳定;
    //    mobile_cmd_vel_[p] = 0.0;
    if (mobile_cmd_vel_[p]>0.2)
       mobile_cmd_vel_[p] = 0.2;
    if(mobile_cmd_vel_[p]<-0.2)
       mobile_cmd_vel_[p] = -0.2;
  }
  

	desiredConfiguration.velocities.resize(numberOfwheel);  
	wheelCommand.joint_names.resize(numberOfwheel);

  wheelCommand.joint_names[0] = "wheel_front_left_base_link";
  wheelCommand.joint_names[1] = "wheel_front_right_base_link";

  desiredConfiguration.velocities[0] = mobile_cmd_vel_[0];//TO DO
  desiredConfiguration.velocities[1] = mobile_cmd_vel_[1];//TO DO

  wheelCommand.header.stamp = ros::Time::now();
	// wheelCommand.header.frame_id = "base_link";
  wheelCommand.points.resize(1); // only one point so far
	wheelCommand.points[0] = desiredConfiguration;
  wheelCommand.points[0].time_from_start = ros::Duration(0); // 1 ns

  //pub_mobile_vol_.publish(twist);

	pub_mobile_vol_.publish(wheelCommand);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // record data 记录数据
  // if((planTime+2)<elapsed_time_.toSec() && elapsed_time_.toSec()<=(2*planTime+2)){
    // Eigen::Matrix<double, 23, 1>  record_data;
    // record_data  <<   F_ext[0],
    //                   F_ext[1],
    //                   F_ext[2],
    //                   F_ext[3],
    //                   F_ext[4],
    //                   F_ext[5],
    //                   target_q[0],
    //                   target_q[1],
    //                   target_q[2],
    //                   position[0],
    //                   position[1],
    //                   position[2],
    //                   orientation.x(),
    //                   orientation.y(),
    //                   orientation.z(),
    //                   orientation.w(),
    //                   planed_quaternion.x(),
    //                   planed_quaternion.y(),
    //                   planed_quaternion.z(),
    //                   planed_quaternion.w(),
    //                   mobile_fed_position_[0],
    //                   mobile_fed_position_[1],
    //                   mobile_fed_position_[2];
    // logger->add("record_data", record_data);


}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
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

void CartesianImpedanceExampleController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
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

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs(); //四元数的双倍覆盖问题
  }
}

/////订阅底盘的位置，速度信息回调函数//////////////////////////////////////////////////////////////////////////////////////
/////TO DO 读取两个轮子的速度////////////////////////////////////////////////////////////////////////////////////////////
void CartesianImpedanceExampleController::MobileCallback(const nav_msgs::OdometryConstPtr& msg) {
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
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


/////关节限位距离///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CartesianImpedanceExampleController::awayFromJointLimits(const Eigen::Matrix<double, 7, 1>& q){
  
    double away_from_joint_limits_distance=0;
    Eigen::Matrix<double, 7, 1> maximum_joint_values_; 
    maximum_joint_values_ << 2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973;
    Eigen::Matrix<double, 7, 1> minimum_joint_values_;
    minimum_joint_values_ << -2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973;
    Eigen::Matrix<double, 7, 1> middle_joint_values_;
    middle_joint_values_ = (maximum_joint_values_ + minimum_joint_values_)/2;
    for(int i = 0; i < 7; i++){
    away_from_joint_limits_distance += pow((q(i)- middle_joint_values_(i))/(maximum_joint_values_(i) - minimum_joint_values_(i)),2);
    }
    away_from_joint_limits_distance=-away_from_joint_limits_distance/14;
    return away_from_joint_limits_distance;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/////关节限位距离导数，用于优化///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::awayFromJointLimitsOptimization(const Eigen::Matrix<double, 7, 1>& q, const double k_0){
    
    Eigen::Matrix<double, 7, 1> away_from_joint_limits_optimization_cmd_;
    Eigen::Matrix<double, 7, 1> maximum_joint_values_; 
    maximum_joint_values_ << 2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973;
    Eigen::Matrix<double, 7, 1> minimum_joint_values_;
    minimum_joint_values_ << -2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973;
    Eigen::Matrix<double, 7, 1> middle_joint_values_;
    middle_joint_values_ = (maximum_joint_values_ + minimum_joint_values_)/2;
    for(int i = 0; i < 7; i++)
    away_from_joint_limits_optimization_cmd_(i) = k_0*(-1.0/7.0)*(q(i)- middle_joint_values_(i))/(maximum_joint_values_(i) - minimum_joint_values_(i));
//     ROS_INFO_STREAM("q - middle_joint_values_\n" << q-middle_joint_values_);
//     ROS_INFO_STREAM("away_from_joint_limits_optimization_cmd_ \n" << away_from_joint_limits_optimization_cmd_);
    return away_from_joint_limits_optimization_cmd_;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/////rpy转qua////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Quaterniond CartesianImpedanceExampleController::rpy2qua(const Eigen::Vector3d& eulerAngle) {  // NOLINT (readability-identifier-naming)
  
  Eigen::Quaterniond target_quaternion;
  Eigen::AngleAxisd rollAngle( Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle( Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle( Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
  target_quaternion=yawAngle*pitchAngle*rollAngle;
  return target_quaternion; 
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*  
////位姿五次多项式规划函数///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 7, 4> JointVelocityExampleController::QuinticPolynomialPlanning(
    const Eigen::Quaterniond& original_quaternion,
    const Eigen::Quaterniond& desired_quaternion,
    const Eigen::Matrix<double, 4, 1>& original_position,
    const Eigen::Matrix<double, 4, 1>& desired_position,
    const double& planTime) {  // NOLINT (readability-identifier-naming)
  //"difference" quaternion 四元数差分 
  Eigen::Quaterniond temp_quaternion(original_quaternion.inverse()*desired_quaternion);
  // convert to axis angle 将四元数差分转为轴角表示方式
  Eigen::AngleAxisd diff_quaternion_angle_axis(temp_quaternion);
  // compute "orientation error" 轴角表示方式中的角度
  double diff_angle= diff_quaternion_angle_axis.angle();//两个四元数的旋转角度
  Eigen::Vector3d stable_axis;
  // compute "orientation error" 轴角表示方式中的转轴  
  stable_axis=diff_quaternion_angle_axis.axis();//两个四元数的旋转轴
  //std：：array相关学习资料 https://blog.csdn.net/qq_38410730/article/details/102802239
  std::array<double, 4> transient_aim_dp{};
  std::array<double, 4> transient_aim_d2p{};    
  transient_aim_dp={0,0,0,0};
  transient_aim_d2p={0,0,0,0};
  //设置初始点与目标点两个点的位置速度加速度信息
  Eigen::Matrix<double, 6, 4> pointSet;
  pointSet << original_position[0],original_position[1],original_position[2],original_position[3],
                    0,0,0,0,
                    0,0,0,0,
                    desired_position[0],desired_position[1],desired_position[2],diff_angle,//jiaodu
                    transient_aim_dp[0],transient_aim_dp[1],transient_aim_dp[2],transient_aim_dp[3],
                    transient_aim_d2p[0],transient_aim_d2p[1],transient_aim_d2p[2],transient_aim_d2p[3];
  Eigen::Matrix<double,6, 6> time_Martix;
  time_Martix <<  0,0,0,0,0,1,
                    0,0,0,0,1,0,
                    0,0,0,2,0,0,
                    pow(planTime,5),pow(planTime,4),pow(planTime,3),pow(planTime,2),pow(planTime,1),1,
                    5*pow(planTime,4),4*pow(planTime,3),3*pow(planTime,2),2*pow(planTime,1),1,0,
                    20*pow(planTime,3),12*pow(planTime,2),6*pow(planTime,1),2,0,0;
 Eigen::Matrix<double,6, 4> para_Matrix;
 para_Matrix.setZero();
 para_Matrix=time_Martix.inverse()*pointSet;
 // 最终输出
 Eigen::Matrix<double,7, 4> desired_matrix;
 desired_matrix.setZero();
 desired_matrix.block<6,4>(0,0) << para_Matrix;
 desired_matrix.block<1,3>(6,0) << stable_axis[0],stable_axis[1],stable_axis[2];// most primy operation is always right 
 return desired_matrix;
 }*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)