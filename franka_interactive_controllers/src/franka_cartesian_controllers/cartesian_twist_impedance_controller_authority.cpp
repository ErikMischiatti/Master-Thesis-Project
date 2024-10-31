// This code was derived from franka_example controllers
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license.
// Current development and modification of this code by Nadia Figueroa (MIT) 2021.

#include <cartesian_twist_impedance_controller_authority.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>

namespace franka_interactive_controllers {

bool CartesianTwistImpedanceControllerAuthority::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_desired_twist_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_twist", 20, &CartesianTwistImpedanceControllerAuthority::desiredTwistCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_alpha_h_ = node_handle.subscribe(
      "/alpha_h", 20, &CartesianTwistImpedanceControllerAuthority::alphaHCallback, this);


  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianTwistImpedanceControllerAuthority: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianTwistImpedanceControllerAuthority: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceControllerAuthority: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceControllerAuthority: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceControllerAuthority: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceControllerAuthority: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceControllerAuthority: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianTwistImpedanceControllerAuthority: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  // Initializing variables
  position_d_.setZero();
  velocity_d_.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_stiffness_.topLeftCorner(3, 3)
          << K_H_trans * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3)
          << K_H_rot * Eigen::Matrix3d::Identity();
  cartesian_damping_.topLeftCorner(3, 3)
          << 2*sqrt(K_H_trans) * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3)
          << 2*sqrt(K_H_rot) * Eigen::Matrix3d::Identity();

  return true;
}

void CartesianTwistImpedanceControllerAuthority::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set desired point to current state
  position_d_ = initial_transform.translation();
  velocity_d_ << 20, 0, 0;
}

void CartesianTwistImpedanceControllerAuthority::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
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
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.setZero();
  error.head(3) << position - position_d_;

  // // orientation error
  // if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
  //   orientation.coeffs() << -orientation.coeffs();
  // }
  // // "difference" quaternion
  // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // // Transform to base frame
  // error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  ROS_WARN_STREAM_THROTTLE(0.5, "forces:"           << -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  position_d_ = position + velocity_d_.head(3) * dt_;
  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // // get state variables
  // franka::RobotState robot_state = state_handle_->getRobotState();
  // std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  // std::array<double, 42> jacobian_array =
  //     model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // // convert to Eigen
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
  //     robot_state.tau_J_d.data());
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Eigen::Vector3d position(transform.translation());

  // // Current and Desired EE velocity
  // Eigen::Matrix<double, 6, 1> velocity;
  // velocity << jacobian * dq;

  // position_d_ = position + velocity_d_.head(3) * dt_;

  // //////////////////////////////////////////////////////////////////////////////////////////////////
  // //////////////////////              COMPUTING TASK CONTROL TORQUE           //////////////////////
  // //////////////////////////////////////////////////////////////////////////////////////////////////
  // // compute control
  // // allocate variables
  // Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_tool(7);

  // //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //++++++++++++++ CLASSICAL IMPEDANCE CONTROL FOR CARTESIAN COMMAND ++++++++++++++//
  // //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  // // Compute task-space errors
  // // --- Pose Error  --- //     
  // Eigen::Matrix<double, 6, 1> pose_error;
  // pose_error.setZero();

  // ROS_WARN_STREAM_THROTTLE(0.5, "position current:"         << position);
  // ROS_WARN_STREAM_THROTTLE(0.5, "position desired:"         << position_d_);
  // pose_error.head(3) << position_d_ - position;
  // ROS_WARN_STREAM_THROTTLE(0.5, "Pose Error:"         << pose_error);
  
  // // --- Velocity Error  --- //
  // Eigen::Matrix<double, 6, 1> vel_error;
  // vel_error.setZero();

  // ROS_WARN_STREAM_THROTTLE(0.5, "vel current:"         << velocity);
  // ROS_WARN_STREAM_THROTTLE(0.5, "vel desired:"         << velocity_d_);
  // vel_error.head(3) << velocity.head(3);
  // ROS_WARN_STREAM_THROTTLE(0.5, "vel Error:"         << vel_error);

  // // Computing control torque from cartesian pose error from integrated velocity command
  // Eigen::VectorXd     F_ee_des_;
  // F_ee_des_.resize(6);
  // F_ee_des_ << cartesian_stiffness_ * pose_error + cartesian_damping_ * vel_error;
  // ROS_WARN_STREAM_THROTTLE(0.5, "F_ee_des_:"           << F_ee_des_.head(3));

  // tau_task << jacobian.transpose() * F_ee_des_;

  // tau_d << tau_task + coriolis;
  // // Saturate torque rate to avoid discontinuities
  // tau_d << saturateTorqueRate(tau_d, tau_J_d);
  // for (size_t i = 0; i < 7; ++i) {
  //   joint_handles_[i].setCommand(tau_d(i));
  // }
}

Eigen::Matrix<double, 7, 1> CartesianTwistImpedanceControllerAuthority::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  ROS_WARN_STREAM_THROTTLE(0.5, "tau raw:"           << tau_d_calculated);
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  ROS_WARN_STREAM_THROTTLE(0.5, "tau saturated:"           << tau_d_saturated);
  return tau_d_saturated;
}

void CartesianTwistImpedanceControllerAuthority::desiredTwistCallback(
    const geometry_msgs::TwistConstPtr& msg) {

  // velocity_d_      << msg->linear.x, msg->linear.y, msg->linear.z;
  // last_cmd_time    = ros::Time::now().toSec();


  // franka::RobotState robot_state = state_handle_->getRobotState();
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Eigen::Vector3d position(transform.translation());

  // position_d_target_  << position + velocity_d_*dt_;
}


void CartesianTwistImpedanceControllerAuthority::alphaHCallback(const std_msgs::Float64::ConstPtr& msg) {
  double alpha_h = msg->data;
  K_trans = (1-alpha_h)*K_R_trans + alpha_h*K_H_trans;
  D_trans = 2*sqrt(K_trans);
  K_rot = (1-alpha_h)*K_R_rot + alpha_h*K_H_rot;
  D_rot = 2*sqrt(K_rot);

  cartesian_stiffness_.topLeftCorner(3, 3)
          << K_trans * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3)
          << K_rot * Eigen::Matrix3d::Identity();

  cartesian_damping_.topLeftCorner(3, 3)
          << D_trans * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3)
          << D_rot * Eigen::Matrix3d::Identity();
}
}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::CartesianTwistImpedanceControllerAuthority,
                       controller_interface::ControllerBase)

