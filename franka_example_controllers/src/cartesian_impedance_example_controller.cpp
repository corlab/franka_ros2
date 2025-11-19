// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>
#include <functional>
#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

#include "franka_example_controllers/pseudo_inversion.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "franka/robot_state.h"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration CartesianImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  for (int i = 1; i <= 7; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type CartesianImpedanceExampleController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // get coriolis, jacobian and robot pose from semantic model
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_array.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // read joint positions and velocities from state interfaces (assumes joint pos/vel are last)
  for (int i = 0; i < 7; ++i) {
    const auto& position_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i));
    const auto& velocity_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i + 1));
    // these optionals should be valid as the controller requested them
    Eigen::Index idx = i;
    // map into local q/dq
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_local(reinterpret_cast<double*>(nullptr));
    (void)q_local;  // silence unused - we'll directly set via members if present
  }

  // If franka::RobotState is available, map q/dq/tau_J_d from it; otherwise read from state interfaces
  Eigen::Matrix<double, 7, 1> q, dq, tau_J_d;
  q.setZero();
  dq.setZero();
  tau_J_d.setZero();
  if (robot_state_ptr_ != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_map(robot_state_ptr_->q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_map(robot_state_ptr_->dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_map(robot_state_ptr_->tau_J_d.data());
    q = q_map;
    dq = dq_map;
    tau_J_d = tau_map;
  } else {
    for (int i = 0; i < 7; ++i) {
      const auto& position_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i));
      const auto& velocity_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i + 1));
      q(i) = position_interface.get_optional().value();
      dq(i) = velocity_interface.get_optional().value();
    }
  }

  // compute error to desired pose
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) - (2.0 * sqrt(nullspace_stiffness_)) * dq);
  tau_d << tau_task + tau_nullspace + coriolis;

  // saturate torque rate
  Eigen::Matrix<double, 7, 1> tau_d_eigen = tau_d;
  tau_d_eigen = saturateTorqueRate(tau_d_eigen, tau_J_d);

  for (int i = 0; i < 7; ++i) {
    if (!command_interfaces_[i].set_value(tau_d_eigen(i))) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  // filter parameters and update targets
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> guard(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

CallbackReturn CartesianImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      return CallbackReturn::ERROR;
    }

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_ + "/robot_state");

    auto_declare<double>("translational_stiffness", 150.0);
    auto_declare<double>("translational_stiffness_x", 150.0);
    auto_declare<double>("translational_stiffness_y", 150.0);
    auto_declare<double>("translational_stiffness_z", 150.0);
    auto_declare<double>("rotational_stiffness", 10.0);
    auto_declare<double>("nullspace_stiffness", 20.0);

    // parameter callback to mimic dynamic_reconfigure
    params_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto& p : params) {
            const auto& name = p.get_name();
            try {
              if (name == "translational_stiffness") {
                double v = p.as_double();
                cartesian_stiffness_target_.topLeftCorner(3, 3) << v * Eigen::Matrix3d::Identity();
                cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * sqrt(v) * Eigen::Matrix3d::Identity();
              } else if (name == "translational_stiffness_x") {
                double v = p.as_double();
                cartesian_stiffness_target_(0, 0) = v;
                cartesian_damping_target_(0, 0) = 2.0 * sqrt(v);
              } else if (name == "translational_stiffness_y") {
                double v = p.as_double();
                cartesian_stiffness_target_(1, 1) = v;
                cartesian_damping_target_(1, 1) = 2.0 * sqrt(v);
              } else if (name == "translational_stiffness_z") {
                double v = p.as_double();
                cartesian_stiffness_target_(2, 2) = v;
                cartesian_damping_target_(2, 2) = 2.0 * sqrt(v);
              } else if (name == "rotational_stiffness") {
                double v = p.as_double();
                cartesian_stiffness_target_.bottomRightCorner(3, 3) << v * Eigen::Matrix3d::Identity();
                cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(v) * Eigen::Matrix3d::Identity();
              } else if (name == "nullspace_stiffness") {
                nullspace_stiffness_target_ = p.as_double();
              }
            } catch (...) {
              result.successful = false;
              result.reason = std::string("Invalid parameter type for ") + name;
              return result;
            }
          }
          return result;
        });

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  double t_stiff = get_node()->get_parameter("translational_stiffness").as_double();
  double t_stiff_x = get_node()->get_parameter("translational_stiffness_x").as_double();
  double t_stiff_y = get_node()->get_parameter("translational_stiffness_y").as_double();
  double t_stiff_z = get_node()->get_parameter("translational_stiffness_z").as_double();
  double r_stiff = get_node()->get_parameter("rotational_stiffness").as_double();
  nullspace_stiffness_target_ = get_node()->get_parameter("nullspace_stiffness").as_double();

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3) << t_stiff * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_(0,0) = t_stiff_x;
  cartesian_stiffness_target_(1,1) = t_stiff_y;
  cartesian_stiffness_target_(2,2) = t_stiff_z;
  cartesian_stiffness_target_.bottomRightCorner(3, 3) << r_stiff * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * sqrt(t_stiff) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_(0,0) = 2.0 * sqrt(t_stiff_x);
  cartesian_damping_target_(1,1) = 2.0 * sqrt(t_stiff_y);
  cartesian_damping_target_(2,2) = 2.0 * sqrt(t_stiff_z);
  cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(r_stiff) * Eigen::Matrix3d::Identity();

  // subscription
  using std::placeholders::_1;
  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "equilibrium_pose", rclcpp::SystemDefaultsQoS(), std::bind(&CartesianImpedanceExampleController::equilibriumPoseCallback, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // find franka robot_state loaned interface to read tau_J_d
  std::string franka_state_iface_name = arm_id_ + "/robot_state";
  auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& iface) {
    return iface.get().get_name() == franka_state_iface_name;
  });
  if (it != state_interfaces_.end()) {
    // bit_cast the raw pointer stored in the optional
    auto raw = (*it).get().get_optional().value();
    robot_state_ptr_ = bit_cast<franka::RobotState*>(raw);
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Could not find franka robot_state interface to read tau_J_d");
    robot_state_ptr_ = nullptr;
  }

  // initialize q and dq from state interfaces (last entries)
  for (int i = 0; i < 7; ++i) {
    const auto& position_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i));
    const auto& velocity_interface = state_interfaces_.at(static_cast<size_t>(state_interfaces_.size() - 2 * 7 + 2 * i + 1));
    q_d_nullspace_(i) = position_interface.get_optional().value();
    // we could also set dq_ if stored locally
  }

  // get initial pose via semantic model
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(pose_array.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;

  dq_filtered_.setZero();
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  robot_state_ptr_ = nullptr;
  if (params_callback_handle_) params_callback_handle_.reset();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w;
  if (last_orientation.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)
