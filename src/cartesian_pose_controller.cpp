#include <fr3_controllers/cartesian_pose_controller.hpp>
#include <fr3_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

controller_interface::return_type CartesianPoseController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/)
{

// On first update, record the initial pose and robot time.
if (initialization_flag_) {
  std::tie(orientation_, position_) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
 
  initial_orientation_ = orientation_;
  initial_position_ = position_;

  initial_robot_time_ = state_interfaces_.back().get_value();
  elapsed_time_ = 0.0;
  // Ensure we start with phase 1.
  reached_A_ = false;

  initialization_flag_ = false;
} 
else {
  robot_time_ = state_interfaces_.back().get_value();
  elapsed_time_ = robot_time_ - initial_robot_time_;
}

// --- Transform A ---
Eigen::Matrix3d rotA;
rotA << 0.0,  1.0, 0.0,
        0.0,  0.0, 1.0,
        1.0,  0.0, 0.0;
Eigen::Quaterniond qA(rotA);
Eigen::Vector3d posA(0.5, -0.1, 0.1);

// --- Transform B ---
Eigen::Quaterniond qB = qA;
Eigen::Vector3d posB(0.5, 0.1, 0.1);

// === Timing Parameters (in seconds) ===
const double total_time_A = 3.0;  // Duration to reach transformA.
const double wait_time_A  = 5.0;  // Hold time at transformA.
const double total_time_B = 2.0;  // Duration for transition from A to B.

Eigen::Quaterniond new_orientation;
Eigen::Vector3d new_position;

// === Phase 1: Interpolation to Transform A ===
if (!reached_A_) {
  double t_A = std::min(1.0, elapsed_time_ / total_time_A);
  // Cosine interpolation for smooth transition.
  t_A = 0.5 * (1.0 - std::cos(t_A * M_PI));
  // Slerp for quaternion interpolation.
  new_orientation = initial_orientation_.slerp(t_A, qA);
  // Linear interpolation for translation.
  new_position = (1 - t_A) * initial_position_ + t_A * posA;

  if (elapsed_time_ >= total_time_A) {
    reached_A_ = true;
    // Reset timer for the hold phase.
    initial_robot_time_ = robot_time_;
    elapsed_time_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Reached transformA. Entering hold phase.");
  }
}
// === Phase 2: Hold at Transform A ===
else if (elapsed_time_ < wait_time_A) {
  new_orientation = qA;
  new_position = posA;
}
// === Phase 3: Transition from Transform A to Transform B ===
else {
  double t_B = std::min(1.0, (elapsed_time_ - wait_time_A) / total_time_B);
  t_B = 0.5 * (1.0 - std::cos(t_B * M_PI));
  new_orientation = qA.slerp(t_B, qB);
  new_position = (1 - t_B) * posA + t_B * posB;

  if (t_B >= 1.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Reached transformB.");
  }
}

if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
  return controller_interface::return_type::OK;
} else {
  RCLCPP_FATAL(get_node()->get_logger(),
               "Set command failed. Did you activate the elbow command interface?");
  return controller_interface::return_type::ERROR;
}
}

CallbackReturn CartesianPoseController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::CartesianPoseController,
                       controller_interface::ControllerInterface)
