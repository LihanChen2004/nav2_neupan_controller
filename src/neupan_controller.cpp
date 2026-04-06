// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_neupan_controller/neupan_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_neupan_controller/obstacle_extractor.hpp"
#include "nav2_neupan_controller/python_bridge.hpp"
#include "nav2_neupan_controller/visualization.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace nav2_neupan_controller
{

// ─────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────

NeuPANController::~NeuPANController() = default;

void NeuPANController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node->get_logger();

  param_handler_ = std::make_unique<ParameterHandler>(node, plugin_name_, logger_);
  path_handler_ = std::make_unique<PathHandler>();
  path_handler_->initialize(tf_, costmap_ros_->getGlobalFrameID(), logger_);

  bridge_ = std::make_unique<PythonBridge>(logger_);
  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(node);

  RCLCPP_INFO(
    logger_, "NeuPAN Controller configured: robot_type=%s",
    param_handler_->getParams()->robot_type.c_str());

  const auto * p = param_handler_->getParams();
  if (!bridge_->initialize(p->neupan_config_path, p->dune_model_path)) {
    throw nav2_core::ControllerException("Failed to initialize Python/NeuPAN in configure()");
  }
}

void NeuPANController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up NeuPAN Controller: %s", plugin_name_.c_str());
  bridge_.reset();
  visualizer_->cleanup();
  visualizer_.reset();
  path_handler_.reset();
  param_handler_.reset();
}

void NeuPANController::activate()
{
  RCLCPP_INFO(logger_, "Activating NeuPAN Controller: %s", plugin_name_.c_str());
  visualizer_->activate();
}

void NeuPANController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating NeuPAN Controller: %s", plugin_name_.c_str());
  visualizer_->deactivate();
}

// ─────────────────────────────────────────────────────────────
//  Plan handling
// ─────────────────────────────────────────────────────────────

void NeuPANController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!path_handler_) return;
  path_handler_->setPlan(path);
  if (path.poses.empty()) return;

  // Cache goal state for arrive-recovery
  const auto & goal_pose = path.poses.back().pose;
  last_goal_state_ = {
    goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation)};
  last_goal_valid_ = true;

  if (bridge_ && bridge_->isInitialized()) {
    bridge_->callReset();
    if (!bridge_->setInitialPath(path_handler_->getNeuPANPath())) {
      RCLCPP_WARN(logger_, "Failed to sync global plan to NeuPAN");
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  Main control loop
// ─────────────────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped NeuPANController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  if (!param_handler_ || !path_handler_ || !bridge_) {
    RCLCPP_ERROR(logger_, "Controller not configured correctly");
    return cmd_vel;
  }

  std::lock_guard<std::mutex> lock(param_handler_->getMutex());
  const Parameters * params = param_handler_->getParams();

  const nav_msgs::msg::Path global_plan = path_handler_->getPlan();
  if (global_plan.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "No global plan available");
    return cmd_vel;
  }

  if (!bridge_->isInitialized()) {
    RCLCPP_WARN(logger_, "Python not initialized, stopping robot");
    return cmd_vel;
  }

  const std::array<double, 3> robot_state = {
    pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)};

  const auto obstacles = extract(*costmap_ros_);

  if (obstacles.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 1000,
      "No obstacle points from costmap — path tracking only");
  }

  PlannerOutput output;
  if (!bridge_->callForward(robot_state, obstacles, params->robot_type, output)) {
    RCLCPP_WARN(logger_, "NeuPAN planner failed, stopping robot");
    return cmd_vel;
  }

  // When NeuPAN's internal path is exhausted but Nav2 hasn't declared goal reached yet,
  // regenerate the path from current pose to the cached goal.
  if (output.arrive && last_goal_valid_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 500, "NeuPAN path arrived — refreshing path to goal");
    bridge_->updatePathFromGoal(robot_state, last_goal_state_);
  }

  // Clamp velocity limits
  cmd_vel.twist = output.cmd_vel;
  cmd_vel.twist.linear.x =
    std::clamp(cmd_vel.twist.linear.x, -params->max_linear_velocity, params->max_linear_velocity);
  cmd_vel.twist.linear.y =
    std::clamp(cmd_vel.twist.linear.y, -params->max_linear_velocity, params->max_linear_velocity);
  cmd_vel.twist.angular.z = std::clamp(
    cmd_vel.twist.angular.z, -params->max_angular_velocity, params->max_angular_velocity);

  // Publish visualization
  auto node = node_.lock();
  const rclcpp::Time stamp = node ? node->now() : rclcpp::Time(0);
  visualizer_->publish(
    output, bridge_->robotInfo(), *params, robot_state, costmap_ros_->getGlobalFrameID(), stamp);

  return cmd_vel;
}

void NeuPANController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (!param_handler_) return;
  std::lock_guard<std::mutex> lock(param_handler_->getMutex());
  auto * p = param_handler_->getParams();

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    p->max_linear_velocity = p->max_linear_velocity_initial;
    p->max_angular_velocity = p->max_angular_velocity_initial;
  } else if (percentage) {
    p->max_linear_velocity = std::max(p->max_linear_velocity_initial * speed_limit / 100.0, 0.0);
    p->max_angular_velocity = p->max_angular_velocity_initial * speed_limit / 100.0;
  } else {
    p->max_linear_velocity = std::max(speed_limit, 0.0);
    if (p->max_linear_velocity_initial > 1e-9) {
      p->max_angular_velocity = std::max(
        p->max_angular_velocity_initial * speed_limit / p->max_linear_velocity_initial, 0.0);
    } else {
      p->max_angular_velocity = std::max(speed_limit, 0.0);
    }
  }
}

}  // namespace nav2_neupan_controller

PLUGINLIB_EXPORT_CLASS(nav2_neupan_controller::NeuPANController, nav2_core::Controller)
