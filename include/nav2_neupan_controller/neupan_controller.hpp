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

#ifndef NAV2_NEUPAN_CONTROLLER__NEUPAN_CONTROLLER_HPP_
#define NAV2_NEUPAN_CONTROLLER__NEUPAN_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_neupan_controller/parameter_handler.hpp"
#include "nav2_neupan_controller/path_handler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_neupan_controller
{

// Forward declarations — full types only needed in neupan_controller.cpp.
class PythonBridge;
class Visualizer;

// Thin orchestrator: lifecycle management + control loop.
// All Python interaction is delegated to PythonBridge.
// All visualization is delegated to Visualizer.
// Obstacle extraction is handled by ObstacleExtractor.
class NeuPANController : public nav2_core::Controller
{
public:
  NeuPANController() = default;
  ~NeuPANController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("NeuPANController")};

  std::unique_ptr<ParameterHandler> param_handler_;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<PythonBridge> bridge_;
  std::unique_ptr<Visualizer> visualizer_;

  // Cached goal state for arrive-recovery (updated on every setPlan call)
  std::array<double, 3> last_goal_state_{};
  bool last_goal_valid_{false};
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__NEUPAN_CONTROLLER_HPP_
