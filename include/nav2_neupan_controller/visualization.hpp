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

#ifndef NAV2_NEUPAN_CONTROLLER__VISUALIZATION_HPP_
#define NAV2_NEUPAN_CONTROLLER__VISUALIZATION_HPP_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "nav2_neupan_controller/neupan_types.hpp"
#include "nav2_neupan_controller/parameter_handler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_neupan_controller
{

// Owns all NeuPAN visualization publishers.
// Accepts plain C++ types (no Python dependency).
class Visualizer
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void cleanup();
  void activate();
  void deactivate();

  void publish(
    const PlannerOutput & output, const RobotInfo & robot_info, const Parameters & params,
    const std::array<double, 3> & robot_state, const std::string & frame_id,
    const rclcpp::Time & stamp);

private:
  nav_msgs::msg::Path statesToPath(
    const std::vector<std::array<double, 3>> & states, const std::string & frame_id,
    const rclcpp::Time & stamp);

  visualization_msgs::msg::MarkerArray pointsToMarkers(
    const std::vector<std::pair<double, double>> & points, float r, float g, float b, double size,
    const std::string & frame_id, const rclcpp::Time & stamp);

  visualization_msgs::msg::Marker buildRobotMarker(
    const RobotInfo & robot_info, const Parameters & params,
    const std::array<double, 3> & robot_state, const std::string & frame_id,
    const rclcpp::Time & stamp);

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr ref_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr initial_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr dune_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr nrmp_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__VISUALIZATION_HPP_
