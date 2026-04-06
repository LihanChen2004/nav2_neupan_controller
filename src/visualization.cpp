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

#include "nav2_neupan_controller/visualization.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_neupan_controller
{

// ─────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────

void Visualizer::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  plan_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("neupan_plan", rclcpp::SystemDefaultsQoS());
  ref_state_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("neupan_ref_state", rclcpp::SystemDefaultsQoS());
  initial_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("neupan_initial_path", rclcpp::SystemDefaultsQoS());
  dune_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "dune_point_markers", rclcpp::SystemDefaultsQoS());
  nrmp_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "nrmp_point_markers", rclcpp::SystemDefaultsQoS());
  robot_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "robot_marker", rclcpp::SystemDefaultsQoS());
}

void Visualizer::cleanup()
{
  plan_pub_.reset();
  ref_state_pub_.reset();
  initial_path_pub_.reset();
  dune_pub_.reset();
  nrmp_pub_.reset();
  robot_pub_.reset();
}

void Visualizer::activate()
{
  if (plan_pub_) plan_pub_->on_activate();
  if (ref_state_pub_) ref_state_pub_->on_activate();
  if (initial_path_pub_) initial_path_pub_->on_activate();
  if (dune_pub_) dune_pub_->on_activate();
  if (nrmp_pub_) nrmp_pub_->on_activate();
  if (robot_pub_) robot_pub_->on_activate();
}

void Visualizer::deactivate()
{
  if (plan_pub_) plan_pub_->on_deactivate();
  if (ref_state_pub_) ref_state_pub_->on_deactivate();
  if (initial_path_pub_) initial_path_pub_->on_deactivate();
  if (dune_pub_) dune_pub_->on_deactivate();
  if (nrmp_pub_) nrmp_pub_->on_deactivate();
  if (robot_pub_) robot_pub_->on_deactivate();
}

// ─────────────────────────────────────────────────────────────
//  Publish
// ─────────────────────────────────────────────────────────────

void Visualizer::publish(
  const PlannerOutput & output, const RobotInfo & robot_info, const Parameters & params,
  const std::array<double, 3> & robot_state, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (plan_pub_ && plan_pub_->is_activated()) {
    plan_pub_->publish(statesToPath(output.opt_states, frame_id, stamp));
  }

  if (ref_state_pub_ && ref_state_pub_->is_activated()) {
    ref_state_pub_->publish(statesToPath(output.ref_states, frame_id, stamp));
  }

  if (initial_path_pub_ && initial_path_pub_->is_activated()) {
    initial_path_pub_->publish(statesToPath(output.initial_path, frame_id, stamp));
  }

  if (dune_pub_ && dune_pub_->is_activated()) {
    // purple: rgb(160, 32, 240)
    dune_pub_->publish(pointsToMarkers(
      output.dune_pts, 160.0f / 255.0f, 32.0f / 255.0f, 240.0f / 255.0f, params.marker_size,
      frame_id, stamp));
  }

  if (nrmp_pub_ && nrmp_pub_->is_activated()) {
    // orange: rgb(255, 128, 0)
    nrmp_pub_->publish(pointsToMarkers(
      output.nrmp_pts, 1.0f, 128.0f / 255.0f, 0.0f, params.marker_size, frame_id, stamp));
  }

  if (robot_pub_ && robot_pub_->is_activated() && robot_info.shape == "rectangle") {
    robot_pub_->publish(buildRobotMarker(robot_info, params, robot_state, frame_id, stamp));
  }
}

// ─────────────────────────────────────────────────────────────
//  Conversion helpers
// ─────────────────────────────────────────────────────────────

nav_msgs::msg::Path Visualizer::statesToPath(
  const std::vector<std::array<double, 3>> & states, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(states.size());

  for (const auto & s : states) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.header.stamp = stamp;
    ps.pose.position.x = s[0];
    ps.pose.position.y = s[1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s[2]);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  return path;
}

visualization_msgs::msg::MarkerArray Visualizer::pointsToMarkers(
  const std::vector<std::pair<double, double>> & points, float r, float g, float b, double size,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.reserve(points.size());

  const float msize = static_cast<float>(size);
  for (size_t i = 0; i < points.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = msize;
    m.color.a = 1.0f;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.pose.position.x = points[i].first;
    m.pose.position.y = points[i].second;
    m.pose.position.z = 0.3;
    m.pose.orientation.w = 1.0;
    arr.markers.push_back(m);
  }
  return arr;
}

visualization_msgs::msg::Marker Visualizer::buildRobotMarker(
  const RobotInfo & robot_info, const Parameters & params,
  const std::array<double, 3> & robot_state, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = robot_info.length;
  m.scale.y = robot_info.width;
  m.scale.z = params.marker_z;
  m.color.a = 1.0f;
  m.color.g = 1.0f;  // green

  const double x = robot_state[0];
  const double y = robot_state[1];
  const double theta = robot_state[2];

  double cx = x, cy = y;
  if (robot_info.kinematics == "acker") {
    // Shift marker centre to the geometric centre of the body
    const double diff_len = (robot_info.length - robot_info.wheelbase) / 2.0;
    cx = x + diff_len * std::cos(theta);
    cy = y + diff_len * std::sin(theta);
  }

  m.pose.position.x = cx;
  m.pose.position.y = cy;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  m.pose.orientation = tf2::toMsg(q);
  return m;
}

}  // namespace nav2_neupan_controller
