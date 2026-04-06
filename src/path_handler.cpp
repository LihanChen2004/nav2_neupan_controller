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

#include "nav2_neupan_controller/path_handler.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"

namespace nav2_neupan_controller
{

void PathHandler::initialize(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & target_frame,
  const rclcpp::Logger & logger)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tf_buffer_ = tf_buffer;
  target_frame_ = target_frame;
  logger_ = logger;

  RCLCPP_INFO(
    logger_, "PathHandler initialized: target_frame=%s",
    target_frame_.empty() ? "unset" : target_frame_.c_str());
}

void PathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If initialized with TF buffer, transform path to target frame
  if (tf_buffer_ && !target_frame_.empty()) {
    global_plan_ = transformPath(path);
  } else {
    global_plan_ = path;
  }
}

nav_msgs::msg::Path PathHandler::getPlan() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return global_plan_;
}

std::vector<NeuPANWaypoint> PathHandler::getNeuPANPath() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<NeuPANWaypoint> path;
  if (global_plan_.poses.empty()) {
    return path;
  }

  path.resize(global_plan_.poses.size());

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & pose = global_plan_.poses[i].pose;
    double theta = tf2::getYaw(pose.orientation);

    // If orientation is identity (planner didn't set it), use path tangent
    if (
      std::abs(pose.orientation.x) < 1e-6 && std::abs(pose.orientation.y) < 1e-6 &&
      std::abs(pose.orientation.z) < 1e-6) {
      if (i + 1 < global_plan_.poses.size()) {
        const double dx = global_plan_.poses[i + 1].pose.position.x - pose.position.x;
        const double dy = global_plan_.poses[i + 1].pose.position.y - pose.position.y;
        theta = std::atan2(dy, dx);
      }
    }

    path[i].x = pose.position.x;
    path[i].y = pose.position.y;
    path[i].theta = theta;

    if (i < global_plan_.poses.size() - 1) {
      const double dx = global_plan_.poses[i + 1].pose.position.x - pose.position.x;
      const double dy = global_plan_.poses[i + 1].pose.position.y - pose.position.y;

      if (dx * std::cos(theta) + dy * std::sin(theta) >= 0.0) {
        path[i].gear = 1;
      } else {
        path[i].gear = -1;
      }
    }
  }

  if (path.size() > 1) {
    path.back().gear = path[path.size() - 2].gear;
  } else if (!path.empty()) {
    path.back().gear = 1;
  }

  return path;
}

nav_msgs::msg::Path PathHandler::transformPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return path;
  }

  nav_msgs::msg::Path transformed_path = path;
  transformed_path.header.frame_id = target_frame_;

  try {
    for (auto & pose_stamped : transformed_path.poses) {
      // Skip if already in target frame
      if (pose_stamped.header.frame_id == target_frame_) {
        continue;
      }

      // Transform the single pose
      geometry_msgs::msg::PoseStamped transformed_pose =
        tf_buffer_->transform(pose_stamped, target_frame_);
      pose_stamped = transformed_pose;
    }

    RCLCPP_DEBUG(
      logger_, "Transformed path from '%s' to '%s' (%zu poses)", path.header.frame_id.c_str(),
      target_frame_.c_str(), path.poses.size());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "Failed to transform path from '%s' to '%s': %s", path.header.frame_id.c_str(),
      target_frame_.c_str(), ex.what());
    // Return original path if transformation fails
    return path;
  }

  return transformed_path;
}

}  // namespace nav2_neupan_controller
