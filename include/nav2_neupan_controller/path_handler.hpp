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

#ifndef NAV2_NEUPAN_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_NEUPAN_CONTROLLER__PATH_HANDLER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_neupan_controller/neupan_types.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_ros/buffer.hpp"

namespace nav2_neupan_controller
{

class PathHandler
{
public:
  PathHandler() = default;
  ~PathHandler() = default;

  /**
   * Initialize the path handler with TF buffer and target frame for coordinate transformation.
   * Must be called before setPlan if coordinate transformation is needed.
   *
   * @param tf_buffer Shared pointer to TF2 buffer for coordinate transformations
   * @param target_frame Target coordinate frame (e.g., costmap global frame)
   * @param logger ROS logger for debug/error messages
   */
  void initialize(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & target_frame,
    const rclcpp::Logger & logger);

  /**
   * Set the global plan (will be transformed to target frame if initialized).
   *
   * @param path Input path from path planner (typically in "map" frame)
   */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
   * Get the global plan in the target frame.
   *
   * @return ROS Path message (already transformed if initialized)
   */
  nav_msgs::msg::Path getPlan() const;

  /**
   * Get the path in NeuPAN format with gear information.
   *
   * @return Vector of NeuPAN waypoints with position, orientation, and gear
   */
  std::vector<NeuPANWaypoint> getNeuPANPath() const;

private:
  mutable std::mutex mutex_;
  nav_msgs::msg::Path global_plan_;

  // Transform-related members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathHandler")};

  /**
   * Transform path from its frame_id to target_frame.
   * Called internally by setPlan if transformation is needed.
   *
   * @param path Input path to transform
   * @return Transformed path (or original if transformation fails)
   */
  nav_msgs::msg::Path transformPath(const nav_msgs::msg::Path & path);
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__PATH_HANDLER_HPP_
