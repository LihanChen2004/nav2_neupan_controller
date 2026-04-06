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

#ifndef NAV2_NEUPAN_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_NEUPAN_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_neupan_controller
{

struct Parameters
{
  // Velocity limits
  double max_linear_velocity;
  double max_angular_velocity;
  double max_linear_velocity_initial;
  double max_angular_velocity_initial;

  // Robot type: "diff", "acker", "omni"
  std::string robot_type;

  // NeuPAN config
  std::string neupan_config_path;
  std::string dune_model_path;

  // Visualization marker sizing
  double marker_size;
  double marker_z;
};

class ParameterHandler
{
public:
  ParameterHandler(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & plugin_name,
    const rclcpp::Logger & logger);

  ~ParameterHandler();

  std::mutex & getMutex() { return mutex_; }

  Parameters * getParams() { return &params_; }

protected:
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("NeuPANParameterHandler")};
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__PARAMETER_HANDLER_HPP_
