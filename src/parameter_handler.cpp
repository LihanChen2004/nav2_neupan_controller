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

#include "nav2_neupan_controller/parameter_handler.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

namespace nav2_neupan_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & plugin_name,
  const rclcpp::Logger & logger)
: node_(node), plugin_name_(plugin_name), logger_(logger)
{
  // Velocity limits
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.5));

  // Robot / NeuPAN config
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_type", rclcpp::ParameterValue(std::string("acker")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".neupan_config_path", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dune_model_path", rclcpp::ParameterValue(std::string("")));

  // Visualization markers
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".marker_size", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".marker_z", rclcpp::ParameterValue(1.0));

  // Read all parameters
  node->get_parameter(plugin_name_ + ".max_linear_velocity", params_.max_linear_velocity);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", params_.max_angular_velocity);
  node->get_parameter(plugin_name_ + ".robot_type", params_.robot_type);
  node->get_parameter(plugin_name_ + ".neupan_config_path", params_.neupan_config_path);
  node->get_parameter(plugin_name_ + ".dune_model_path", params_.dune_model_path);
  node->get_parameter(plugin_name_ + ".marker_size", params_.marker_size);
  node->get_parameter(plugin_name_ + ".marker_z", params_.marker_z);

  params_.max_linear_velocity_initial = params_.max_linear_velocity;
  params_.max_angular_velocity_initial = params_.max_angular_velocity;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::dynamicParametersCallback, this, std::placeholders::_1));
}

ParameterHandler::~ParameterHandler()
{
  auto node = node_.lock();
  if (node && dyn_params_handler_) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".max_linear_velocity") {
        params_.max_linear_velocity = std::max(0.0, parameter.as_double());
        params_.max_linear_velocity_initial = params_.max_linear_velocity;
      } else if (name == plugin_name_ + ".max_angular_velocity") {
        params_.max_angular_velocity = std::max(0.0, parameter.as_double());
        params_.max_angular_velocity_initial = params_.max_angular_velocity;
      } else if (name == plugin_name_ + ".marker_size") {
        params_.marker_size = std::max(0.001, parameter.as_double());
      } else if (name == plugin_name_ + ".marker_z") {
        params_.marker_z = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".robot_type") {
        params_.robot_type = parameter.as_string();
      } else if (name == plugin_name_ + ".neupan_config_path") {
        params_.neupan_config_path = parameter.as_string();
      } else if (name == plugin_name_ + ".dune_model_path") {
        params_.dune_model_path = parameter.as_string();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_neupan_controller
