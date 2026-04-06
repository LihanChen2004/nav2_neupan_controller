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

#ifndef NAV2_NEUPAN_CONTROLLER__NEUPAN_TYPES_HPP_
#define NAV2_NEUPAN_CONTROLLER__NEUPAN_TYPES_HPP_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

namespace nav2_neupan_controller
{

// Cached geometry queried once from the Python robot object after initialization.
struct RobotInfo
{
  std::string shape{"rectangle"};
  std::string kinematics{"diff"};
  double length{0.5};
  double width{0.3};
  double wheelbase{0.3};
};

struct NeuPANWaypoint
{
  double x;
  double y;
  double theta;
  int gear;
};

// Data returned by one NeuPAN forward() call.
// All Python objects are converted to plain C++ types inside the GIL,
// so callers never touch PyObject*.
struct PlannerOutput
{
  bool stop{false};    // collision-stop requested by NeuPAN
  bool arrive{false};  // NeuPAN internal path exhausted

  geometry_msgs::msg::Twist cmd_vel;

  // Visualization data (converted from Python inside the bridge)
  std::vector<std::array<double, 3>> opt_states;    // optimized trajectory  [x,y,θ]
  std::vector<std::array<double, 3>> ref_states;    // reference trajectory  [x,y,θ]
  std::vector<std::array<double, 3>> initial_path;  // initial path           [x,y,θ]
  std::vector<std::pair<double, double>> dune_pts;  // DUNE obstacle points  [x,y]
  std::vector<std::pair<double, double>> nrmp_pts;  // NRMP obstacle points  [x,y]
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__NEUPAN_TYPES_HPP_
