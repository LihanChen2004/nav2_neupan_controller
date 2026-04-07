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

#ifndef NAV2_NEUPAN_CONTROLLER__OBSTACLE_EXTRACTOR_HPP_
#define NAV2_NEUPAN_CONTROLLER__OBSTACLE_EXTRACTOR_HPP_

#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_neupan_controller
{

// Extracts obstacle points from the active costmap and returns them in map frame.
// Obstacle cells with cost >= INSCRIBED_INFLATED_OBSTACLE are included.

std::vector<std::pair<double, double>> extract(nav2_costmap_2d::Costmap2DROS & costmap_ros);

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__OBSTACLE_EXTRACTOR_HPP_
