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

#include "nav2_neupan_controller/obstacle_extractor.hpp"

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_neupan_controller
{

std::vector<std::pair<double, double>> extract(nav2_costmap_2d::Costmap2DROS & costmap_ros)
{
  std::vector<std::pair<double, double>> points;

  auto * costmap = costmap_ros.getCostmap();
  if (!costmap) return points;

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  if (size_x == 0 || size_y == 0) return points;

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) continue;
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

      double world_x = 0.0, world_y = 0.0;
      costmap->mapToWorld(mx, my, world_x, world_y);
      points.emplace_back(world_x, world_y);
    }
  }

  return points;
}

}  // namespace nav2_neupan_controller
