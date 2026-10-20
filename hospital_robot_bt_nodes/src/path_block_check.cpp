// Copyright 2026 Improvia
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

#include "hospital_robot_bt_nodes/path_block_check.hpp"

#include <cmath>
#include <cstddef>

namespace hospital_robot_bt_nodes
{

bool PathBlockCheck::is_blocked(
  const nav_msgs::msg::Path & path,
  const nav2_msgs::msg::Costmap & costmap,
  double lookahead_m,
  uint8_t lethal_threshold) const
{
  const auto & meta = costmap.metadata;
  const std::size_t size_x = meta.size_x;
  const std::size_t size_y = meta.size_y;
  if (path.poses.empty() || size_x == 0 || size_y == 0 || meta.resolution <= 0.0f ||
    costmap.data.size() < size_x * size_y)
  {
    return false;
  }

  const double origin_x = meta.origin.position.x;
  const double origin_y = meta.origin.position.y;
  const double resolution = meta.resolution;

  double travelled = 0.0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    if (i > 0) {
      const auto & prev = path.poses[i - 1].pose.position;
      travelled += std::hypot(p.x - prev.x, p.y - prev.y);
      if (travelled > lookahead_m) {
        break;
      }
    }

    const double cx = std::floor((p.x - origin_x) / resolution);
    const double cy = std::floor((p.y - origin_y) / resolution);
    if (cx < 0.0 || cy < 0.0 || cx >= static_cast<double>(size_x) ||
      cy >= static_cast<double>(size_y))
    {
      continue;  // outside the costmap: no information, not blocking
    }

    const std::size_t index = static_cast<std::size_t>(cy) * size_x + static_cast<std::size_t>(cx);
    if (costmap.data[index] >= lethal_threshold) {
      return true;
    }
  }
  return false;
}

}  // namespace hospital_robot_bt_nodes
