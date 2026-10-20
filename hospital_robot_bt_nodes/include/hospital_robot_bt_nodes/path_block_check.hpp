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

#ifndef HOSPITAL_ROBOT_BT_NODES__PATH_BLOCK_CHECK_HPP_
#define HOSPITAL_ROBOT_BT_NODES__PATH_BLOCK_CHECK_HPP_

#include <cstdint>

#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"

namespace hospital_robot_bt_nodes
{

/// Checks whether the first part of a path runs through lethal costmap cells.
/// Pure logic: only message types, no rclcpp.
class PathBlockCheck
{
public:
  /// Walks the path from its first pose (the robot) up to lookahead_m metres
  /// along the path and returns true if any pose lies on a cell inside the
  /// costmap with cost >= lethal_threshold. Poses outside the costmap are not
  /// blocking. An empty path or an empty costmap returns false.
  /// Assumes an axis-aligned costmap in the same frame as the path (as Nav2 publishes it).
  bool is_blocked(
    const nav_msgs::msg::Path & path,
    const nav2_msgs::msg::Costmap & costmap,
    double lookahead_m,
    uint8_t lethal_threshold) const;
};

}  // namespace hospital_robot_bt_nodes

#endif  // HOSPITAL_ROBOT_BT_NODES__PATH_BLOCK_CHECK_HPP_
