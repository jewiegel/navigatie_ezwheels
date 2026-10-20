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

#include <gtest/gtest.h>

#include <cstdint>

#include "hospital_robot_bt_nodes/path_block_check.hpp"

using hospital_robot_bt_nodes::PathBlockCheck;

namespace
{

constexpr uint8_t kLethal = 253;

// 10 x 10 cells of 0.1 m, origin (0, 0): covers x, y in [0, 1).
nav2_msgs::msg::Costmap make_costmap()
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 0.1f;
  costmap.metadata.size_x = 10;
  costmap.metadata.size_y = 10;
  costmap.data.assign(100, 0);
  return costmap;
}

void set_cost(nav2_msgs::msg::Costmap & costmap, unsigned x, unsigned y, uint8_t cost)
{
  costmap.data[y * costmap.metadata.size_x + x] = cost;
}

// Straight line along y = 0.05 from x_start, one pose every 0.1 m.
nav_msgs::msg::Path make_path(double x_start, int poses)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < poses; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x_start + 0.1 * i + 0.05;
    pose.pose.position.y = 0.05;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PathBlockCheck, FreePathIsNotBlocked)
{
  EXPECT_FALSE(PathBlockCheck().is_blocked(make_path(0.0, 10), make_costmap(), 1.5, kLethal));
}

TEST(PathBlockCheck, LethalCellWithinLookaheadIsBlocked)
{
  auto costmap = make_costmap();
  set_cost(costmap, 3, 0, 254);  // 0.3 m along the path
  EXPECT_TRUE(PathBlockCheck().is_blocked(make_path(0.0, 10), costmap, 0.5, kLethal));
}

TEST(PathBlockCheck, LethalCellJustBeyondLookaheadIsNotBlocked)
{
  auto costmap = make_costmap();
  set_cost(costmap, 6, 0, 254);  // pose 6 lies 0.6 m along the path
  EXPECT_FALSE(PathBlockCheck().is_blocked(make_path(0.0, 10), costmap, 0.55, kLethal));
  EXPECT_TRUE(PathBlockCheck().is_blocked(make_path(0.0, 10), costmap, 0.65, kLethal));
}

TEST(PathBlockCheck, CostBelowThresholdIsNotBlocked)
{
  auto costmap = make_costmap();
  set_cost(costmap, 3, 0, kLethal - 1);
  EXPECT_FALSE(PathBlockCheck().is_blocked(make_path(0.0, 10), costmap, 1.5, kLethal));
}

TEST(PathBlockCheck, PosesOutsideCostmapAreIgnored)
{
  auto costmap = make_costmap();
  // Path starts at x = -0.45 (outside), enters the map and hits a lethal cell at x = 0.25.
  set_cost(costmap, 2, 0, 254);
  EXPECT_FALSE(PathBlockCheck().is_blocked(make_path(-0.5, 5), costmap, 1.5, kLethal));
  EXPECT_TRUE(PathBlockCheck().is_blocked(make_path(-0.5, 10), costmap, 1.5, kLethal));
}

TEST(PathBlockCheck, EmptyPathIsNotBlocked)
{
  auto costmap = make_costmap();
  set_cost(costmap, 0, 0, 254);
  EXPECT_FALSE(PathBlockCheck().is_blocked(nav_msgs::msg::Path(), costmap, 1.5, kLethal));
}

TEST(PathBlockCheck, EmptyCostmapIsNotBlocked)
{
  EXPECT_FALSE(
    PathBlockCheck().is_blocked(
      make_path(0.0, 10), nav2_msgs::msg::Costmap(), 1.5, kLethal));
}
