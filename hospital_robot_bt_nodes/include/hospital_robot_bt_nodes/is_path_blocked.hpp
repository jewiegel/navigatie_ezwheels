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

#ifndef HOSPITAL_ROBOT_BT_NODES__IS_PATH_BLOCKED_HPP_
#define HOSPITAL_ROBOT_BT_NODES__IS_PATH_BLOCKED_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "hospital_robot_bt_nodes/path_block_check.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hospital_robot_bt_nodes
{

/// SUCCESS if the first `lookahead` metres of `path` cross a lethal costmap cell.
/// FAILURE if the path is free, or if there is no path or no costmap yet.
class IsPathBlocked : public BT::ConditionNode
{
public:
  IsPathBlocked(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check, starting at the robot"),
      BT::InputPort<double>("lookahead", 1.5, "Distance along the path to check [m]"),
      BT::InputPort<std::string>(
        "costmap_topic", "global_costmap/costmap_raw", "Relative nav2_msgs/Costmap topic"),
      // int instead of uint8: BehaviorTree.CPP v3 cannot convert port strings to uint8_t.
      BT::InputPort<int>("lethal_threshold", 253, "Cost >= this value blocks [0-255]"),
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  PathBlockCheck check_;
};

}  // namespace hospital_robot_bt_nodes

#endif  // HOSPITAL_ROBOT_BT_NODES__IS_PATH_BLOCKED_HPP_
