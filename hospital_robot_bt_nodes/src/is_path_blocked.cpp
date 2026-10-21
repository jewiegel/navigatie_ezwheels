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

#include "hospital_robot_bt_nodes/is_path_blocked.hpp"

#include <cstdint>
#include <string>

namespace hospital_robot_bt_nodes
{

IsPathBlocked::IsPathBlocked(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic = "global_costmap/costmap_raw";
  getInput("costmap_topic", topic);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // Matches the Nav2 costmap_raw publisher; depth 1 keeps only the latest costmap.
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {costmap_ = msg;}, options);
}

BT::NodeStatus IsPathBlocked::tick()
{
  callback_group_executor_.spin_some();

  int threshold = 253;
  getInput("lethal_threshold", threshold);
  if (threshold < 0 || threshold > 255) {
    throw BT::RuntimeError(
            "IsPathBlocked [", name(), "]: lethal_threshold must be 0-255, got ",
            std::to_string(threshold));
  }
  double lookahead = 1.5;
  getInput("lookahead", lookahead);

  nav_msgs::msg::Path path;
  if (!costmap_ || !getInput("path", path)) {
    return BT::NodeStatus::FAILURE;  // no information is not the same as blocked
  }

  const bool blocked = check_.is_blocked(
    path, *costmap_, lookahead, static_cast<uint8_t>(threshold));
  return blocked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace hospital_robot_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hospital_robot_bt_nodes::IsPathBlocked>("IsPathBlocked");
}
