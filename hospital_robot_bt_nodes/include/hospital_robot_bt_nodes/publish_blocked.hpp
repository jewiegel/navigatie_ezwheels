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

#ifndef HOSPITAL_ROBOT_BT_NODES__PUBLISH_BLOCKED_HPP_
#define HOSPITAL_ROBOT_BT_NODES__PUBLISH_BLOCKED_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hospital_robot_bt_nodes
{

/// Publishes the input port `blocked` as std_msgs/Bool and always returns SUCCESS.
/// The publisher is transient_local, so late subscribers get the last value.
class PublishBlocked : public BT::SyncActionNode
{
public:
  PublishBlocked(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("blocked", "Value to publish"),
      BT::InputPort<std::string>("topic", "navigation/blocked", "Relative topic name"),
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

}  // namespace hospital_robot_bt_nodes

#endif  // HOSPITAL_ROBOT_BT_NODES__PUBLISH_BLOCKED_HPP_
