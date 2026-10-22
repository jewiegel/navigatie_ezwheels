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

#include "hospital_robot_bt_nodes/get_next_waypoint.hpp"

namespace hospital_robot_bt_nodes
{

GetNextWaypoint::GetNextWaypoint(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (config.blackboard->getAny("node")) {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
      "remaining_waypoints", rclcpp::QoS(1).reliable().transient_local());
  }
}

BT::NodeStatus GetNextWaypoint::tick()
{
  Goals goals;
  getInput("goals", goals);
  if (publisher_) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = goals.empty() ? "map" : goals.front().header.frame_id;
    for (const auto & goal : goals) {
      msg.poses.push_back(goal.pose);
    }
    publisher_->publish(msg);
  }
  if (goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("goal", goals.front());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hospital_robot_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hospital_robot_bt_nodes::GetNextWaypoint>("GetNextWaypoint");
}
