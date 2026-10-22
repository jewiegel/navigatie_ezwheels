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

#ifndef HOSPITAL_ROBOT_BT_NODES__GET_NEXT_WAYPOINT_HPP_
#define HOSPITAL_ROBOT_BT_NODES__GET_NEXT_WAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hospital_robot_bt_nodes
{

/// Outputs the first waypoint of `goals` as `goal`. FAILURE if `goals` is empty,
/// which the hospital tree uses as "route finished".
/// Also publishes all remaining waypoints (current goal first, empty when the route is
/// done) on `remaining_waypoints` (PoseArray, transient_local), so the turn detector sees
/// turns at waypoints, whoever started the route. Only when the blackboard has a "node".
class GetNextWaypoint : public BT::SyncActionNode
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

  GetNextWaypoint(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("goals", "Remaining waypoints"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Next waypoint"),
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

}  // namespace hospital_robot_bt_nodes

#endif  // HOSPITAL_ROBOT_BT_NODES__GET_NEXT_WAYPOINT_HPP_
