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

#ifndef HOSPITAL_ROBOT_BT_NODES__POP_WAYPOINT_HPP_
#define HOSPITAL_ROBOT_BT_NODES__POP_WAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hospital_robot_bt_nodes
{

/// Removes the first waypoint (the one just reached) from the goal list. Always SUCCESS.
class PopWaypoint : public BT::SyncActionNode
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

  PopWaypoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Waypoints, first one reached"),
      BT::OutputPort<Goals>("output_goals", "Waypoints without the first one"),
    };
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace hospital_robot_bt_nodes

#endif  // HOSPITAL_ROBOT_BT_NODES__POP_WAYPOINT_HPP_
