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

BT::NodeStatus GetNextWaypoint::tick()
{
  Goals goals;
  if (!getInput("goals", goals) || goals.empty()) {
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
