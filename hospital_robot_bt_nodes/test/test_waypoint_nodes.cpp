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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "hospital_robot_bt_nodes/get_next_waypoint.hpp"
#include "hospital_robot_bt_nodes/pop_waypoint.hpp"
#include "rclcpp/rclcpp.hpp"

using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

class WaypointNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<hospital_robot_bt_nodes::GetNextWaypoint>("GetNextWaypoint");
    factory_.registerNodeType<hospital_robot_bt_nodes::PopWaypoint>("PopWaypoint");
    blackboard_ = BT::Blackboard::create();
  }

  BT::Tree make_tree(const std::string & node_xml)
  {
    return factory_.createTreeFromText(
      "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + node_xml +
      "</BehaviorTree></root>", blackboard_);
  }

  static Goals make_goals(std::vector<double> xs)
  {
    Goals goals;
    for (double x : xs) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      goals.push_back(pose);
    }
    return goals;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

TEST_F(WaypointNodesTest, GetNextWaypointOutputsFirstGoal)
{
  blackboard_->set<Goals>("goals", make_goals({1.0, 2.0}));
  auto tree = make_tree("<GetNextWaypoint goals=\"{goals}\" goal=\"{goal}\"/>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(blackboard_->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 1.0);
  EXPECT_EQ(blackboard_->get<Goals>("goals").size(), 2u);  // not consumed
}

TEST_F(WaypointNodesTest, GetNextWaypointFailsWhenRouteDone)
{
  blackboard_->set<Goals>("goals", Goals());
  auto tree = make_tree("<GetNextWaypoint goals=\"{goals}\" goal=\"{goal}\"/>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(WaypointNodesTest, PopWaypointRemovesFirstGoal)
{
  blackboard_->set<Goals>("goals", make_goals({1.0, 2.0}));
  auto tree = make_tree("<PopWaypoint input_goals=\"{goals}\" output_goals=\"{goals}\"/>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  auto goals = blackboard_->get<Goals>("goals");
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_DOUBLE_EQ(goals.front().pose.position.x, 2.0);
}

TEST_F(WaypointNodesTest, PopWaypointOnEmptyListStaysEmpty)
{
  blackboard_->set<Goals>("goals", Goals());
  auto tree = make_tree("<PopWaypoint input_goals=\"{goals}\" output_goals=\"{goals}\"/>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard_->get<Goals>("goals").empty());
}

TEST_F(WaypointNodesTest, GetNextWaypointPublishesRemainingWaypoints)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("get_next_waypoint_test");
  blackboard_->set<rclcpp::Node::SharedPtr>("node", node);
  std::vector<size_t> sizes;
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "remaining_waypoints", rclcpp::QoS(10).reliable().transient_local(),
    [&](geometry_msgs::msg::PoseArray::SharedPtr msg) {sizes.push_back(msg->poses.size());});

  blackboard_->set<Goals>("goals", make_goals({1.0, 2.0}));
  auto tree = make_tree("<GetNextWaypoint goals=\"{goals}\" goal=\"{goal}\"/>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  blackboard_->set<Goals>("goals", Goals());
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);  // route done: empty list

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (sizes.size() < 2 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(sizes, (std::vector<size_t>{2u, 0u}));
  rclcpp::shutdown();
}
