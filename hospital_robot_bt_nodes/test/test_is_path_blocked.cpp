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

#include "behaviortree_cpp_v3/bt_factory.h"
#include "hospital_robot_bt_nodes/is_path_blocked.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IsPathBlockedTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("is_path_blocked_test", "test_ns");
    costmap_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(
      "global_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local());
    factory_.registerNodeType<hospital_robot_bt_nodes::IsPathBlocked>("IsPathBlocked");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  }

  BT::Tree make_tree(const std::string & attributes = "")
  {
    return factory_.createTreeFromText(
      "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">"
      "<IsPathBlocked path=\"{path}\" lookahead=\"1.0\" " + attributes + "/>"
      "</BehaviorTree></root>", blackboard_);
  }

  // 10 x 10 cells of 0.1 m at origin (0, 0); optionally one lethal cell.
  void publish_costmap(bool with_obstacle)
  {
    nav2_msgs::msg::Costmap costmap;
    costmap.metadata.resolution = 0.1f;
    costmap.metadata.size_x = 10;
    costmap.metadata.size_y = 10;
    costmap.data.assign(100, 0);
    if (with_obstacle) {
      costmap.data[3] = 254;  // cell (3, 0): 0.3 m along the path
    }
    costmap_pub_->publish(costmap);
  }

  void set_path()
  {
    nav_msgs::msg::Path path;
    for (int i = 0; i < 10; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = 0.1 * i + 0.05;
      pose.pose.position.y = 0.05;
      path.poses.push_back(pose);
    }
    blackboard_->set<nav_msgs::msg::Path>("path", path);
  }

  // Ticks until the expected status is returned (the costmap arrives asynchronously).
  BT::NodeStatus tick_until(BT::Tree & tree, BT::NodeStatus expected)
  {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      status = tree.tickRoot();
      if (status == expected) {
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    return status;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

TEST_F(IsPathBlockedTest, SuccessWhenBlockedFailureWhenFree)
{
  auto tree = make_tree();
  set_path();
  publish_costmap(true);
  EXPECT_EQ(tick_until(tree, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
  publish_costmap(false);
  EXPECT_EQ(tick_until(tree, BT::NodeStatus::FAILURE), BT::NodeStatus::FAILURE);
}

TEST_F(IsPathBlockedTest, FailureWithoutCostmap)
{
  auto tree = make_tree();
  set_path();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(IsPathBlockedTest, FailureWithoutPath)
{
  auto tree = make_tree();
  publish_costmap(true);
  // Even with a lethal costmap: no path means no information, so never SUCCESS.
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    std::this_thread::sleep_for(10ms);
  }
}

TEST_F(IsPathBlockedTest, InvalidThresholdThrows)
{
  auto tree = make_tree("lethal_threshold=\"300\"");
  EXPECT_THROW(tree.tickRoot(), BT::RuntimeError);
}
