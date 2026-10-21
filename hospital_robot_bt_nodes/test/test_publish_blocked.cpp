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
#include "hospital_robot_bt_nodes/publish_blocked.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class PublishBlockedTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("publish_blocked_test", "test_ns");
    factory_.registerNodeType<hospital_robot_bt_nodes::PublishBlocked>("PublishBlocked");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  }

  BT::Tree make_tree(const std::string & node_xml)
  {
    return factory_.createTreeFromText(
      "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + node_xml +
      "</BehaviorTree></root>", blackboard_);
  }

  // Ticks the tree once and waits for the message on the (namespaced) topic.
  bool tick_and_receive(BT::Tree & tree, const std::string & topic, bool & value)
  {
    bool received = false;
    auto sub = node_->create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [&](std_msgs::msg::Bool::SharedPtr msg) {
        received = true;
        value = msg->data;
      });
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(10ms);
    }
    return received;
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

TEST_F(PublishBlockedTest, PublishesTrue)
{
  auto tree = make_tree("<PublishBlocked blocked=\"true\"/>");
  bool value = false;
  ASSERT_TRUE(tick_and_receive(tree, "/test_ns/navigation/blocked", value));
  EXPECT_TRUE(value);
}

TEST_F(PublishBlockedTest, PublishesFalseOnCustomRelativeTopic)
{
  auto tree = make_tree("<PublishBlocked blocked=\"false\" topic=\"other/blocked\"/>");
  bool value = true;
  ASSERT_TRUE(tick_and_receive(tree, "/test_ns/other/blocked", value));
  EXPECT_FALSE(value);
}

TEST_F(PublishBlockedTest, MissingPortThrows)
{
  auto tree = make_tree("<PublishBlocked/>");
  EXPECT_THROW(tree.tickRoot(), BT::RuntimeError);
}

TEST_F(PublishBlockedTest, NonBoolPortThrows)
{
  auto tree = make_tree("<PublishBlocked blocked=\"maybe\"/>");
  EXPECT_THROW(tree.tickRoot(), BT::RuntimeError);
}
