#include "my_bt_nodes/object_seen.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace my_bt_nodes
{

my_bt_nodes::ObjectSeen::ObjectSeen(const std::string &name, const BT::NodeConfiguration &config) 
    : BT::ConditionNode(name, config)
{
}

BT::PortsList ObjectSeen::providedPorts()
{
    return {
        BT::InputPort<std::string>(
            "topic",
            "/object_seen",
            "Topic met std_msgs/Bool"
        )
    };
}

BT::NodeStatus ObjectSeen::tick()
{
    std::string topic;
    getInput("topic", topic);
    RCLCPP_INFO(rclcpp::get_logger("ObjectSeen"), "Bruh, ik luister naar: %s", topic.c_str());
    return BT::NodeStatus::SUCCESS;
}

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<my_bt_nodes::ObjectSeen>("ObjectSeen");
}
