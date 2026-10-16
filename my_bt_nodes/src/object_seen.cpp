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
    return {};
}

BT::NodeStatus ObjectSeen::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("ObjectSeen"), "Tik test");
    return BT::NodeStatus::SUCCESS;
}

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<my_bt_nodes::ObjectSeen>("ObjectSeen");
}
