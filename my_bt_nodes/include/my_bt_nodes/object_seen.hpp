#ifndef MY_BT_NODES__OBJECT_SEEN_HPP_
#define MY_BT_NODES__OBJECT_SEEN_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"

namespace my_bt_nodes
{
class ObjectSeen : public BT::ConditionNode
{    
public:
    ObjectSeen(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
}

#endif