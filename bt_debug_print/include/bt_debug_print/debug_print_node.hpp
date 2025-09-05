#pragma once

#include <behaviortree_cpp/action_node.h>

namespace bt_debug_print {

class DebugPrint : public BT::SyncActionNode {
public:
  DebugPrint(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace bt_debug_print
