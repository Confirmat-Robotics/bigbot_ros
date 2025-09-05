#include "bt_debug_print/debug_print_node.hpp"
#include <chrono>
#include <iostream>

using namespace bt_debug_print;

DebugPrint::DebugPrint(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList DebugPrint::providedPorts() {
  return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus DebugPrint::tick() {
  std::string msg;
  if (!getInput("message", msg)) {
    msg = "[DebugPrint] No message provided.";
  }
  std::cout << "[DebugPrint] " << msg << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// Register the node
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<bt_debug_print::DebugPrint>("DebugPrint");
}
