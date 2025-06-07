#ifndef RAISIN_BRIDGE_
#define RAISIN_BRIDGE_

#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
#include <any>
#include "ament_index_cpp/get_package_prefix.hpp"
#include <dlfcn.h>

using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:  
  BridgeNode()
  : rclcpp::Node("bridge_node")
  {
  }
  ~BridgeNode()
  {
    for (void* handle : loaded_libraries_) {
      if (handle) {
        dlclose(handle);
      }
    }
    raisin_node_->cleanupResources();
  }
  void connect();
  void register_ros2_to_raisin(std::string type_name, std::string topic_name);
  void register_raisin_to_ros2(std::string type_name, std::string topic_name);
  
  std::unique_ptr<raisin::Node> raisin_node_;
  std::shared_ptr<raisin::Remote::Connection> connection_;
  std::map<std::string, std::any> ros2_publishers;
  std::map<std::string, std::any> ros2_subscriptions;
  std::map<std::string, std::any> raisin_publishers;
  std::map<std::string, std::any> raisin_subscribers;
  
  std::vector<void*> loaded_libraries_;  // or std::unordered_map if needed
  
  typedef void (* register_t) (BridgeNode * bridgeNode, std::string type_name, std::string topic_name);
};

#endif //RAISIN_BRIDGE_
