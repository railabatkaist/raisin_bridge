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
    this->declare_parameter<std::string>("id", "raisin_bridge");
    this->declare_parameter<int>("network_type", 0);
    this->declare_parameter<std::string>("peer_id", "");
    this->declare_parameter<std::string>("peer_ip", "");
    this->declare_parameter<std::vector<std::string>>("network_interface", std::vector<std::string>{""});

    this->get_parameter("id", id_);
    this->get_parameter("peer_id", peerId_);
    this->get_parameter("peer_ip", peerIp_);
    this->get_parameter("network_type", networkType_);
    this->get_parameter("network_interface", netInterface_);

    std::vector<std::vector<std::string>> threads = {{"main"}};
    clientNetwork_ = std::make_shared<raisin::Network>(
        id_,
        "test",
        threads,
        netInterface_);
    clientNetwork_->launchServer();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    connection_timer_ = this->create_wall_timer(
      period_ms, std::bind(&BridgeNode::connect, this));

    raisin_node_ = std::make_unique<raisin::Node>(clientNetwork_);
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
  std::shared_ptr<raisin::Network> clientNetwork_;
  int networkType_;
  std::string id_, peerId_, peerIp_;
  std::vector<std::string> netInterface_;
  rclcpp::TimerBase::SharedPtr connection_timer_;

  std::map<std::string, std::any> ros2_publishers;
  std::map<std::string, std::any> ros2_subscriptions;
  std::map<std::string, std::any> raisin_publishers;
  std::map<std::string, std::any> raisin_subscribers;
  std::map<std::string, std::any> ros2_services;
  std::map<std::string, std::any> ros2_clients;
  std::map<std::string, std::any> raisin_services;
  std::map<std::string, std::any> raisin_clients;
  
  std::vector<void*> loaded_libraries_;  // or std::unordered_map if needed
  
  typedef void (* register_t) (BridgeNode * bridgeNode, std::string type_name, std::string topic_name);
};

#endif //RAISIN_BRIDGE_
