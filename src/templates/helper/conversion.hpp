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
  BridgeNode(std::string serverId, std::string clientId, std::string networkInterface)
  : rclcpp::Node("bridge_node")
  {
      std::vector<std::vector<std::string>> threads = {{"main"}};
      std::shared_ptr<raisin::Network> clientNetwork = std::make_shared<raisin::Network>(
          clientId,
          "test",
          threads,
          networkInterface);

      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Attempt to connect to the server
      if (!connection_) {
          connection_ = clientNetwork->connect(serverId);
          std::cerr << "[Client] Failed to connect to server at " << serverId << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      std::cout << "[Client] Connected to server at " << serverId << ". Starting message exchange..." <<
      std::endl;

      raisin_node_ = std::make_unique<raisin::Node>(raisin::Node(clientNetwork));
  }

  void register_ros2_to_raisin(std::string type_name, std::string topic_name);
  void register_raisin_to_ros2(std::string type_name, std::string topic_name);
  
  std::unique_ptr<raisin::Node> raisin_node_;
  std::shared_ptr<raisin::Remote::Connection> connection_;
  std::map<std::string, std::any> ros2_publishers;
  std::map<std::string, std::any> ros2_subscriptions;
  std::map<std::string, std::any> raisin_publishers;
  std::map<std::string, std::any> raisin_subscribers;
  
  typedef void (* register_t) (BridgeNode * bridgeNode, std::string type_name, std::string topic_name);
};