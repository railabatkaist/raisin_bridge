#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
@@INCLUDE_DEPENDENCIES@@

using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:  
  BridgeNode(std::string serverId, std::string clientId)
  : rclcpp::Node("bridge_node")
  {
      std::vector<std::vector<std::string>> threads = {{"main"}};
      std::shared_ptr<raisin::Network> clientNetwork = std::make_shared<raisin::Network>(
          clientId,
          "test",
          threads);

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

  template <typename T_ROS, typename T_RAISIN>
  void register_ros2_to_raisin(std::string topic_name){
      auto raisin_publisher = raisin_node_->createPublisher<T_RAISIN>(topic_name);
      auto ros2_subscription = this->create_subscription<T_ROS>(
          topic_name, 10,
          [raisin_publisher](std::shared_ptr<T_ROS> msg) {
              raisin_publisher->publish(to_raisin_msg(*msg));  // Publish the same message
          }
      );
  }

  template <typename T_ROS, typename T_RAISIN>
  void register_raisin_to_ros2(std::string topic_name){
      auto ros2_publisher = this->create_publisher<T_ROS>(topic_name, 10);
      auto raisin_subscriber = raisin_node_->createSubscriber<T_RAISIN>(
          topic_name, connection_,
          std::bind([ros2_publisher](std::shared_ptr<T_RAISIN> msg) {
              ros2_publisher->publish(to_ros_msg(*msg));  // Publish the same message
          }, _1)
      );
  }

  private:
    std::unique_ptr<raisin::Node> raisin_node_;
    std::shared_ptr<raisin::Remote::Connection> connection_;
};