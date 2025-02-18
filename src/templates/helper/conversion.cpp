#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
#include "include/conversion.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:
  BridgeNode(std::string clientId, std::string serverId)
    : rclcpp::Node("bridge_node")
    {
        std::vector<std::vector<std::string>> threads = {{"main"}};
        std::shared_ptr<raisin::Network> clientNetwork = std::make_shared<raisin::Network>(
            clientId,
            "test",
            threads);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Attempt to connect to the server
        std::shared_ptr<raisin::Remote::Connection> connection;
        if (!connection) {
            connection = clientNetwork->connect(serverId);
            std::cerr << "[Client] Failed to connect to server at " << serverId << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "[Client] Connected to server at " << serverId << ". Starting message exchange..." <<
        std::endl;

        raisin_node_ = std::make_unique<raisin::Node>(raisin::Node(clientNetwork));
    }

// @@REGISTER_FUNCTIONS@@
//     void register_ros2_to_raisin(std::string topic_name, std::string message_type){
// @@ROS2_TO_RAISIN@@
//         auto callback = std::bind(&BridgeNode::ros2_to_raisin_callback<@@MESSAGE_TYPE@@>, this, _1);
//         ros2_subscription_@@MESSAGE_TYPE@@_ = this->create_subscription<@@MESSAGE_TYPE@@>(topic_name, 10, callback);
//         raisin_subscription_@@MESSAGE_TYPE@@_ = raisin_node_.create_subscriber<@@MESSAGE_TYPE@@>(topic_name);
//     }

//     void register_raisin_to_ros2(std::string topic_name, std::string message_type){
// @@RAISIN_TO_ROS2@@
//         auto callback = std::bind(&BridgeNode::raisin_to_ros2_callback<@@MESSAGE_TYPE@@>, this, _1);
//         ros2_subscription_@@MESSAGE_TYPE@@_ = this->create_subscription<@@MESSAGE_TYPE@@>(topic_name, 10, callback);
//         raisin_subscription_@@MESSAGE_TYPE@@_ = raisin_node_.create_subscriber<@@MESSAGE_TYPE@@>(topic_name);
//     }

  private:
    std::unique_ptr<raisin::Node> raisin_node_;
    // publishers
@@PUBLISHERS@@
    // subscribers
@@SUBSCRIBERS@@
};