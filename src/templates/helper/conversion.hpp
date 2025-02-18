@@INCLUDE_PACKAGES@@

#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:
  BridgeNode(std::string clientId, std::string serverId)
    : Node("bridge_node")
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

        raisin_node_ = raisin::Node(clientNetwork);
    }

  private:
    raisin::Node raisin_node_;
@@SUBSCRIBERS@@
};