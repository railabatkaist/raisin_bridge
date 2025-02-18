// Copyright (c) 2024 Raion Robotics, Inc.
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by munjuhyeok on 2/16/24.
//


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "include/conversion.hpp"

#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"

#include <interfaces/raisin_interfaces/conversion.hpp>
#include <raisin_interfaces/msg/imu.hpp>

#include <include/raisin_interfaces/msg/imu.hpp>
// #include "raisin_interfaces/include/raisin_interfaces/raisin_interfaces/msg/imu.hpp"


#include "interfaces.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:
  BridgeNode()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<raisin_interfaces::msg::Imu>(
      "topic", 10, std::bind(&BridgeNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const raisin_interfaces::msg::Imu::SharedPtr msg) const
    {
      auto raisin_msg = to_raisin_msg(*msg);
      std::cout<<"Received message: "<<raisin_msg.quaternion_w<<std::endl;
    }
    rclcpp::Subscription<raisin_interfaces::msg::Imu>::SharedPtr subscription_;
};

void subscriberCallback1(const std::shared_ptr<raisin::raisin_interfaces::msg::Imu> msg)
{
  std::cout << "subscriber1 works " << to_ros_msg(*msg).angular_velocity_x << std::endl;
}

int main(int argc, char * argv[])
{
  std::string serverId = "server";
  std::string clientId = "client";

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

    raisin::Node node(clientNetwork);

    auto publisher = node.createPublisher<raisin::raisin_interfaces::msg::Imu>("imu");

    auto subscriber1 = node.createSubscriber<raisin::raisin_interfaces::msg::Imu>(
      "imu", connection, std::bind(
        subscriberCallback1,
        _1));

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeNode>());
    rclcpp::shutdown();
    
    return 0;
}