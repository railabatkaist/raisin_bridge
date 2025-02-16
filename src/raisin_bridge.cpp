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

#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"

#include <ros2_raisin_interfaces/msg/imu.hpp>

#include "interfaces.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available

class BridgeNode : public rclcpp::Node
{
  public:
  BridgeNode()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<ros2_raisin_interfaces::msg::Imu>(
      "topic", 10, std::bind(&BridgeNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const ros2_raisin_interfaces::msg::Imu msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<ros2_raisin_interfaces::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    std::vector<std::vector<std::string>> threads = {{"main"}};
    std::shared_ptr<raisin::Network> serverNetwork = std::make_shared<raisin::Network>(
      "server",
      "test",
      threads);

    raisin::Node node(serverNetwork);

    auto publisher = node.createPublisher<raisin::raisin_interfaces::msg::Imu>("imu");

    return 0;
}