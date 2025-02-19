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

// #include "raisin_interfaces/include/raisin_interfaces/raisin_interfaces/msg/imu.hpp"

int main(int argc, char * argv[])
{
  std::string serverId = "server";
  std::string clientId = "raisin_bridge";

  rclcpp::init(argc, argv);
  std::shared_ptr<BridgeNode> bridge_node = std::make_shared<BridgeNode>(serverId, clientId);
  
  bridge_node->register_raisin_to_ros2<std_msgs::msg::String, raisin::std_msgs::msg::String>("imu");
  bridge_node->register_ros2_to_raisin<std_msgs::msg::String, raisin::std_msgs::msg::String>("chatter");

  rclcpp::spin(bridge_node);

  rclcpp::shutdown();
  
  return 0;
}