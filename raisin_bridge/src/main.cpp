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

#include "raisin_bridge/raisin_bridge.hpp"

int main(int argc, char * argv[])
{
  raisin::LockRecorder::getInstance() = std::make_shared<raisin::LockRecorder>();

  rclcpp::init(argc, argv);
  std::shared_ptr<BridgeNode> bridge_node = std::make_shared<BridgeNode>();
  bridge_node->connect();

  bridge_node->declare_parameter<std::vector<std::string>>("topics_ros2_to_raisin", {});
  bridge_node->declare_parameter<std::vector<std::string>>("topics_raisin_to_ros2", {});
  bridge_node->declare_parameter<std::vector<std::string>>("services_ros2_to_raisin", {});
  bridge_node->declare_parameter<std::vector<std::string>>("services_raisin_to_ros2", {});

  // Get parameters
  std::vector<std::string> topics_ros2_to_raisin, topics_raisin_to_ros2, services_ros2_to_raisin, services_raisin_to_ros2;
  bridge_node->get_parameter("topics_ros2_to_raisin", topics_ros2_to_raisin);
  bridge_node->get_parameter("topics_raisin_to_ros2", topics_raisin_to_ros2);
  bridge_node->get_parameter("services_ros2_to_raisin", services_ros2_to_raisin);
  bridge_node->get_parameter("services_raisin_to_ros2", services_raisin_to_ros2);

  for (const auto& topic : topics_ros2_to_raisin) {
    size_t pos = topic.find(", ");
    if (pos != std::string::npos) {
      std::string type = topic.substr(0, pos);
      std::string name = topic.substr(pos + 2);

      bridge_node->register_ros2_to_raisin_msg(type, name);
    }
  }

  for (const auto& topic : topics_raisin_to_ros2) {
    size_t pos = topic.find(", ");
    if (pos != std::string::npos) {
      std::string type = topic.substr(0, pos);
      std::string name = topic.substr(pos + 2);

      bridge_node->register_raisin_to_ros2_msg(type, name);
    }
  }

  for (const auto& service : services_ros2_to_raisin) {
    size_t pos = service.find(", ");
    if (pos != std::string::npos) {
      std::string type = service.substr(0, pos);
      std::string name = service.substr(pos + 2);

      bridge_node->register_ros2_to_raisin_srv(type, name);
    }
  }

  for (const auto& service : services_raisin_to_ros2) {
    size_t pos = service.find(", ");
    if (pos != std::string::npos) {
      std::string type = service.substr(0, pos);
      std::string name = service.substr(pos + 2);

      bridge_node->register_raisin_to_ros2_srv(type, name);
    }
  }

  rclcpp::spin(bridge_node);

  rclcpp::shutdown();
  
  return 0;
}