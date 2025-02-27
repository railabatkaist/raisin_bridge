#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
#include "raisin_bridge_helper/conversion.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available


void BridgeNode::register_ros2_to_raisin(std::string type_name, std::string topic_name)
{@@ROS2_TO_RAISIN@@
}
void BridgeNode::register_raisin_to_ros2(std::string type_name, std::string topic_name)
{@@RAISIN_TO_ROS2@@
}