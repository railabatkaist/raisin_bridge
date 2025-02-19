#include "rclcpp/rclcpp.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
#include "include/conversion.hpp"


using namespace std::placeholders;  // To make _1, _2, etc., available


template <typename T_ROS, typename T_RAISIN>
void BridgeNode::register_ros2_to_raisin(std::string topic_name){
    auto raisin_publisher = raisin_node_->createPublisher<T_RAISIN>(topic_name);
    auto ros2_subscription = this->create_subscription<T_ROS>(
        topic_name, 10,
        [raisin_publisher](std::shared_ptr<T_ROS> msg) {
            raisin_publisher->publish(to_raisin_msg(*msg));  // Publish the same message
        }
    );
}

template <typename T_ROS, typename T_RAISIN>
void BridgeNode::register_raisin_to_ros2(std::string topic_name){
    auto ros2_publisher = this->create_publisher<T_ROS>(topic_name, 10);
    auto raisin_subscriber = raisin_node_->createSubscriber<T_RAISIN>(
        topic_name, connection_,
        std::bind([ros2_publisher](std::shared_ptr<T_RAISIN> msg) {
            ros2_publisher->publish(to_ros_msg(*msg));  // Publish the same message
        }, _1)
    );
}
