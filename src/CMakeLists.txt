cmake_minimum_required(VERSION 3.8)
project(raisin_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)
find_package(raisin_network)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(raisin_bridge_helper REQUIRED)
include(../generated/helper/find_interfaces.cmake)

add_executable(raisin_bridge raisin_bridge.cpp)

ament_target_dependencies(raisin_bridge rclcpp raisin_bridge_helper)
target_link_libraries(raisin_bridge raisin_network::raisin_network)
target_include_directories(raisin_bridge PRIVATE ../generated)

ament_package()
