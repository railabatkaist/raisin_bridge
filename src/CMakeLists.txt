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

add_executable(raisin_bridge_node raisin_bridge.cpp)

ament_target_dependencies(raisin_bridge_node rclcpp raisin_bridge_helper)
target_link_libraries(raisin_bridge_node raisin_network)
target_include_directories(raisin_bridge_node PRIVATE ../generated)
target_include_directories(raisin_bridge_node PRIVATE  $ENV{RAISIN_WS}/install)
target_include_directories(raisin_bridge_node PRIVATE  ../generated/interfaces)
# target_include_directories(raisin_bridge_node PRIVATE  ../../../install)

install(TARGETS
raisin_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ../launch ../config
  DESTINATION share/${PROJECT_NAME})
ament_package()
