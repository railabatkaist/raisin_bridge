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

add_executable(raisin_bridge_node raisin_bridge.cpp ../generated/interfaces/raisin_interfaces/conversion.cpp 
../generated/interfaces/std_msgs/conversion.cpp ../generated/interfaces/builtin_interfaces/conversion.cpp)

ament_target_dependencies(raisin_bridge_node rclcpp raisin_bridge_helper)
ament_target_dependencies(raisin_bridge_node raisin_interfaces)
target_link_libraries(raisin_bridge_node raisin_network::raisin_network)
target_include_directories(raisin_bridge_node PRIVATE ../generated)
target_include_directories(raisin_bridge_node PRIVATE  $ENV{RAISIN_WS}/install)
target_include_directories(raisin_bridge_node PRIVATE  ../generated/interfaces)
# target_include_directories(raisin_bridge_node PRIVATE  ../../../install)

install(TARGETS
raisin_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
