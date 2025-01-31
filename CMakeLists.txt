cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_PREFIX_PATH "/home/user/raisin_ws/install")
include_directories("/home/user/raisin_ws/install/lib")
find_package(raisin_network REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
target_link_libraries(talker raisin_network::raisin_network)
# target_include_directories(talker PRIVATE "/home/user/raisin_ws/install/include/raisin_master")

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
