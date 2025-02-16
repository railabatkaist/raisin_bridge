cmake_minimum_required(VERSION 3.8)
project(raisin_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)

# Get the relative path of all .msg and .srv files
file(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "interfaces/**/*msg")
file(GLOB_RECURSE srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "interfaces/**/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)
message("msg_files: ${msg_files}")
rosidl_generate_interfaces(raisin_bridge
  "interfaces/raisin_ffmpeg_image_transport/msg/FFMPEGPacket.msg"
  DEPENDENCIES std_msgs sensor_msgs
)


# set(EXTERNAL_INTERFACE_PATH "/home/user/raisin_ws/")

# Generate ROS 2 message and service files
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "src/raisin/raisin_interfaces/msg/Imu.msg"
#   DEPENDENCIES std_msgs
#   PATHS ${EXTERNAL_INTERFACE_PATH}
# )
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "/home/user/raisin_ws/src/raisin/raisin_interfaces/msg/Imu.msg"
#   DEPENDENCIES std_msgs
# )
# rosidl_generate_interfaces(${PROJECT_NAME}
# "/home/user/raisin_ws/src/raisin/raisin_interfaces/msg/Imu.msg"
# DEPENDENCIES std_msgs
# )


# find_package(raisin_network REQUIRED)
# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)
# target_link_libraries(talker raisin_network::raisin_network)
# target_include_directories(talker PRIVATE "/home/user/raisin_ws/install/include/raisin_master")

# install(TARGETS
#   talker
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION "include/${PROJECT_NAME}"
# ) 

ament_package()
