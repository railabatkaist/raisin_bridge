cmake_minimum_required(VERSION 3.8)
project(raisin_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)
find_package(raisin_network)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## raisin bridge
add_library(raisin_bridge src/raisin_bridge.cpp)

ament_target_dependencies(raisin_bridge rclcpp ament_index_cpp)
target_link_libraries(raisin_bridge raisin_network)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(raisin_bridge PRIVATE  $ENV{RAISIN_WS}/install)

ament_export_dependencies(ament_index_cpp)
ament_export_targets(raisin_bridge_export HAS_LIBRARY_TARGET)
install(        
  TARGETS raisin_bridge        
  EXPORT raisin_bridge_export        
  ARCHIVE DESTINATION lib        
  LIBRARY DESTINATION lib        
  RUNTIME DESTINATION bin        
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

add_executable(raisin_bridge_node src/main.cpp)

ament_target_dependencies(raisin_bridge_node rclcpp)
target_link_libraries(raisin_bridge_node raisin_network raisin_bridge)
target_include_directories(raisin_bridge_node PRIVATE  $ENV{RAISIN_WS}/install)

install(TARGETS
raisin_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
