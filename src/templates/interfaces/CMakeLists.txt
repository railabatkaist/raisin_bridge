cmake_minimum_required(VERSION 3.8)
project(@@PROJECT_NAME@@)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)
find_package(raisin_network REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# TODO: replace the following dependencies with the ones built by myself
find_package(raisin_bridge_helper)
@@FIND_DEPENDENCIES@@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*msg")
file(GLOB_RECURSE srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}_ros2
    ${msg_files}
    ${srv_files}
    @@DEPENDENCIES@@
)

add_library(${PROJECT_NAME}_conversion SHARED conversion.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_conversion ${PROJECT_NAME}_ros2
"rosidl_typesupport_cpp")
ament_target_dependencies(${PROJECT_NAME}_conversion raisin_bridge_helper)
target_link_libraries(${PROJECT_NAME}_conversion raisin_network)
target_include_directories(${PROJECT_NAME}_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_conversion PRIVATE  $ENV{RAISIN_WS}/install)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_conversion DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
