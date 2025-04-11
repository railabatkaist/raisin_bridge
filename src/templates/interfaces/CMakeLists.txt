cmake_minimum_required(VERSION 3.8)
project(@@PROJECT_NAME@@)
set(DEPENDENCIES @@DEPENDENCIES@@)
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
foreach(PACKAGE ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*msg")
file(GLOB_RECURSE srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

if (NOT DEPENDENCIES)
  rosidl_generate_interfaces(${PROJECT_NAME}
      ${msg_files}
      ${srv_files}
  )
else()
  rosidl_generate_interfaces(${PROJECT_NAME}
      ${msg_files}
      ${srv_files}
      DEPENDENCIES
      ${DEPENDENCIES}
  )
endif()

add_library(${PROJECT_NAME}_conversion SHARED conversion.cpp)
rosidl_target_interfaces(${PROJECT_NAME}_conversion ${PROJECT_NAME}
"rosidl_typesupport_cpp")
ament_target_dependencies(${PROJECT_NAME}_conversion raisin_bridge_helper)
foreach(PACKAGE ${DEPENDENCIES})
  ament_target_dependencies(${PROJECT_NAME}_conversion ${PACKAGE})
endforeach()
target_link_libraries(${PROJECT_NAME}_conversion raisin_network)
target_include_directories(${PROJECT_NAME}_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_conversion PRIVATE  $ENV{RAISIN_WS}/install)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_conversion DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_conversion
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories("include/${PROJECT_NAME}")

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
