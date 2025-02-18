cmake_minimum_required(VERSION 3.22)
find_package(ament_cmake REQUIRED)
project(raisin_bridge_helper)

set(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)
find_package(raisin_network REQUIRED)
@@CONTENT@@

target_link_libraries(raisin_bridge_helper raisin_network::raisin_network)
target_include_directories(raisin_bridge_helper PRIVATE  $ENV{RAISIN_WS}/install)
target_include_directories(raisin_bridge_helper PRIVATE  ../interfaces)
ament_export_targets(raisin_bridge_helper_export HAS_LIBRARY_TARGET)
install(        
  TARGETS raisin_bridge_helper        
  EXPORT raisin_bridge_helper_export        
  ARCHIVE DESTINATION lib        
  LIBRARY DESTINATION lib        
  RUNTIME DESTINATION bin        
  INCLUDES DESTINATION include
)
install(DIRECTORY ../interfaces/ DESTINATION include)
install(DIRECTORY include DESTINATION include)

ament_package()