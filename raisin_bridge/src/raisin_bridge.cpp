#include "raisin_bridge/raisin_bridge.hpp"

void BridgeNode::connect()
{
  // Attempt to connect to the server
  while (!connection_) {
    if (networkType_ == 0) {
      connection_ = clientNetwork_->connect(peerId_);
    } else if (networkType_ == 1) {
      connection_ = clientNetwork_->connect(peerIp_, 9002);
    } else {
      std::cerr<<"invalid network type"<<std::endl;
    }
    if (!connection_) {
      if (networkType_ == 0) {
        std::cerr<<"Failed to connect to server " << peerId_ << " at " << netInterface_ <<std::endl;
      } else if (networkType_ == 1) {
        std::cerr<<"Failed to connect to server " << peerIp_ <<std::endl;
      }
    } else {
      if (networkType_ == 0) {
        std::cout<<"Connected to server " << peerId_ << " at " << netInterface_ <<std::endl;
      } else if (networkType_ == 1) {
        std::cout<<"Connected to connect to server " << peerIp_ <<std::endl;
      }
    }
  }


}

void BridgeNode::register_ros2_to_raisin(std::string type_name, std::string topic_name)
{
  std::stringstream data(type_name);
  std::string project_name, temp;
  std::getline(data, project_name,'/');
  std::getline(data, temp,'/');
  std::getline(data, type_name,'/');

  std::string filePath = ament_index_cpp::get_package_prefix(project_name + "_conversion") + "/lib/" + project_name + "/lib" + project_name + "_conversion.so";
  void* handle = dlopen(filePath.c_str(), RTLD_LAZY);
  if (!handle)
    std::cerr << "Cannot open library: " << dlerror() << '\n';
  dlerror();
  register_t register_ros2_to_raisin = (register_t) dlsym(handle, "register_ros2_to_raisin");
  if (!register_ros2_to_raisin)
    std::cerr << "Cannot load symbol 'register_ros2_to_raisin': " << dlerror() << '\n';
  register_ros2_to_raisin(this, type_name, topic_name);

  loaded_libraries_.push_back(handle); // keep it alive
}
void BridgeNode::register_raisin_to_ros2(std::string type_name, std::string topic_name)
{
  std::stringstream data(type_name);
  std::string project_name, temp;
  std::getline(data, project_name,'/');
  std::getline(data, temp,'/');
  std::getline(data, type_name,'/');
  
  std::string filePath = ament_index_cpp::get_package_prefix(project_name + "_conversion") + "/lib/" + project_name + "/lib" + project_name + "_conversion.so";
  void* handle = dlopen(filePath.c_str(), RTLD_LAZY);  if (!handle)
    std::cerr << "Cannot open library: " << dlerror() << '\n';
  dlerror();
  register_t register_raisin_to_ros2 = (register_t) dlsym(handle, "register_raisin_to_ros2");
  if (!register_raisin_to_ros2)
    std::cerr << "Cannot load symbol 'register_raisin_to_ros2': " << dlerror() << '\n';
  register_raisin_to_ros2(this, type_name, topic_name);

  loaded_libraries_.push_back(handle); // keep it alive
}