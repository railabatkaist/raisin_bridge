#include "raisin_bridge/raisin_bridge.hpp"

void BridgeNode::connect()
{
  this->declare_parameter<std::string>("id", "raisin_bridge");
  this->declare_parameter<int>("network_type", 0);
  this->declare_parameter<std::string>("peer_id", "");
  this->declare_parameter<std::string>("peer_ip", "");
  this->declare_parameter<std::string>("network_interface", "");

  int networkType;
  std::string id, peerId, peerIp, netInterface;;
  this->get_parameter("id", id);
  this->get_parameter("peer_id", peerId);
  this->get_parameter("peer_ip", peerIp);
  this->get_parameter("network_type", networkType);
  this->get_parameter("network_interface", netInterface);

  std::vector<std::vector<std::string>> threads = {{"main"}};
  std::shared_ptr<raisin::Network> clientNetwork = std::make_shared<raisin::Network>(
      id,
      "test",
      threads,
      netInterface);
  clientNetwork->launchServer();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Attempt to connect to the server
  if (networkType == 0) {
    connection_ = clientNetwork->connect(peerId);
  } else if (networkType == 1) {
    connection_ = clientNetwork->connect(peerIp, 9002);
  } else {
    std::cerr<<"invalid network type"<<std::endl;
  }

  if (!connection_) {
    if (networkType == 0) {
      std::cerr<<"Failed to connect to server " << peerId << " at " << netInterface <<std::endl;
    } else if (networkType == 1) {
      std::cerr<<"Failed to connect to server " << peerIp <<std::endl;
    }
  } else {
    if (networkType == 0) {
      std::cout<<"Connected to server " << peerId << " at " << netInterface <<std::endl;
    } else if (networkType == 1) {
      std::cout<<"Connected to connect to server " << peerIp <<std::endl;
    }
  }

  raisin_node_ = std::make_unique<raisin::Node>(clientNetwork);
}

void BridgeNode::register_ros2_to_raisin(std::string type_name, std::string topic_name)
{
  std::stringstream data(type_name);
  std::string project_name, temp;
  std::getline(data, project_name,'/');
  std::getline(data, temp,'/');
  std::getline(data, type_name,'/');

  std::string filePath = ament_index_cpp::get_package_prefix(project_name) + "/lib/" + project_name + "/lib" + project_name + "_conversion.so";
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
  
  std::string filePath = ament_index_cpp::get_package_prefix(project_name) + "/lib/" + project_name + "/lib" + project_name + "_conversion.so";
  void* handle = dlopen(filePath.c_str(), RTLD_LAZY);  if (!handle)
    std::cerr << "Cannot open library: " << dlerror() << '\n';
  dlerror();
  register_t register_raisin_to_ros2 = (register_t) dlsym(handle, "register_raisin_to_ros2");
  if (!register_raisin_to_ros2)
    std::cerr << "Cannot load symbol 'register_raisin_to_ros2': " << dlerror() << '\n';
  register_raisin_to_ros2(this, type_name, topic_name);

  loaded_libraries_.push_back(handle); // keep it alive
}