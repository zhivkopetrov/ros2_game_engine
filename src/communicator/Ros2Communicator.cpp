//Corresponding header
#include "ros2_game_engine/communicator/Ros2Communicator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Ros2Communicator::init([[maybe_unused]]const std::any& cfg) {
  return SUCCESS;
}

void Ros2Communicator::deinit() {
  _ros2ExecutorThread.join();
}

void Ros2Communicator::start() {
  _ros2ExecutorThread = std::thread([this]() {
    LOGG("Spinning on ROS2 executor");
    //blocking call
    _executor.spin();
  });
}

void Ros2Communicator::shutdown() {
  _executor.cancel();
}

void Ros2Communicator::registerNode(const std::shared_ptr<rclcpp::Node> &node) {
  _executor.add_node(node);
}
void Ros2Communicator::unregisterNode(
    const std::shared_ptr<rclcpp::Node> &node) {
  _executor.remove_node(node);
}
