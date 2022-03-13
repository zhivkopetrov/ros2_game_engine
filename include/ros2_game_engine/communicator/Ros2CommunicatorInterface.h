#ifndef ROS2_GAME_ENGINE_ROS2COMMUNICATOR_INTERFACE_H_
#define ROS2_GAME_ENGINE_ROS2COMMUNICATOR_INTERFACE_H_

//System headers
#include <cstdint>
#include <memory>
#include <functional>

//Other libraries headers

//Own components headers

//Forward declarations
namespace rclcpp {
class Node;
} //namespace rclcpp

using RegisterNodeCb =
    std::function<void(const std::shared_ptr<rclcpp::Node>& node)>;
using UnregisterNodeCb =
    std::function<void(const std::shared_ptr<rclcpp::Node>& node)>;

struct Ros2CommunicatorInterface {
  RegisterNodeCb registerNodeCb;
  UnregisterNodeCb unregisterNodeCb;
};

#endif /* ROS2_GAME_ENGINE_ROS2COMMUNICATOR_INTERFACE_H_ */
