#ifndef ROS2_GAME_ENGINE_ROS2GAMEENGINEFUNCTIONALDEFINES_H_
#define ROS2_GAME_ENGINE_ROS2GAMEENGINEFUNCTIONALDEFINES_H_

//C system headers

//C++ system headers
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

#endif /* ROS2_GAME_ENGINE_ROS2GAMEENGINEFUNCTIONALDEFINES_H_ */
