#ifndef ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_
#define ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <thread>

//Other libraries headers
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include "game_engine/communicator/Communicator.h"

//Own components headers

//Forward declarations

class Ros2Communicator final : public Communicator {
public:
  int32_t init(const std::any& cfg) override;
  void deinit() override;

  void start() override;
  void shutdown() override;

  void registerNode(const std::shared_ptr<rclcpp::Node>& node);
  void unregisterNode(const std::shared_ptr<rclcpp::Node>& node);

private:
  std::thread _ros2ExecutorThread;

  /* StaticSingleThreadedExecutor expects all nodes to have their
   * publishers, subscriptions, ros timers, etc...created during initialization
   *
   * NOTE: constructor of the executor expects ros to be initialized
   * */
  rclcpp::executors::StaticSingleThreadedExecutor _executor;
};

#endif /* ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_ */
