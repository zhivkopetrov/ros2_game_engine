#ifndef ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_
#define ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_

//System headers
#include <cstdint>
#include <thread>
#include <memory>

//Other libraries headers
#include <rclcpp/executor.hpp>
#include "game_engine/communicator/Communicator.h"

//Own components headers
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"

//Forward declarations

class Ros2Communicator final : public Communicator {
public:
  ErrorCode init(const std::any& cfg) override;
  void deinit() override;

  void start() override;
  void shutdown() override;

  void registerNode(const std::shared_ptr<rclcpp::Node>& node);
  void unregisterNode(const std::shared_ptr<rclcpp::Node>& node);

private:
  void createExecutor(const Ros2CommunicatorConfig& cfg);

  ExecutionPolicy _executionPolicy = ExecutionPolicy::RUN_IN_DEDICATED_THREAD;
  std::thread _ros2ExecutorThread;

  /* StaticSingleThreadedExecutor expects all nodes to have their
   * publishers, subscriptions, ros timers, etc...created during initialization
   *
   * NOTE: constructor of the executor expects ros to be initialized
   * */
  std::unique_ptr<rclcpp::Executor> _executor;
};

#endif /* ROS2_GAME_ENGINE_ROS2COMMUNICATOR_H_ */
