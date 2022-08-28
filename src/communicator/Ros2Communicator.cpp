//Corresponding header
#include "ros2_game_engine/communicator/Ros2Communicator.h"

//System headers

//Other libraries headers
#include <rclcpp/executors.hpp>
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"

ErrorCode Ros2Communicator::init(const std::any& cfg) {
  ErrorCode err = ErrorCode::SUCCESS;
  const Ros2CommunicatorConfig parsedCfg = [&cfg, &err]() {
    Ros2CommunicatorConfig localCfg;
    try {
      localCfg = std::any_cast<const Ros2CommunicatorConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<Ros2CommunicatorConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing Ros2CommunicatorConfig failed");
    return ErrorCode::FAILURE;
  }

  createExecutor(parsedCfg);
  return ErrorCode::SUCCESS;
}

void Ros2Communicator::deinit() {
  _ros2ExecutorThread.join();
}

void Ros2Communicator::start() {
  _ros2ExecutorThread = std::thread([this]() {
    LOG("Spinning on ROS2 Executor");
    //blocking call
    _executor->spin();
  });
}

void Ros2Communicator::shutdown() {
  _executor->cancel();
}

void Ros2Communicator::registerNode(const std::shared_ptr<rclcpp::Node> &node) {
  _executor->add_node(node);
}
void Ros2Communicator::unregisterNode(
    const std::shared_ptr<rclcpp::Node> &node) {
  _executor->remove_node(node);
}

void Ros2Communicator::createExecutor(const Ros2CommunicatorConfig& cfg) {
  switch (cfg.executorType) {
  case ExecutorType::SINGLE_THREADED:
    _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    LOG("Creating ROS2 SingleThreadedExecutor");
    break;
  case ExecutorType::STATIC_SINGLE_THREADED:
    _executor =
        std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    LOG("Creating ROS2 StaticSingleThreadedExecutor");
    break;
  case ExecutorType::MULTI_THREADED: {
    _executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), cfg.numberOfThreads);
    const size_t threadsCtn = (0 == cfg.numberOfThreads) ?
        std::thread::hardware_concurrency() : cfg.numberOfThreads;
    LOG("Creating ROS2 MultiThreadedExecutor with: [%zu] threads", threadsCtn);
  }
    break;
  default:
    LOGERR("Error, received unsupported ExecutorType: [%d]. Defaulting to "
           "SINGLE_THREADED executor", getEnumValue(cfg.executorType));
    _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    break;
  }
}
