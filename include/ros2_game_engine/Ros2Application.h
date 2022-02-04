#ifndef ROS2_GAME_ENGINE_ROS2APPLICATION_H_
#define ROS2_GAME_ENGINE_ROS2APPLICATION_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <memory>
#include <thread>

//Other libraries headers
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include "game_engine/Application.h"

//Own components headers

//Forward declarations

class Ros2Application : public Application {
public:
  Ros2Application(std::unique_ptr<Game> game);
  ~Ros2Application() noexcept;

  int32_t init(const ApplicationConfig& cfg);

  int32_t run();

  void registerNode(const std::shared_ptr<rclcpp::Node>& node);
  void unregisterNode(const std::shared_ptr<rclcpp::Node>& node);

private:
  int32_t loadDependencies(int32_t argc, char **args) override;
  void unloadDependencies() override;

  std::thread _ros2ExecutorThread;

  /* StaticSingleThreadedExecutor expects all nodes to have their
   * publishers, subscriptions, ros timers, etc...created during initialization
   *
   * NOTE: constructor of the executor expects ros to be initialized
   * */
  std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> _executor;
};

#endif /* ROS2_GAME_ENGINE_ROS2APPLICATION_H_ */
