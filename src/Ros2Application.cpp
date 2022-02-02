//Corresponding header
#include "ros2_game_engine/Ros2Application.h"

//C system headers

//C++ system headers

#include <csignal>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

Ros2Application::Ros2Application(std::unique_ptr<Game> game)
    : Application(std::move(game)) {

}

Ros2Application::~Ros2Application() noexcept {
  _executor->cancel();
  _ros2ExecutorThread.join();
}

int32_t Ros2Application::run(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes) {
  _ros2ExecutorThread = std::thread([this, &nodes]() {
    for (const auto& node: nodes) {
      _executor->add_node(node);
    }

    LOGG("Spinning on ROS2 executor");
    //blocking call
    _executor->spin();
  });

  //give some time for executor to start
  using namespace std::literals;
  std::this_thread::sleep_for(2ms);

  return Application::run();
}

int32_t Ros2Application::loadDependencies(int32_t argc, char **args) {
  if (SUCCESS != Application::loadDependencies(argc, args)) {
    LOGERR("Error in Application::loadDependencies() -> Terminating ...");
    return FAILURE;
  }

  rclcpp::init(argc, args);
  _executor =
      std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
  return SUCCESS;
}

void Ros2Application::unloadDependencies() {
  rclcpp::shutdown();
  Application::unloadDependencies();
}
