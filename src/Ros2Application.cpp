//Corresponding header
#include "ros2_game_engine/Ros2Application.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ros2_game_engine/Ros2Game.h"

Ros2Application::Ros2Application(std::unique_ptr<Game> game)
    : Application(std::move(game)) {

}

Ros2Application::~Ros2Application() noexcept {
  _executor->cancel();
  _ros2ExecutorThread.join();
}

int32_t Ros2Application::init(const ApplicationConfig &cfg) {
  if (SUCCESS != Application::init(cfg)) {
    LOGERR("Application::init() failed");
    return FAILURE;
  }

  _executor =
      std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();

  using namespace std::placeholders;
  const auto registerNodeCb =
      std::bind(&Ros2Application::registerNode, this, _1);
  const auto unregisterNodeCb =
      std::bind(&Ros2Application::unregisterNode, this, _1);

  Ros2Game *ros2Game = static_cast<Ros2Game*>(_game.get());
  ros2Game->setNodeCallbacks(registerNodeCb, unregisterNodeCb);

  if (SUCCESS != ros2Game->initNodes()) {
    LOGERR("ros2Game->initNodes() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t Ros2Application::run() {
  _ros2ExecutorThread = std::thread([this]() {
    LOGG("Spinning on ROS2 executor");
    //blocking call
    _executor->spin();
  });

  //give some time for executor to start
  using namespace std::literals;
  std::this_thread::sleep_for(2ms);

  return Application::run();
}

void Ros2Application::registerNode(const std::shared_ptr<rclcpp::Node> &node) {
  _executor->add_node(node);
}
void Ros2Application::unregisterNode(
    const std::shared_ptr<rclcpp::Node> &node) {
  _executor->remove_node(node);
}

