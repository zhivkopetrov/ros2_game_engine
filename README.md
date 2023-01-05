# ros2_game_engine

## A C++ ROS2 communicator static library
The library is meant to be used as extension of the game_engine library.
- https://github.com/zhivkopetrov/game_engine

## The ROS2 communicator library supports:
- Convinient way to create and configure the standard executors provided by ROS2
- Ability to block the calling thread or spawn a dedicated thread, where the executor will run
- The supported executors are:
```
- SINGLE_THREADED - single thread callback execution. ROS2 timers and callbacks could be dynamically added/removed
- STATIC_SINGLE_THREADED - single thread callback execution. ROS2 timers and callbacks can only be provided during initialization. This enables efficient work of the executor
- MULTI_THREADED - multi thread callback execution. ROS2 timers and callbacks could be dynamically added/removed. Callbacks should be configured using ROS2 callback_groups
```

## Library specifics
- A ROS2 node should be explicitly added to the executor before sppinning (running). Use the provided API
- The ROS2 communicator implemented in the library can easily be constructed as game_engine communicator

Examples:
```
static std::unique_ptr<RoboCollectorGui> createRoboCollectorGui(
    const std::unique_ptr<Ros2Communicator>& communicator) {
  using namespace std::placeholders;
  Ros2CommunicatorInterface interface;
  interface.registerNodeCb =
      std::bind(&Ros2Communicator::registerNode, communicator.get(), _1);
  interface.unregisterNodeCb =
      std::bind(&Ros2Communicator::unregisterNode, communicator.get(), _1);

  return std::make_unique<RoboCollectorGui>(interface);
}

int32_t main(int32_t argc, char **args) {
  Application app;

  const auto dependencies =
      RoboCollectorGuiConfigGenerator::generateDependencies(argc, args);
  if (ErrorCode::SUCCESS != app.loadDependencies(dependencies)) {
    LOGERR("app.loadDependencies() failed");
    return EXIT_FAILURE;
  }

  auto communicator = std::make_unique<Ros2Communicator>();
  auto game = createRoboCollectorGui(communicator);
  app.obtain(std::move(game), std::move(communicator));

  const auto cfg = RoboCollectorGuiConfigGenerator::generateConfig();
  if (ErrorCode::SUCCESS != app.init(cfg)) {
    LOGERR("app.init() failed");
    return EXIT_FAILURE;
  }

  if (ErrorCode::SUCCESS != app.run()) {
    LOGERR("app.run() failed");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
```

## Usage from plain CMake
- Consume directly with find_package(ros2_game_engine) in a CMakeLists.txt
- Link against your target with suitable access modifier
```
target_link_libraries(
    ${PROJECT_NAME} 
    PUBLIC
        ros2_game_engine::ros2_game_engine
)
```

## Usage as part ROS2(colcon) meta-build systems
- Consume directly with find_package(ros2_game_engine) in the package CMakeLists.txt
- Link agains your target
- The library automatically exposes and install it's artifacts following ROS2 structure
- Example usage project: https://github.com/zhivkopetrov/robotics_v1

## Dependencies
- cmake_helpers - https://github.com/zhivkopetrov/cmake_helpers.git
- utils - https://github.com/zhivkopetrov/utils
- resource_utils - https://github.com/zhivkopetrov/resource_utils
- sdl_utils - https://github.com/zhivkopetrov/sdl_utils
- manager_utils - https://github.com/zhivkopetrov/manager_utils
- game_engine - https://github.com/zhivkopetrov/game_engine

## Supported Platforms
Linux:
  - g++ 12
  - clang++ 14
  - Emscripten (em++) 3.1.28
  - Robot Operating System 2 (ROS2)
    - Through colcon meta-build system (CMake based)
  - Robot Operating System 1 (ROS1)
    - Through catkin meta-build system (CMake based)
      - Due to soon ROS1 end-of-life catkin builds are not actively supported

Windows:
  - MSVC++ (>= 14.20) Visual Studio 2019