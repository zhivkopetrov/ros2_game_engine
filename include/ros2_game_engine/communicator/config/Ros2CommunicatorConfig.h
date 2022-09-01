#ifndef ROS2_GAME_ENGINE_ROS2COMMUNICATORCONFIG_H_
#define ROS2_GAME_ENGINE_ROS2COMMUNICATORCONFIG_H_

//System headers
#include <cstddef>
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

enum class ExecutorType {
  SINGLE_THREADED,
  STATIC_SINGLE_THREADED,
  MULTI_THREADED
};

enum class ExecutionPolicy {
  BLOCKING,               //will block calling thread
  RUN_IN_DEDICATED_THREAD //will spawn a new thread and block there
};

struct Ros2CommunicatorConfig {
  ExecutorType executorType = ExecutorType::SINGLE_THREADED;

  //used in case of multithreaded executor
  //default value of 0 will use number of cpu threads
  size_t numberOfThreads = 0;

  ExecutionPolicy executionPolicy = ExecutionPolicy::RUN_IN_DEDICATED_THREAD;
};

std::string getExecutorName(ExecutorType executorType);

#endif /* ROS2_GAME_ENGINE_ROS2COMMUNICATORCONFIG_H_ */
