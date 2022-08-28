//Corresponding header
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"

//System headers

//Other libraries headers

//Own components headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

std::string getExecutorName(ExecutorType executorType) {
  switch (executorType) {
  case ExecutorType::SINGLE_THREADED:
    return "SINGLE_THREADED";

  case ExecutorType::STATIC_SINGLE_THREADED:
    return "STATIC_SINGLE_THREADED";

  case ExecutorType::MULTI_THREADED:
    return "MULTI_THREADED";

  default:
    LOGERR("Error, received unsupported ExecutorType: [%d]",
        getEnumValue(executorType));
    return "UNSUPPORTED_EXECUTOR";
  }
}
