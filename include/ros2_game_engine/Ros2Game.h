#ifndef ROS2_GAME_ENGINE_ROS2GAME_H_
#define ROS2_GAME_ENGINE_ROS2GAME_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "game_engine/Game.h"

//Own components headers
#include "ros2_game_engine/defines/Ros2GameEngineFunctionalDefines.h"

//Forward declarations

class Ros2Game : public Game {
public:
  virtual int32_t initNodes() = 0;

  void setNodeCallbacks(const RegisterNodeCb& registerNodeCb,
                        const UnregisterNodeCb& unregisterNodeCb) {
    _registerNodeCb = registerNodeCb;
    _unregisterNodeCb = unregisterNodeCb;
  }

protected:
  RegisterNodeCb _registerNodeCb;
  UnregisterNodeCb _unregisterNodeCb;
};

#endif /* ROS2_GAME_ENGINE_ROS2GAME_H_ */
