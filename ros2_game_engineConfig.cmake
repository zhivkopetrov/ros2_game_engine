include(CMakeFindDependencyMacro)

#find_dependency will correctly forward REQUIRED or QUIET args to the consumer
#find_package is only for internal use
find_dependency(game_engine REQUIRED)

if(NOT TARGET ros2_game_engine::ros2_game_engine)
  include(${CMAKE_CURRENT_LIST_DIR}/ros2_game_engineTargets.cmake)
endif()

# This is for catkin compatibility.
set(ros2_game_engine_LIBRARIES ros2_game_engine::ros2_game_engine)

get_target_property(
    ros2_game_engine_INCLUDE_DIRS
    ros2_game_engine::ros2_game_engine
    INTERFACE_INCLUDE_DIRECTORIES
)

