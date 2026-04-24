cmake_minimum_required(VERSION 3.22)
project(rm_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeneralMessage.msg"
  "msg/SendMessage.msg"
  "msg/RemoteControl.msg"
  "msg/GameState.msg"
  "msg/GameResult.msg"
  "msg/RobotHP.msg"
  "msg/FieldEvents.msg"
  "msg/RefereeWarning.msg"
  "msg/DartInfo.msg"
  "msg/RobotStatus.msg"
  "msg/BufferAndHeat.msg"
  "msg/RobotBuffs.msg"
  "msg/HurtEvent.msg"
  "msg/RFIDStatus.msg"
  "msg/SentryDecision.msg"
  "msg/RadarDecision.msg"
  "msg/RobotInteraction.msg"
  "msg/KeyMouseSimulation.msg"
  "msg/RefRemoteControl.msg"
  "msg/SetVTMChannel.msg"
  "msg/QueryVTMChannel.msg"
  "msg/RobotPosition.msg"
  "msg/ShootData.msg"
  "msg/AmmoAllowance.msg"
  "msg/DartCmd.msg"
  "msg/GroundPositions.msg"
  "msg/RadarMark.msg"
  "msg/CustomController.msg"
  "msg/MapDownlink.msg"
  "msg/RadarMapUplink.msg"
  "msg/PathUplink.msg"
  "msg/CustomString.msg"
  "msg/RobotCustomData.msg"
  "msg/RobotCustomDataLarge.msg"
  "msg/ClientCustomCommand.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
