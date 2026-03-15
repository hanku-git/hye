cmake_minimum_required(VERSION 3.5)

project(kcr_control_msg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/RscState.msg"
  "msg/InputParameter.msg"
  "msg/OutputParameter.msg"
  "srv/Impedance.srv"
  "srv/MoveQ.srv"
  "srv/MoveX.srv"
  "srv/SingleBool.srv"
  "srv/SingleString.srv"
  "srv/StopRobot.srv"
  "srv/TcpPayload.srv"
  "srv/OperatingStatus.srv"
  "srv/Blending.srv"
  "srv/Jog.srv"
  "srv/XpadJog.srv"
  "srv/DirectTorqueControl.srv"
  "srv/Command.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()