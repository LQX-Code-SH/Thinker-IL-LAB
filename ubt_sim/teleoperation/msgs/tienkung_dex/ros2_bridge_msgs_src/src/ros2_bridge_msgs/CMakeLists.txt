cmake_minimum_required(VERSION 3.8)
project(ros2_bridge_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCtrl.msg"
  "msg/ArmCtrl.msg"
  "msg/MotorStatus.msg"
  "msg/HeadCtrl.msg"
  "msg/HeadStatus.msg"
  "msg/WaistStatus.msg"
  "msg/ArmStatus.msg"
  "msg/LegStatus.msg"
  "msg/ImuStatus.msg"
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
