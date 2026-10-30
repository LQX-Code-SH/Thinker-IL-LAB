cmake_minimum_required(VERSION 3.16)
project(shm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 自动收集 msg/srv/action 文件
file(
  GLOB_RECURSE msg_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg")
file(
  GLOB_RECURSE srv_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "srv/*.srv")
file(
  GLOB_RECURSE action_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "action/*.action")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
