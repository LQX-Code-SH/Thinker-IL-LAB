cmake_minimum_required(VERSION 3.5)
project(ecat_task_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shm_msgs REQUIRED)

# ---- 自动收集 msg/srv/action 文件（相对路径） ----
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")

# ---- 生成接口 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}

  DEPENDENCIES
    shm_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
