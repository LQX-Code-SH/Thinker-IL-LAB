cmake_minimum_required(VERSION 3.5)
project(sys_task_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosa_msgs REQUIRED)

# ---- 自动收集 msg/srv/action 但确保是“相对路径” ----

# msg
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

# srv
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")

# action
file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")

# ---- 生成接口 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}

  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    rosa_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
