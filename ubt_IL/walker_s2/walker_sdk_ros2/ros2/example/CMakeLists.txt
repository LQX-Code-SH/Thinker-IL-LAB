cmake_minimum_required(VERSION 3.16)
project(ubt_ros2_example)

# find_require_package
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(emb_task_msgs REQUIRED)
find_package(sys_task_msgs REQUIRED)
find_package(mc_state_msgs REQUIRED)
find_package(mc_task_msgs REQUIRED)
find_package(shm_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 自动扫描 src 下所有 cpp 文件
file(GLOB_RECURSE CPP_SOURCES "src/**/*.cpp")

# 遍历每个文件，生成同名可执行文件
foreach(SRC_FILE ${CPP_SOURCES})
  get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)
  add_executable(${TARGET_NAME} ${SRC_FILE})
  ament_target_dependencies(
    ${TARGET_NAME}
    rclcpp
    sensor_msgs
    mc_state_msgs
    mc_task_msgs
    emb_task_msgs
    sys_task_msgs
    rclcpp_action
    shm_msgs
    std_srvs)
  install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
