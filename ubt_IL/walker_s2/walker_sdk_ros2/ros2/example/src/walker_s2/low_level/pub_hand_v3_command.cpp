#include <mc_task_msgs/msg/joint_command.hpp>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pub_hand_command");
  
  // Create publishers for left and right hand controllers
  auto left_hand_publisher = node->create_publisher<mc_task_msgs::msg::JointCommand>(
    "/mc/left_hand/command", 10);
  auto right_hand_publisher = node->create_publisher<mc_task_msgs::msg::JointCommand>(
    "/mc/right_hand/command", 10);
    
  // Define joint names for left hand
  std::vector<std::string> left_joint_names = {
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp"
  };
  
  // Define joint names for right hand
  std::vector<std::string> right_joint_names = {
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp"
  };
    
  rclcpp::Rate rate(500);
  double time_cnt = 0.0;
  
  while (rclcpp::ok()) {
    // Publish joint commands for left hand
    mc_task_msgs::msg::JointCommand left_cmd;
    left_cmd.header.stamp = node->now();
    left_cmd.names.resize(left_joint_names.size());
    left_cmd.position.resize(left_joint_names.size());
    left_cmd.mode.resize(left_joint_names.size());
    
    for (size_t i = 0; i < left_joint_names.size(); i++) {
      left_cmd.names[i] = left_joint_names[i];
      left_cmd.position[i] = sin(time_cnt + i * 0.2) * 0.3;  // Add phase difference for each joint
      left_cmd.mode[i] = 5;
    }
    
    // Publish joint commands for right hand
    mc_task_msgs::msg::JointCommand right_cmd;
    right_cmd.header.stamp = node->now();
    right_cmd.names.resize(right_joint_names.size());
    right_cmd.position.resize(right_joint_names.size());
    right_cmd.mode.resize(right_joint_names.size());
    
    for (size_t i = 0; i < right_joint_names.size(); i++) {
      right_cmd.names[i] = right_joint_names[i];
      right_cmd.position[i] = sin(time_cnt + i * 0.2) * 0.3;  // Add phase difference for each joint
      right_cmd.mode[i] = 5;
    }
    
    // Publish commands
    left_hand_publisher->publish(left_cmd);
    right_hand_publisher->publish(right_cmd);
    
    time_cnt += 0.002;
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}