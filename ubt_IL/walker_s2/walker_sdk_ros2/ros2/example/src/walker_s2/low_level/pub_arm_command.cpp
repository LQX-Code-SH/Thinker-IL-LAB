#include <mc_task_msgs/msg/robot_command.hpp>
#include <mc_task_msgs/msg/joint_cmd.hpp>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pub_head_command");
  auto cmd_publisher_ = node->create_publisher<mc_task_msgs::msg::RobotCommand>("/mc/sdk/robot_command", 10);
  rclcpp::Rate rate(500);
  double time_cnt = 0.0;
  
  while (rclcpp::ok()) {
    mc_task_msgs::msg::RobotCommand cmd;
    cmd.header.stamp = node->now();

    mc_task_msgs::msg::JointCmd L_shoulder_pitch;
    L_shoulder_pitch.name = "L_elbow_yaw_joint";
    L_shoulder_pitch.control_mode = mc_task_msgs::msg::JointCmd::MODE_POSITION;
    L_shoulder_pitch.position = sin(time_cnt) * 0.5;
    cmd.joint_cmd.push_back(L_shoulder_pitch);

    cmd_publisher_->publish(cmd);

    time_cnt += 0.002;
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}