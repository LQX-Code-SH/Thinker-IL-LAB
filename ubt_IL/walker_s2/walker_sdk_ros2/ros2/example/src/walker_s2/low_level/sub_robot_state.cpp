#include "rclcpp/rclcpp.hpp"
#include "mc_state_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <memory>

class RobotStateSubscriber : public rclcpp::Node
{
public:
  RobotStateSubscriber() : Node("sub_robot_state")
  {
    // 创建QoS配置，使用系统默认的传感器数据QoS
    rclcpp::QoS qos_settings(10);
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_settings.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    
    subscription_ = this->create_subscription<mc_state_msgs::msg::RobotState>(
      "/mc/sdk/robot_state", 
      qos_settings,
      std::bind(&RobotStateSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const mc_state_msgs::msg::RobotState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "================================");
    
    for (size_t i = 0; i < msg->joint_states.name.size(); i++) {
      std::cout << "Joint: " << msg->joint_states.name[i] 
                << " Joint position: " << msg->joint_states.position[i] << std::endl;
    }

    for (const auto& item : msg->imu_states) {
      std::cout << "Imu: " << item.header.frame_id 
                << ", acc:" << item.linear_acceleration.x << " "
                << item.linear_acceleration.y << " " << item.linear_acceleration.z
                << ", gyro:" << item.angular_velocity.x << " " 
                << item.angular_velocity.y << " "
                << item.angular_velocity.z << std::endl;
    }

    for (const auto& item : msg->ft_states) {
      std::cout << "Ft: " << item.header.frame_id 
                << ", force: " << item.wrench.force.x << " "
                << item.wrench.force.y << " " << item.wrench.force.z 
                << ", torque: " << item.wrench.torque.x << " "
                << item.wrench.torque.y << " " << item.wrench.torque.z << std::endl;
    }
  }

  rclcpp::Subscription<mc_state_msgs::msg::RobotState>::SharedPtr subscription_;
};

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStateSubscriber>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}