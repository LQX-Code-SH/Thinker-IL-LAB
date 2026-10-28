#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <memory>

class HandsStateSubscriber : public rclcpp::Node
{
public:
  HandsStateSubscriber() : Node("sub_hands_state")
  {
    // 创建QoS配置，使用系统默认的传感器数据QoS
    rclcpp::QoS qos_settings(10);
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_settings.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    
    // 订阅左手关节状态
    left_hand_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/mc/left_hand/joint_states", 
      qos_settings,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->left_hand_callback(msg);
      });
      
    // 订阅右手关节状态
    right_hand_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/mc/right_hand/joint_states", 
      qos_settings,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->right_hand_callback(msg);
      });
  }

private:
  void left_hand_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "======= Left Hand Joint States =======");
    
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::cout << "Joint: " << msg->name[i] 
                << " Position: " << msg->position[i] << std::endl;
    }
  }
  
  void right_hand_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "======= Right Hand Joint States =======");
    
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::cout << "Joint: " << msg->name[i] 
                << " Position: " << msg->position[i] << std::endl;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_hand_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_hand_subscription_;
};

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandsStateSubscriber>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}