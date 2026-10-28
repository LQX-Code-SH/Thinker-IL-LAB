#include "emb_task_msgs/msg/battery_state.hpp"
#include "rclcpp/rclcpp.hpp"

class BatterySubscriber : public rclcpp::Node
{
 public:
  BatterySubscriber() : Node("battery_subscriber")
  {
    subscription_ = this->create_subscription<emb_task_msgs::msg::BatteryState>(
        "/emb/battery_state", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&BatterySubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

 private:
  void topic_callback(const emb_task_msgs::msg::BatteryState::SharedPtr msg)
  {
    int size = msg->batteries_states.size();
    RCLCPP_INFO(this->get_logger(), "-----------------------------");
    for (int i = 0; i < size; i++)
    {
      RCLCPP_INFO(this->get_logger(), "------Battery %d state:-----", i + 1);
      RCLCPP_INFO(this->get_logger(), "  Charge status: %s",
                  msg->batteries_states[i].charge_status.c_str());
      RCLCPP_INFO(this->get_logger(), "  Voltage: %.3f V",
                  msg->batteries_states[i].voltage);
      RCLCPP_INFO(this->get_logger(), "  Current: %.3f A",
                  msg->batteries_states[i].current);
      RCLCPP_INFO(this->get_logger(), "  Temperature: %.3f °C",
                  msg->batteries_states[i].temperature);
      RCLCPP_INFO(this->get_logger(), "  Max voltage diff: %.3f V",
                  msg->batteries_states[i].maxdifvol);
      RCLCPP_INFO(this->get_logger(), "  Battery SOC: %.3f %%",
                  msg->batteries_states[i].batsoc);
      RCLCPP_INFO(this->get_logger(), "  Remaining charge time: %.3f s",
                  msg->batteries_states[i].remainchargetime);
      RCLCPP_INFO(this->get_logger(), "  Health status: %u",
                  msg->batteries_states[i].healthstatus);
      RCLCPP_INFO(this->get_logger(), "  Remaining use life: %.3f times",
                  msg->batteries_states[i].remainuselife);
    }
  }

  rclcpp::Subscription<emb_task_msgs::msg::BatteryState>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySubscriber>());
  rclcpp::shutdown();
  return 0;
}
