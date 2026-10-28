#include "emb_task_msgs/msg/inner_data.hpp"
#include "rclcpp/rclcpp.hpp"

class InnerDataSubscriber : public rclcpp::Node
{
 public:
  InnerDataSubscriber() : Node("inner_data_subscriber")
  {
    subscription_ = this->create_subscription<emb_task_msgs::msg::InnerData>(
        "/emb/powerboard_innerdata",  // 话题名称
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&InnerDataSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

 private:
  void topic_callback(const emb_task_msgs::msg::InnerData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "------ Inner Data ------");
    RCLCPP_INFO(this->get_logger(), "Orin Voltage: %.3f V",
                msg->adc_orin_value);
    RCLCPP_INFO(this->get_logger(), "Orin Current: %.3f A",
                msg->adc_orin_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Arm Current: %.3f A",
                msg->adc_arm_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Total Current: %.3f A",
                msg->adc_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Left Leg Current: %.3f A",
                msg->adc_leftleg_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Right Leg Current: %.3f A",
                msg->adc_rightleg_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Waist Current: %.3f A",
                msg->adc_waist_ibus_value);
    RCLCPP_INFO(this->get_logger(), "Charge Voltage: %.3f V",
                msg->adc_charge_det_value);
    RCLCPP_INFO(this->get_logger(), "Total Voltage: %.3f V",
                msg->adc_vdc1_value);
    RCLCPP_INFO(this->get_logger(), "MOSFET Temp: %.3f °C", msg->adc_mos_temp);
    RCLCPP_INFO(this->get_logger(), "5V Output Voltage: %.3f V", msg->adc_1v5);
    RCLCPP_INFO(this->get_logger(), "Chip Temp: %.3f °C", msg->temptature);
    RCLCPP_INFO(this->get_logger(), "Reference Voltage: %.3f V", msg->vrefint);
    RCLCPP_INFO(this->get_logger(), "X86 Current: %.3f A",
                msg->adc_x86_ibus_value);
    RCLCPP_INFO(this->get_logger(), "RK Current: %.3f A",
                msg->adc_rk_ibus_value);
    RCLCPP_INFO(this->get_logger(), "3V Reference Voltage: %.3f V",
                msg->adc_3vref_value);
    RCLCPP_INFO(this->get_logger(), "Error Code: %u", msg->err_code);
  }

  rclcpp::Subscription<emb_task_msgs::msg::InnerData>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InnerDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}
