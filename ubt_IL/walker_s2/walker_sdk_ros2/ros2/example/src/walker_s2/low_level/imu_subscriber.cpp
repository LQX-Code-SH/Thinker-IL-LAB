#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuSubscriber : public rclcpp::Node
{
 public:
  ImuSubscriber() : Node("imu_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensor/imu/orin",  // IMU话题名称
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&ImuSubscriber::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Orientation: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                msg->orientation.w, msg->orientation.x, msg->orientation.y,
                msg->orientation.z);

    RCLCPP_INFO(this->get_logger(),
                "Angular velocity: [x=%.3f, y=%.3f, z=%.3f] rad/s",
                msg->angular_velocity.x, msg->angular_velocity.y,
                msg->angular_velocity.z);

    RCLCPP_INFO(this->get_logger(),
                "Linear acceleration: [x=%.3f, y=%.3f, z=%.3f] m/s^2",
                msg->linear_acceleration.x, msg->linear_acceleration.y,
                msg->linear_acceleration.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSubscriber>());
  rclcpp::shutdown();
  return 0;
}
