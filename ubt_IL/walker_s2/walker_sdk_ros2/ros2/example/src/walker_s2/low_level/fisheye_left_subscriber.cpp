#include "rclcpp/rclcpp.hpp"
#include "shm_msgs/msg/image2m.hpp"

class FishEyeLeftSubscriber : public rclcpp::Node
{
 public:
  FishEyeLeftSubscriber() : Node("fish_eye_left_subscriber")
  {
    subscription_ = this->create_subscription<shm_msgs::msg::Image2m>(
        "/sensor/camera/fisheye_left/image/raw",  // 话题名称
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&FishEyeLeftSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

 private:
  void topic_callback(const shm_msgs::msg::Image2m::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Cruent Time: %d sec %u nanosec",
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "Frame id: %s",
                reinterpret_cast<char *>(msg->header.frame_id.data.data()));
    RCLCPP_INFO(this->get_logger(), "Height * Width:  %u * %u", msg->height,
                msg->width);
    RCLCPP_INFO(this->get_logger(), "Encoding: %s",
                reinterpret_cast<char *>(msg->encoding.data.data()));
    RCLCPP_INFO(this->get_logger(), "Bigendian: %u", msg->is_bigendian);
    RCLCPP_INFO(this->get_logger(), "Step: %u", msg->step);
    RCLCPP_INFO(this->get_logger(), "Matrix data length: %zu",
                sizeof(msg->data) / sizeof(msg->data[0]));
  }

  rclcpp::Subscription<shm_msgs::msg::Image2m>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FishEyeLeftSubscriber>());
  rclcpp::shutdown();
  return 0;
}
