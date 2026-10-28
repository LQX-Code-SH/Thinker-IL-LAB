#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sys_task_msgs/action/tts.hpp"
#include <future>  // for std::promise / std::future

using namespace std::chrono_literals;

class SpeechClient : public rclcpp::Node
{
 public:
  SpeechClient() : Node("simple_speech_client")
  {
    // 创建 Action 客户端
    action_client_ = rclcpp_action::create_client<sys_task_msgs::action::Tts>(
        this, "/sys/speech/tts");

    // 等待 Action Server 可用
    if (!action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    // 声明并获取参数
    this->declare_parameter<std::string>("file_path", "");
    this->get_parameter("file_path", file_path_);

    if (file_path_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "File path parameter not provided");
      return;
    }

    // 发送目标请求
    send_goal();
  }

  std::shared_future<void> get_result_future()
  {
    return result_promise_.get_future();
  }

 private:
  void send_goal()
  {
    sys_task_msgs::action::Tts::Goal goal_msg;
    goal_msg.type = 0;
    goal_msg.is_break = true;
    goal_msg.file_path = file_path_;

    RCLCPP_INFO(this->get_logger(), "Sending goal with file path: %s",
                file_path_.c_str());

    rclcpp_action::Client<sys_task_msgs::action::Tts>::SendGoalOptions options;

    // Goal response 回调
    options.goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<
                   sys_task_msgs::action::Tts>> goal_handle) {
          if (!goal_handle)
          {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            result_promise_.set_value();  // 提前结束
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
          }
        };

    // Feedback 回调（这里可以忽略或打印）
    options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<
                   sys_task_msgs::action::Tts>::SharedPtr,
               const std::shared_ptr<const sys_task_msgs::action::Tts::Feedback>
                   feedback) {
          RCLCPP_INFO(this->get_logger(), "Feedback received...");
        };

    // Result 回调
    options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               sys_task_msgs::action::Tts>::WrappedResult &result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "Result received successfully");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code");
              break;
          }
          result_promise_.set_value();  // 通知 main 可以退出
        };

    action_client_->async_send_goal(goal_msg, options);
  }

  std::string file_path_;
  rclcpp_action::Client<sys_task_msgs::action::Tts>::SharedPtr action_client_;

  std::promise<void> result_promise_;  // 用于退出控制
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeechClient>();

  // 等待 result 回调通知
  auto future = node->get_result_future();
  rclcpp::spin_until_future_complete(node, future);

  rclcpp::shutdown();
  return 0;
}
