#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sys_task_msgs/action/tts.hpp>

class TtsClient : public rclcpp::Node
{
 public:
  using Tts = sys_task_msgs::action::Tts;
  using GoalHandleTts = rclcpp_action::ClientGoalHandle<Tts>;

  TtsClient() : Node("tts_client_demo")
  {
    client_ = rclcpp_action::create_client<Tts>(this,
                                                "/sys/speech/tts"  // action 名
    );

    // 等待服务器
    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(get_logger(), "TTS action server not available!");
      return;
    }

    send_tts_goal();
  }

 private:
  rclcpp_action::Client<Tts>::SharedPtr client_;

  void send_tts_goal()
  {
    Tts::Goal goal;

    goal.type = Tts::Goal::TTS;  // 使用语音合成
    goal.is_break = true;

    // TTS 参数
    goal.text = "你好，我是语音合成测试。";
    goal.speaker = "male_01";
    goal.speed = 50;
    goal.volume = 100;
    goal.pitch = 50;
    goal.language = "zh";
    goal.format = "wav";
    goal.need_save = true;

    RCLCPP_INFO(this->get_logger(), "Sending TTS goal...");

    auto send_goal_options = rclcpp_action::Client<Tts>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&TtsClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal, send_goal_options);
  }

  void result_callback(const GoalHandleTts::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "TTS Succeeded.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "TTS Aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "TTS Canceled.");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown result code.");
        break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TtsClient>());
  rclcpp::shutdown();
  return 0;
}
