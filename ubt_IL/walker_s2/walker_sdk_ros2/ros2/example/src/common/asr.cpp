#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sys_task_msgs/msg/asr.hpp>

class AsrDemo : public rclcpp::Node
{
public:
    AsrDemo() : Node("asr_demo_node")
    {
        // ASR topic subscriber
        asr_sub_ = this->create_subscription<sys_task_msgs::msg::Asr>(
            "/sys/speech/asr",
            10,
            std::bind(&AsrDemo::asr_callback, this, std::placeholders::_1));

        // Service client to control ASR enable/disable
        enable_client_ = this->create_client<std_srvs::srv::SetBool>("/sys/asr/enable");

        // 等待服务
        if (!enable_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "ASR enable service not available.");
            return;
        }

        // 同步调用
        enable_asr(true);

        RCLCPP_INFO(this->get_logger(), "ASR Demo started. Waiting for ASR messages...");
    }

    ~AsrDemo()
    {
        enable_asr(false);
    }

private:
    rclcpp::Subscription<sys_task_msgs::msg::Asr>::SharedPtr asr_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;

    void enable_asr(bool enable)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        // 直接同步调用
        auto result_future = enable_client_->async_send_request(request);

        // 使用 NodeBaseInterface 同步等待
        auto ret_code = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

        if (ret_code == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(),
                            enable ? "ASR Enabled." : "ASR Disabled.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Request failed: %s", response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /sys/asr/enable service.");
        }
    }

    void asr_callback(const sys_task_msgs::msg::Asr::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "[ASR] text=\"%s\" language=%s wordnum=%d",
                    msg->text.c_str(),
                    msg->language.c_str(),
                    msg->wordnum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AsrDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
