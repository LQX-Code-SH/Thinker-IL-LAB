#include <rclcpp/rclcpp.hpp>
#include <mc_task_msgs/srv/walker_motion.hpp>
#include <nlohmann/json.hpp>

#include <chrono>

using WalkerMotion = mc_task_msgs::srv::WalkerMotion;
using json = nlohmann::json;

class UpperBodyServiceClient : public rclcpp::Node
{
public:
  UpperBodyServiceClient() : Node("upper_body_service_client")
  {
    client_ = create_client<WalkerMotion>("/mc/manipulation/service");

    RCLCPP_INFO(get_logger(), "Waiting for /mc/manipulation/service ...");
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Service not available after 10s, exit.");
      rclcpp::shutdown();
      return;
    }

    send_upper_body_goal();
  }

private:
  rclcpp::Client<WalkerMotion>::SharedPtr client_;

  void send_upper_body_goal()
  {
    // ========== 1. 构造 JSON 参数 ==========
    // S2 上半身 16 DOF: waist(2) + left_arm(7) + right_arm(7)
    // ⚠️ 第一个路点必须是当前关节角！实际使用前请先执行:
    //    ros2 topic echo /mc/whole_joint_states --once
    //    将输出填入 goals[0]
    json j;
    j["component_names"] = {"waist", "left_arm", "right_arm"};
    j["goals"] = {
      // 路点1: 当前位置（2026-06-10 读取）
      {0.0, -0.00038,
       -0.00153, -0.14937, -1.56897, -1.56754, 2.87947, -0.00029, 0.00038,
       0.00173, -0.14956, 1.56897, -1.56773, -2.88062, 0.00029, 0.00115},
      // 路点2: 双臂前伸目标
      {0.0, 0.0,
       0.8, 0.3, -0.5, -1.2, 1.5, -0.3, 0.0,
       0.8, -0.3, 0.5, -1.2, -1.5, 0.3, 0.0}
    };
    j["mode"] = 1;           // 轨迹优化器模式（推荐）
    j["vel_scale"] = 0.4;    // 最大关节角速度比例 [0.2, 0.8]

    // ========== 2. 构造 Service Request ==========
    auto request = std::make_shared<WalkerMotion::Request>();
    request->motion_id = "move_components_json";
    request->json_args = j.dump();
    request->cmd = "start";

    RCLCPP_INFO(get_logger(), "Sending request:");
    RCLCPP_INFO(get_logger(), "  motion_id: %s", request->motion_id.c_str());
    RCLCPP_INFO(get_logger(), "  json_args: %s", request->json_args.c_str());

    // ========== 3. 异步调用 ==========
    client_->async_send_request(
      request,
      [this](rclcpp::Client<WalkerMotion>::SharedFuture future) {
        auto response = future.get();
        if (response->ok) {
          RCLCPP_INFO(get_logger(), "Motion succeeded: %s", response->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Motion failed: %s", response->message.c_str());
        }
        rclcpp::shutdown();
      }
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UpperBodyServiceClient>());
  rclcpp::shutdown();
  return 0;
}
