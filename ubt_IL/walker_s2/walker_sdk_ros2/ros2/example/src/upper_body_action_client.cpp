#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mc_task_msgs/action/arm_task.hpp>
#include <nlohmann/json.hpp>

#include <chrono>

using ArmTask = mc_task_msgs::action::ArmTask;
using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;
using json = nlohmann::json;

class UpperBodyActionClient : public rclcpp::Node
{
public:
  UpperBodyActionClient() : Node("upper_body_action_client")
  {
    client_ = rclcpp_action::create_client<ArmTask>(this, "/mc/manipulation/action");

    RCLCPP_INFO(get_logger(), "Waiting for /mc/manipulation/action ...");
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after 10s, exit.");
      return;
    }

    send_upper_body_goal();
  }

private:
  rclcpp_action::Client<ArmTask>::SharedPtr client_;

  void send_upper_body_goal()
  {
    // ========== 1. 构造 JSON 参数（与 Service 方式相同）==========
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

    // ========== 2. 构造 Action Goal ==========
    auto goal = ArmTask::Goal();
    goal.task_name = "move_components_json";
    goal.yaml_args = j.dump();

    RCLCPP_INFO(get_logger(), "Sending action goal:");
    RCLCPP_INFO(get_logger(), "  task_name: %s", goal.task_name.c_str());

    // ========== 3. 设置回调 ==========
    auto send_goal_options = rclcpp_action::Client<ArmTask>::SendGoalOptions();

    // goal 被服务端接受/拒绝
    send_goal_options.goal_response_callback =
      [this](const GoalHandleArmTask::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
          rclcpp::shutdown();
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result...");
        }
      };

    // 执行进度反馈
    send_goal_options.feedback_callback =
      [this](GoalHandleArmTask::SharedPtr,
             const std::shared_ptr<const ArmTask::Feedback> feedback) {
        RCLCPP_INFO(get_logger(),
          "Feedback: state=%u, progress=%.1f/%.1f s",
          feedback->state.state,
          feedback->current_time,
          feedback->total_time);
      };

    // 最终结果
    send_goal_options.result_callback =
      [this](const GoalHandleArmTask::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Action succeeded (state=%u, desc=%s)",
              result.result->state.state,
              result.result->state.desc.c_str());
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Action aborted (state=%u, desc=%s)",
              result.result->state.state,
              result.result->state.desc.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Action canceled");
            break;
          default:
            RCLCPP_WARN(get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
            break;
        }
        rclcpp::shutdown();
      };

    // ========== 4. 异步发送 ==========
    client_->async_send_goal(goal, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UpperBodyActionClient>());
  rclcpp::shutdown();
  return 0;
}
