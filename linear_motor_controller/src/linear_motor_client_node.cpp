#include "rclcpp/rclcpp.hpp"
#include "linear_motor_msgs/srv/act.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("linear_motor_client_node");

  auto client = node->create_client<linear_motor_msgs::srv::Act>("/action_command");

  // サーバーが起動するまで待つ
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "サービスが見つかりません");
    return 1;
  }

  auto request = std::make_shared<linear_motor_msgs::srv::Act::Request>();

  RCLCPP_INFO(node->get_logger(), "Request action command: [up]");
  request->action = "up";
  auto up_future = client->async_send_request(request);

  rclcpp::sleep_for(std::chrono::milliseconds(5000));

  RCLCPP_INFO(node->get_logger(), "Request action command: [down]");
  request->action = "down";
  auto down_future = client->async_send_request(request);

  // 応答を待機
  if (rclcpp::spin_until_future_complete(node, up_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = up_future.get();
    RCLCPP_INFO(node->get_logger(), "Response: success = %s", response->result.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "サービス呼び出しに失敗しました");
  }

  rclcpp::shutdown();
  return 0;
}