#include "rclcpp/rclcpp.hpp"
#include "linear_motor_msgs/srv/act.hpp"

class LinearMotorControllerNode : public rclcpp::Node
{
  public:
  LinearMotorControllerNode() : Node("linear_motor_controller_node")
  {
    this->declare_parameter<int>("IN1_PIN", 14); //GPIO14
    this->declare_parameter<int>("IN2_PIN", 15); //GPIO15

    this->get_parameter("IN1_PIN", in1_pin);
    this->get_parameter("IN2_PIN", in2_pin);

    act_srv = this->create_service<linear_motor_msgs::srv::Act>(
      "action_command",
      std::bind(&LinearMotorControllerNode::act_srv_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  private:
  rclcpp::Service<linear_motor_msgs::srv::Act>::SharedPtr act_srv;

  int in1_pin, in2_pin;
  
  void act_srv_callback(
    const std::shared_ptr<linear_motor_msgs::srv::Act::Request> request,
    std::shared_ptr<linear_motor_msgs::srv::Act::Response> response
  ) const
  {
    response->result = "success";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\naction: [%s]", request->action.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->result.c_str());
  }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearMotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}