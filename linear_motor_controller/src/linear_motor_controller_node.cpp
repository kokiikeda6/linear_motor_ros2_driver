#include "rclcpp/rclcpp.hpp"
#include "linear_motor_msgs/srv/act.hpp"
#include <iostream>
#include <gpiod.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

const std::string CHIP_NAME = "gpiochip4"; //Raspi5の場合
const int IN1_PIN = 14; // GPIO 14
const int IN2_PIN = 15; // GPIO 15

class LinearMotorControllerNode : public rclcpp::Node
{
  public:
  LinearMotorControllerNode() : Node("linear_motor_controller_node")
  {
    this->declare_parameter<std::float_t>("extend_distance", 4.0);  // [mm]
    this->declare_parameter<std::float_t>("extend_speed", 4.0); // [mm/s]

    this->get_parameter("extend_distance", extend_distance);
    this->get_parameter("extend_distance", extend_speed);

    act_srv = this->create_service<linear_motor_msgs::srv::Act>(
      "action_command",
      std::bind(&LinearMotorControllerNode::act_srv_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    try {
            // 1. チップを開く
            chip_ = std::make_unique<gpiod::chip>(CHIP_NAME);

            // 2. ピン（ライン）を取得
            line_in1_ = chip_->get_line(IN1_PIN);
            line_in2_ = chip_->get_line(IN2_PIN);

            // 3. ピンを出力モードで要求 (request) する
            line_in1_.request({"motor-controller", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0); // 初期値 LOW
            line_in2_.request({"motor-controller", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0); // 初期値 LOW

            RCLCPP_INFO(this->get_logger(), "GPIO lines successfully acquired and set to output.");
            
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed: %s", e.what());
          throw;
        }
  }

  private:
  std::unique_ptr<gpiod::chip> chip_; // GPIOチップオブジェクト
  gpiod::line line_in1_;             // IN1ピンのラインオブジェクト
  gpiod::line line_in2_;             // IN2ピンのラインオブジェクト
  rclcpp::Service<linear_motor_msgs::srv::Act>::SharedPtr act_srv;

  float_t extend_distance;
  float_t extend_speed;
  float_t operating_time = extend_distance / extend_speed;

  void act_srv_callback(
    const std::shared_ptr<linear_motor_msgs::srv::Act::Request> request,
    std::shared_ptr<linear_motor_msgs::srv::Act::Response> response)
  {
    if(request->action=="up"){
      shrink_motor();
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(operating_time).to_chrono<std::chrono::milliseconds>());
      stop_motor();
    }

    if(request->action=="down"){
      extend_motor();
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(operating_time).to_chrono<std::chrono::milliseconds>());
      stop_motor();
    }

    if(request->action=="up_1mm"){
      shrink_motor();
      rclcpp::sleep_for(250ms);
      stop_motor();
    }

    if(request->action=="down_1mm"){
      extend_motor();
      rclcpp::sleep_for(250ms);
      stop_motor();
    }

    response->result = "success";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\naction: [%s]", request->action.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->result.c_str());
  }
  
  void extend_motor(){
    line_in1_.set_value(1);
    line_in2_.set_value(0);
  }

  void shrink_motor(){
    line_in1_.set_value(0);
    line_in2_.set_value(1);
  }

  void brake_motor(){
    line_in1_.set_value(1);
    line_in2_.set_value(1);
  }

  void stop_motor(){
    line_in1_.set_value(0);
    line_in2_.set_value(0);
  }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearMotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}
