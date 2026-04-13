#include "motor_test/motor_sub.hpp"
#include "motor_test/msg/motor_param.hpp"
// TODO: Swap this to python prbably?

MotorSub::MotorSub()
: Node("motor_sub")
{
  this->declare_parameter<std::string>("input", "motor_params");
  std::string input = this->get_parameter("input").as_string();

  subscription_ = this->create_subscription<motor_test::msg::MotorParam>(
    input,
    10,
    [this](motor_test::msg::MotorParam::UniquePtr msg) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received motor params: q=%.3f kp=%.3f kd=%.3f tau=%.3f",
        msg->q, msg->kp, msg->kd, msg->tau);
    });

  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", input.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSub>());
  rclcpp::shutdown();
  return 0;
}
