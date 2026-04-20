#ifndef MOTOR_TEST_MOTOR_SUB_HPP_
#define MOTOR_TEST_MOTOR_SUB_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "motor_test/msg/motor_param.hpp"

class MotorSub : public rclcpp::Node
{
public:
  MotorSub();

private:
  rclcpp::Subscription<motor_test::msg::MotorParam>::SharedPtr subscription_;
};

#endif  // MOTOR_TEST_MOTOR_SUB_HPP_
