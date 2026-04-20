// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_test/msg/detail/motor_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_test
{

namespace msg
{

namespace builder
{

class Init_MotorFeedback_motors
{
public:
  Init_MotorFeedback_motors()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::motor_test::msg::MotorFeedback motors(::motor_test::msg::MotorFeedback::_motors_type arg)
  {
    msg_.motors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_test::msg::MotorFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_test::msg::MotorFeedback>()
{
  return motor_test::msg::builder::Init_MotorFeedback_motors();
}

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__BUILDER_HPP_
