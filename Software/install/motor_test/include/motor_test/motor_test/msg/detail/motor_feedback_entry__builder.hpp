// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback_entry.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__BUILDER_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_test/msg/detail/motor_feedback_entry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_test
{

namespace msg
{

namespace builder
{

class Init_MotorFeedbackEntry_q_dot
{
public:
  explicit Init_MotorFeedbackEntry_q_dot(::motor_test::msg::MotorFeedbackEntry & msg)
  : msg_(msg)
  {}
  ::motor_test::msg::MotorFeedbackEntry q_dot(::motor_test::msg::MotorFeedbackEntry::_q_dot_type arg)
  {
    msg_.q_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_test::msg::MotorFeedbackEntry msg_;
};

class Init_MotorFeedbackEntry_q
{
public:
  explicit Init_MotorFeedbackEntry_q(::motor_test::msg::MotorFeedbackEntry & msg)
  : msg_(msg)
  {}
  Init_MotorFeedbackEntry_q_dot q(::motor_test::msg::MotorFeedbackEntry::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorFeedbackEntry_q_dot(msg_);
  }

private:
  ::motor_test::msg::MotorFeedbackEntry msg_;
};

class Init_MotorFeedbackEntry_name
{
public:
  Init_MotorFeedbackEntry_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorFeedbackEntry_q name(::motor_test::msg::MotorFeedbackEntry::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MotorFeedbackEntry_q(msg_);
  }

private:
  ::motor_test::msg::MotorFeedbackEntry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_test::msg::MotorFeedbackEntry>()
{
  return motor_test::msg::builder::Init_MotorFeedbackEntry_name();
}

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__BUILDER_HPP_
