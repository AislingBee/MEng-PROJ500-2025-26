// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_param.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__BUILDER_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_test/msg/detail/motor_param__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_test
{

namespace msg
{

namespace builder
{

class Init_MotorParam_tau
{
public:
  explicit Init_MotorParam_tau(::motor_test::msg::MotorParam & msg)
  : msg_(msg)
  {}
  ::motor_test::msg::MotorParam tau(::motor_test::msg::MotorParam::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_test::msg::MotorParam msg_;
};

class Init_MotorParam_kd
{
public:
  explicit Init_MotorParam_kd(::motor_test::msg::MotorParam & msg)
  : msg_(msg)
  {}
  Init_MotorParam_tau kd(::motor_test::msg::MotorParam::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MotorParam_tau(msg_);
  }

private:
  ::motor_test::msg::MotorParam msg_;
};

class Init_MotorParam_kp
{
public:
  explicit Init_MotorParam_kp(::motor_test::msg::MotorParam & msg)
  : msg_(msg)
  {}
  Init_MotorParam_kd kp(::motor_test::msg::MotorParam::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorParam_kd(msg_);
  }

private:
  ::motor_test::msg::MotorParam msg_;
};

class Init_MotorParam_q
{
public:
  Init_MotorParam_q()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorParam_kp q(::motor_test::msg::MotorParam::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorParam_kp(msg_);
  }

private:
  ::motor_test::msg::MotorParam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_test::msg::MotorParam>()
{
  return motor_test::msg::builder::Init_MotorParam_q();
}

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__BUILDER_HPP_
