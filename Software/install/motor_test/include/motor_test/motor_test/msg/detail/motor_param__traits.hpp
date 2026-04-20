// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_param.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__TRAITS_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_test/msg/detail/motor_param__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorParam & msg,
  std::ostream & out)
{
  out << "{";
  // member: q
  {
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << ", ";
  }

  // member: tau
  {
    out << "tau: ";
    rosidl_generator_traits::value_to_yaml(msg.tau, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }

  // member: tau
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tau: ";
    rosidl_generator_traits::value_to_yaml(msg.tau, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorParam & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace motor_test

namespace rosidl_generator_traits
{

[[deprecated("use motor_test::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_test::msg::MotorParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_test::msg::MotorParam & msg)
{
  return motor_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_test::msg::MotorParam>()
{
  return "motor_test::msg::MotorParam";
}

template<>
inline const char * name<motor_test::msg::MotorParam>()
{
  return "motor_test/msg/MotorParam";
}

template<>
struct has_fixed_size<motor_test::msg::MotorParam>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motor_test::msg::MotorParam>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motor_test::msg::MotorParam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__TRAITS_HPP_
