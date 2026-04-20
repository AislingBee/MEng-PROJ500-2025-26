// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__TRAITS_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_test/msg/detail/motor_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'motors'
#include "motor_test/msg/detail/motor_feedback_entry__traits.hpp"

namespace motor_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: motors
  {
    if (msg.motors.size() == 0) {
      out << "motors: []";
    } else {
      out << "motors: [";
      size_t pending_items = msg.motors.size();
      for (auto item : msg.motors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motors.size() == 0) {
      out << "motors: []\n";
    } else {
      out << "motors:\n";
      for (auto item : msg.motors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorFeedback & msg, bool use_flow_style = false)
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
  const motor_test::msg::MotorFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_test::msg::MotorFeedback & msg)
{
  return motor_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_test::msg::MotorFeedback>()
{
  return "motor_test::msg::MotorFeedback";
}

template<>
inline const char * name<motor_test::msg::MotorFeedback>()
{
  return "motor_test/msg/MotorFeedback";
}

template<>
struct has_fixed_size<motor_test::msg::MotorFeedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_test::msg::MotorFeedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_test::msg::MotorFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__TRAITS_HPP_
