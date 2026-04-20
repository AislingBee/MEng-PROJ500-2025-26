// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback_entry.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__TRAITS_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_test/msg/detail/motor_feedback_entry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorFeedbackEntry & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: q
  {
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << ", ";
  }

  // member: q_dot
  {
    out << "q_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q_dot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorFeedbackEntry & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << "\n";
  }

  // member: q_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q_dot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorFeedbackEntry & msg, bool use_flow_style = false)
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
  const motor_test::msg::MotorFeedbackEntry & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_test::msg::MotorFeedbackEntry & msg)
{
  return motor_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_test::msg::MotorFeedbackEntry>()
{
  return "motor_test::msg::MotorFeedbackEntry";
}

template<>
inline const char * name<motor_test::msg::MotorFeedbackEntry>()
{
  return "motor_test/msg/MotorFeedbackEntry";
}

template<>
struct has_fixed_size<motor_test::msg::MotorFeedbackEntry>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_test::msg::MotorFeedbackEntry>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_test::msg::MotorFeedbackEntry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__TRAITS_HPP_
