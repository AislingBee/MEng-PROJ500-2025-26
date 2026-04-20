// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_test/msg/detail/motor_feedback__functions.h"
#include "motor_test/msg/detail/motor_feedback__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace motor_test
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorFeedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) motor_test::msg::MotorFeedback(_init);
}

void MotorFeedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<motor_test::msg::MotorFeedback *>(message_memory);
  typed_message->~MotorFeedback();
}

size_t size_function__MotorFeedback__motors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<motor_test::msg::MotorFeedbackEntry> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorFeedback__motors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<motor_test::msg::MotorFeedbackEntry> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorFeedback__motors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<motor_test::msg::MotorFeedbackEntry> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorFeedback__motors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const motor_test::msg::MotorFeedbackEntry *>(
    get_const_function__MotorFeedback__motors(untyped_member, index));
  auto & value = *reinterpret_cast<motor_test::msg::MotorFeedbackEntry *>(untyped_value);
  value = item;
}

void assign_function__MotorFeedback__motors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<motor_test::msg::MotorFeedbackEntry *>(
    get_function__MotorFeedback__motors(untyped_member, index));
  const auto & value = *reinterpret_cast<const motor_test::msg::MotorFeedbackEntry *>(untyped_value);
  item = value;
}

void resize_function__MotorFeedback__motors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<motor_test::msg::MotorFeedbackEntry> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorFeedback_message_member_array[1] = {
  {
    "motors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<motor_test::msg::MotorFeedbackEntry>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test::msg::MotorFeedback, motors),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorFeedback__motors,  // size() function pointer
    get_const_function__MotorFeedback__motors,  // get_const(index) function pointer
    get_function__MotorFeedback__motors,  // get(index) function pointer
    fetch_function__MotorFeedback__motors,  // fetch(index, &value) function pointer
    assign_function__MotorFeedback__motors,  // assign(index, value) function pointer
    resize_function__MotorFeedback__motors  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorFeedback_message_members = {
  "motor_test::msg",  // message namespace
  "MotorFeedback",  // message name
  1,  // number of fields
  sizeof(motor_test::msg::MotorFeedback),
  false,  // has_any_key_member_
  MotorFeedback_message_member_array,  // message members
  MotorFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorFeedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorFeedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorFeedback_message_members,
  get_message_typesupport_handle_function,
  &motor_test__msg__MotorFeedback__get_type_hash,
  &motor_test__msg__MotorFeedback__get_type_description,
  &motor_test__msg__MotorFeedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace motor_test


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<motor_test::msg::MotorFeedback>()
{
  return &::motor_test::msg::rosidl_typesupport_introspection_cpp::MotorFeedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, motor_test, msg, MotorFeedback)() {
  return &::motor_test::msg::rosidl_typesupport_introspection_cpp::MotorFeedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
