// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_test/msg/detail/motor_feedback_entry__functions.h"
#include "motor_test/msg/detail/motor_feedback_entry__struct.hpp"
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

void MotorFeedbackEntry_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) motor_test::msg::MotorFeedbackEntry(_init);
}

void MotorFeedbackEntry_fini_function(void * message_memory)
{
  auto typed_message = static_cast<motor_test::msg::MotorFeedbackEntry *>(message_memory);
  typed_message->~MotorFeedbackEntry();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorFeedbackEntry_message_member_array[3] = {
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test::msg::MotorFeedbackEntry, name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "q",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test::msg::MotorFeedbackEntry, q),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "q_dot",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test::msg::MotorFeedbackEntry, q_dot),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorFeedbackEntry_message_members = {
  "motor_test::msg",  // message namespace
  "MotorFeedbackEntry",  // message name
  3,  // number of fields
  sizeof(motor_test::msg::MotorFeedbackEntry),
  false,  // has_any_key_member_
  MotorFeedbackEntry_message_member_array,  // message members
  MotorFeedbackEntry_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorFeedbackEntry_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorFeedbackEntry_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorFeedbackEntry_message_members,
  get_message_typesupport_handle_function,
  &motor_test__msg__MotorFeedbackEntry__get_type_hash,
  &motor_test__msg__MotorFeedbackEntry__get_type_description,
  &motor_test__msg__MotorFeedbackEntry__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace motor_test


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<motor_test::msg::MotorFeedbackEntry>()
{
  return &::motor_test::msg::rosidl_typesupport_introspection_cpp::MotorFeedbackEntry_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, motor_test, msg, MotorFeedbackEntry)() {
  return &::motor_test::msg::rosidl_typesupport_introspection_cpp::MotorFeedbackEntry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
