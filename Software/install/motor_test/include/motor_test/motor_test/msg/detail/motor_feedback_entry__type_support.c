// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_test/msg/detail/motor_feedback_entry__rosidl_typesupport_introspection_c.h"
#include "motor_test/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_test/msg/detail/motor_feedback_entry__functions.h"
#include "motor_test/msg/detail/motor_feedback_entry__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_test__msg__MotorFeedbackEntry__init(message_memory);
}

void motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_fini_function(void * message_memory)
{
  motor_test__msg__MotorFeedbackEntry__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_member_array[3] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test__msg__MotorFeedbackEntry, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "q",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test__msg__MotorFeedbackEntry, q),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "q_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test__msg__MotorFeedbackEntry, q_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_members = {
  "motor_test__msg",  // message namespace
  "MotorFeedbackEntry",  // message name
  3,  // number of fields
  sizeof(motor_test__msg__MotorFeedbackEntry),
  false,  // has_any_key_member_
  motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_member_array,  // message members
  motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_type_support_handle = {
  0,
  &motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_members,
  get_message_typesupport_handle_function,
  &motor_test__msg__MotorFeedbackEntry__get_type_hash,
  &motor_test__msg__MotorFeedbackEntry__get_type_description,
  &motor_test__msg__MotorFeedbackEntry__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_test
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_test, msg, MotorFeedbackEntry)() {
  if (!motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_type_support_handle.typesupport_identifier) {
    motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_test__msg__MotorFeedbackEntry__rosidl_typesupport_introspection_c__MotorFeedbackEntry_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
