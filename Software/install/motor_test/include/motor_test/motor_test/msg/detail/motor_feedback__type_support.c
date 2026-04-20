// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_test/msg/detail/motor_feedback__rosidl_typesupport_introspection_c.h"
#include "motor_test/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_test/msg/detail/motor_feedback__functions.h"
#include "motor_test/msg/detail/motor_feedback__struct.h"


// Include directives for member types
// Member `motors`
#include "motor_test/msg/motor_feedback_entry.h"
// Member `motors`
#include "motor_test/msg/detail/motor_feedback_entry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_test__msg__MotorFeedback__init(message_memory);
}

void motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_fini_function(void * message_memory)
{
  motor_test__msg__MotorFeedback__fini(message_memory);
}

size_t motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__size_function__MotorFeedback__motors(
  const void * untyped_member)
{
  const motor_test__msg__MotorFeedbackEntry__Sequence * member =
    (const motor_test__msg__MotorFeedbackEntry__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__motors(
  const void * untyped_member, size_t index)
{
  const motor_test__msg__MotorFeedbackEntry__Sequence * member =
    (const motor_test__msg__MotorFeedbackEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__motors(
  void * untyped_member, size_t index)
{
  motor_test__msg__MotorFeedbackEntry__Sequence * member =
    (motor_test__msg__MotorFeedbackEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__fetch_function__MotorFeedback__motors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const motor_test__msg__MotorFeedbackEntry * item =
    ((const motor_test__msg__MotorFeedbackEntry *)
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__motors(untyped_member, index));
  motor_test__msg__MotorFeedbackEntry * value =
    (motor_test__msg__MotorFeedbackEntry *)(untyped_value);
  *value = *item;
}

void motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__assign_function__MotorFeedback__motors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  motor_test__msg__MotorFeedbackEntry * item =
    ((motor_test__msg__MotorFeedbackEntry *)
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__motors(untyped_member, index));
  const motor_test__msg__MotorFeedbackEntry * value =
    (const motor_test__msg__MotorFeedbackEntry *)(untyped_value);
  *item = *value;
}

bool motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__resize_function__MotorFeedback__motors(
  void * untyped_member, size_t size)
{
  motor_test__msg__MotorFeedbackEntry__Sequence * member =
    (motor_test__msg__MotorFeedbackEntry__Sequence *)(untyped_member);
  motor_test__msg__MotorFeedbackEntry__Sequence__fini(member);
  return motor_test__msg__MotorFeedbackEntry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array[1] = {
  {
    "motors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_test__msg__MotorFeedback, motors),  // bytes offset in struct
    NULL,  // default value
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__size_function__MotorFeedback__motors,  // size() function pointer
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_const_function__MotorFeedback__motors,  // get_const(index) function pointer
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__get_function__MotorFeedback__motors,  // get(index) function pointer
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__fetch_function__MotorFeedback__motors,  // fetch(index, &value) function pointer
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__assign_function__MotorFeedback__motors,  // assign(index, value) function pointer
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__resize_function__MotorFeedback__motors  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_members = {
  "motor_test__msg",  // message namespace
  "MotorFeedback",  // message name
  1,  // number of fields
  sizeof(motor_test__msg__MotorFeedback),
  false,  // has_any_key_member_
  motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array,  // message members
  motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle = {
  0,
  &motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_members,
  get_message_typesupport_handle_function,
  &motor_test__msg__MotorFeedback__get_type_hash,
  &motor_test__msg__MotorFeedback__get_type_description,
  &motor_test__msg__MotorFeedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_test
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_test, msg, MotorFeedback)() {
  motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_test, msg, MotorFeedbackEntry)();
  if (!motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle.typesupport_identifier) {
    motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_test__msg__MotorFeedback__rosidl_typesupport_introspection_c__MotorFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
