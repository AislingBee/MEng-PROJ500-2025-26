// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback_entry.h"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_H_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorFeedbackEntry in the package motor_test.
typedef struct motor_test__msg__MotorFeedbackEntry
{
  rosidl_runtime_c__String name;
  double q;
  double q_dot;
} motor_test__msg__MotorFeedbackEntry;

// Struct for a sequence of motor_test__msg__MotorFeedbackEntry.
typedef struct motor_test__msg__MotorFeedbackEntry__Sequence
{
  motor_test__msg__MotorFeedbackEntry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_test__msg__MotorFeedbackEntry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_H_
