// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_param.h"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_H_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorParam in the package motor_test.
typedef struct motor_test__msg__MotorParam
{
  double q;
  double kp;
  double kd;
  double tau;
} motor_test__msg__MotorParam;

// Struct for a sequence of motor_test__msg__MotorParam.
typedef struct motor_test__msg__MotorParam__Sequence
{
  motor_test__msg__MotorParam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_test__msg__MotorParam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_H_
