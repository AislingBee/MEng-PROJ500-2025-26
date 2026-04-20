// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice
#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_test/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "motor_test/msg/detail/motor_param__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
bool cdr_serialize_motor_test__msg__MotorParam(
  const motor_test__msg__MotorParam * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
bool cdr_deserialize_motor_test__msg__MotorParam(
  eprosima::fastcdr::Cdr &,
  motor_test__msg__MotorParam * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
size_t get_serialized_size_motor_test__msg__MotorParam(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
size_t max_serialized_size_motor_test__msg__MotorParam(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
bool cdr_serialize_key_motor_test__msg__MotorParam(
  const motor_test__msg__MotorParam * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
size_t get_serialized_size_key_motor_test__msg__MotorParam(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
size_t max_serialized_size_key_motor_test__msg__MotorParam(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_test
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, motor_test, msg, MotorParam)();

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
