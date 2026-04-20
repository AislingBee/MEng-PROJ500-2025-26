// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice

#include "motor_test/msg/detail/motor_param__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_test
const rosidl_type_hash_t *
motor_test__msg__MotorParam__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x74, 0xb4, 0x24, 0x44, 0xf8, 0x0f, 0xa7, 0x60,
      0x7d, 0xb2, 0xfa, 0x05, 0xe3, 0xda, 0xf8, 0x3c,
      0xad, 0xa2, 0xcf, 0x31, 0x21, 0xc5, 0x3e, 0x76,
      0x04, 0xb3, 0xc7, 0x1f, 0xdf, 0xc3, 0xbd, 0xcd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char motor_test__msg__MotorParam__TYPE_NAME[] = "motor_test/msg/MotorParam";

// Define type names, field names, and default values
static char motor_test__msg__MotorParam__FIELD_NAME__q[] = "q";
static char motor_test__msg__MotorParam__FIELD_NAME__kp[] = "kp";
static char motor_test__msg__MotorParam__FIELD_NAME__kd[] = "kd";
static char motor_test__msg__MotorParam__FIELD_NAME__tau[] = "tau";

static rosidl_runtime_c__type_description__Field motor_test__msg__MotorParam__FIELDS[] = {
  {
    {motor_test__msg__MotorParam__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_test__msg__MotorParam__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_test__msg__MotorParam__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_test__msg__MotorParam__FIELD_NAME__tau, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
motor_test__msg__MotorParam__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_test__msg__MotorParam__TYPE_NAME, 25, 25},
      {motor_test__msg__MotorParam__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 q\n"
  "float64 kp\n"
  "float64 kd\n"
  "float64 tau";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_test__msg__MotorParam__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_test__msg__MotorParam__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 44, 44},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_test__msg__MotorParam__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_test__msg__MotorParam__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
