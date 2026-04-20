// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

#include "motor_test/msg/detail/motor_feedback_entry__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_test
const rosidl_type_hash_t *
motor_test__msg__MotorFeedbackEntry__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x34, 0xba, 0x88, 0x2e, 0x49, 0xb1, 0x69, 0x30,
      0x48, 0xca, 0xf4, 0xfc, 0xa9, 0xd0, 0xb8, 0x71,
      0xbc, 0xb6, 0x4c, 0x74, 0x60, 0x0f, 0x01, 0x6c,
      0x3d, 0x8d, 0x67, 0xc0, 0xb5, 0x36, 0x11, 0x06,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char motor_test__msg__MotorFeedbackEntry__TYPE_NAME[] = "motor_test/msg/MotorFeedbackEntry";

// Define type names, field names, and default values
static char motor_test__msg__MotorFeedbackEntry__FIELD_NAME__name[] = "name";
static char motor_test__msg__MotorFeedbackEntry__FIELD_NAME__q[] = "q";
static char motor_test__msg__MotorFeedbackEntry__FIELD_NAME__q_dot[] = "q_dot";

static rosidl_runtime_c__type_description__Field motor_test__msg__MotorFeedbackEntry__FIELDS[] = {
  {
    {motor_test__msg__MotorFeedbackEntry__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_test__msg__MotorFeedbackEntry__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_test__msg__MotorFeedbackEntry__FIELD_NAME__q_dot, 5, 5},
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
motor_test__msg__MotorFeedbackEntry__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_test__msg__MotorFeedbackEntry__TYPE_NAME, 33, 33},
      {motor_test__msg__MotorFeedbackEntry__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string name\n"
  "float64 q\n"
  "float64 q_dot";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_test__msg__MotorFeedbackEntry__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_test__msg__MotorFeedbackEntry__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 36, 36},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_test__msg__MotorFeedbackEntry__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_test__msg__MotorFeedbackEntry__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
