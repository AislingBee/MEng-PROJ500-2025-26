// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

#include "motor_test/msg/detail/motor_feedback__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_test
const rosidl_type_hash_t *
motor_test__msg__MotorFeedback__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0xfb, 0x97, 0x98, 0xb3, 0x04, 0x18, 0xc6,
      0x23, 0x46, 0x53, 0xd0, 0x06, 0x92, 0xbb, 0x31,
      0x19, 0x3a, 0x9a, 0xa0, 0x71, 0x2f, 0xe4, 0x11,
      0xb1, 0xf1, 0x8e, 0xc5, 0x4a, 0x6c, 0xe7, 0x6d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "motor_test/msg/detail/motor_feedback_entry__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t motor_test__msg__MotorFeedbackEntry__EXPECTED_HASH = {1, {
    0x34, 0xba, 0x88, 0x2e, 0x49, 0xb1, 0x69, 0x30,
    0x48, 0xca, 0xf4, 0xfc, 0xa9, 0xd0, 0xb8, 0x71,
    0xbc, 0xb6, 0x4c, 0x74, 0x60, 0x0f, 0x01, 0x6c,
    0x3d, 0x8d, 0x67, 0xc0, 0xb5, 0x36, 0x11, 0x06,
  }};
#endif

static char motor_test__msg__MotorFeedback__TYPE_NAME[] = "motor_test/msg/MotorFeedback";
static char motor_test__msg__MotorFeedbackEntry__TYPE_NAME[] = "motor_test/msg/MotorFeedbackEntry";

// Define type names, field names, and default values
static char motor_test__msg__MotorFeedback__FIELD_NAME__motors[] = "motors";

static rosidl_runtime_c__type_description__Field motor_test__msg__MotorFeedback__FIELDS[] = {
  {
    {motor_test__msg__MotorFeedback__FIELD_NAME__motors, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {motor_test__msg__MotorFeedbackEntry__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription motor_test__msg__MotorFeedback__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {motor_test__msg__MotorFeedbackEntry__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
motor_test__msg__MotorFeedback__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_test__msg__MotorFeedback__TYPE_NAME, 28, 28},
      {motor_test__msg__MotorFeedback__FIELDS, 1, 1},
    },
    {motor_test__msg__MotorFeedback__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&motor_test__msg__MotorFeedbackEntry__EXPECTED_HASH, motor_test__msg__MotorFeedbackEntry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = motor_test__msg__MotorFeedbackEntry__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "motor_test/MotorFeedbackEntry[] motors";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_test__msg__MotorFeedback__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_test__msg__MotorFeedback__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 39, 39},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_test__msg__MotorFeedback__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_test__msg__MotorFeedback__get_individual_type_description_source(NULL),
    sources[1] = *motor_test__msg__MotorFeedbackEntry__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
