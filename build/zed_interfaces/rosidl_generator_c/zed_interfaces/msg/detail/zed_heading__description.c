// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zed_interfaces:msg/ZedHeading.idl
// generated code does not contain a copyright notice

#include "zed_interfaces/msg/detail/zed_heading__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zed_interfaces
const rosidl_type_hash_t *
zed_interfaces__msg__ZedHeading__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xb1, 0x84, 0x35, 0xc4, 0xc3, 0x28, 0xbd,
      0x36, 0x4d, 0x03, 0x79, 0xd1, 0x7d, 0xe9, 0xc9,
      0xfd, 0xe3, 0x9a, 0xed, 0xdb, 0x58, 0x6b, 0xd0,
      0xd8, 0x76, 0x4a, 0xc4, 0x1f, 0x91, 0xe2, 0x16,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char zed_interfaces__msg__ZedHeading__TYPE_NAME[] = "zed_interfaces/msg/ZedHeading";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__header[] = "header";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__raw_x[] = "raw_x";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__raw_z[] = "raw_z";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__corrected_x[] = "corrected_x";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__corrected_z[] = "corrected_z";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__magnetic_heading_deg[] = "magnetic_heading_deg";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__robot_yaw_deg[] = "robot_yaw_deg";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__robot_yaw_rad[] = "robot_yaw_rad";
static char zed_interfaces__msg__ZedHeading__FIELD_NAME__valid[] = "valid";

static rosidl_runtime_c__type_description__Field zed_interfaces__msg__ZedHeading__FIELDS[] = {
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__raw_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__raw_z, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__corrected_x, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__corrected_z, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__magnetic_heading_deg, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__robot_yaw_deg, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__robot_yaw_rad, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zed_interfaces__msg__ZedHeading__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription zed_interfaces__msg__ZedHeading__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zed_interfaces__msg__ZedHeading__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zed_interfaces__msg__ZedHeading__TYPE_NAME, 29, 29},
      {zed_interfaces__msg__ZedHeading__FIELDS, 9, 9},
    },
    {zed_interfaces__msg__ZedHeading__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "float32 raw_x\n"
  "float32 raw_z\n"
  "\n"
  "float32 corrected_x\n"
  "float32 corrected_z\n"
  "\n"
  "float32 magnetic_heading_deg\n"
  "float32 robot_yaw_deg\n"
  "float32 robot_yaw_rad\n"
  "\n"
  "bool valid";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zed_interfaces__msg__ZedHeading__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zed_interfaces__msg__ZedHeading__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 179, 179},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zed_interfaces__msg__ZedHeading__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zed_interfaces__msg__ZedHeading__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
