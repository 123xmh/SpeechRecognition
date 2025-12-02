// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from chest_interfaces:msg/VoiceCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "chest_interfaces/msg/detail/voice_command__rosidl_typesupport_introspection_c.h"
#include "chest_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "chest_interfaces/msg/detail/voice_command__functions.h"
#include "chest_interfaces/msg/detail/voice_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  chest_interfaces__msg__VoiceCommand__init(message_memory);
}

void VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_fini_function(void * message_memory)
{
  chest_interfaces__msg__VoiceCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, frame_header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, command_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, param1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, param2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, param3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chest_interfaces__msg__VoiceCommand, is_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_members = {
  "chest_interfaces__msg",  // message namespace
  "VoiceCommand",  // message name
  7,  // number of fields
  sizeof(chest_interfaces__msg__VoiceCommand),
  VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_member_array,  // message members
  VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_type_support_handle = {
  0,
  &VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chest_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chest_interfaces, msg, VoiceCommand)() {
  VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_type_support_handle.typesupport_identifier) {
    VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VoiceCommand__rosidl_typesupport_introspection_c__VoiceCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
