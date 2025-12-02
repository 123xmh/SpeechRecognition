// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chest_interfaces:msg/VoiceCommand.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chest_interfaces__msg__VoiceCommand __attribute__((deprecated))
#else
# define DEPRECATED__chest_interfaces__msg__VoiceCommand __declspec(deprecated)
#endif

namespace chest_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VoiceCommand_
{
  using Type = VoiceCommand_<ContainerAllocator>;

  explicit VoiceCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_header = 0;
      this->command_id = 0ul;
      this->param1 = 0l;
      this->param2 = 0l;
      this->param3 = 0;
      this->is_valid = false;
    }
  }

  explicit VoiceCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_header = 0;
      this->command_id = 0ul;
      this->param1 = 0l;
      this->param2 = 0l;
      this->param3 = 0;
      this->is_valid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _frame_header_type =
    uint16_t;
  _frame_header_type frame_header;
  using _command_id_type =
    uint32_t;
  _command_id_type command_id;
  using _param1_type =
    int32_t;
  _param1_type param1;
  using _param2_type =
    int32_t;
  _param2_type param2;
  using _param3_type =
    uint8_t;
  _param3_type param3;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__frame_header(
    const uint16_t & _arg)
  {
    this->frame_header = _arg;
    return *this;
  }
  Type & set__command_id(
    const uint32_t & _arg)
  {
    this->command_id = _arg;
    return *this;
  }
  Type & set__param1(
    const int32_t & _arg)
  {
    this->param1 = _arg;
    return *this;
  }
  Type & set__param2(
    const int32_t & _arg)
  {
    this->param2 = _arg;
    return *this;
  }
  Type & set__param3(
    const uint8_t & _arg)
  {
    this->param3 = _arg;
    return *this;
  }
  Type & set__is_valid(
    const bool & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chest_interfaces::msg::VoiceCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const chest_interfaces::msg::VoiceCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VoiceCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VoiceCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chest_interfaces__msg__VoiceCommand
    std::shared_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chest_interfaces__msg__VoiceCommand
    std::shared_ptr<chest_interfaces::msg::VoiceCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VoiceCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->frame_header != other.frame_header) {
      return false;
    }
    if (this->command_id != other.command_id) {
      return false;
    }
    if (this->param1 != other.param1) {
      return false;
    }
    if (this->param2 != other.param2) {
      return false;
    }
    if (this->param3 != other.param3) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const VoiceCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VoiceCommand_

// alias to use template instance with default allocator
using VoiceCommand =
  chest_interfaces::msg::VoiceCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_COMMAND__STRUCT_HPP_
