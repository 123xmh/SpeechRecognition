// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_HPP_

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
# define DEPRECATED__chest_interfaces__msg__VoiceText __attribute__((deprecated))
#else
# define DEPRECATED__chest_interfaces__msg__VoiceText __declspec(deprecated)
#endif

namespace chest_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VoiceText_
{
  using Type = VoiceText_<ContainerAllocator>;

  explicit VoiceText_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_text = "";
      this->is_final = false;
      this->confidence = 0.0f;
    }
  }

  explicit VoiceText_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    raw_text(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_text = "";
      this->is_final = false;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _raw_text_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _raw_text_type raw_text;
  using _is_final_type =
    bool;
  _is_final_type is_final;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__raw_text(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->raw_text = _arg;
    return *this;
  }
  Type & set__is_final(
    const bool & _arg)
  {
    this->is_final = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chest_interfaces::msg::VoiceText_<ContainerAllocator> *;
  using ConstRawPtr =
    const chest_interfaces::msg::VoiceText_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VoiceText_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VoiceText_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chest_interfaces__msg__VoiceText
    std::shared_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chest_interfaces__msg__VoiceText
    std::shared_ptr<chest_interfaces::msg::VoiceText_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VoiceText_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->raw_text != other.raw_text) {
      return false;
    }
    if (this->is_final != other.is_final) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const VoiceText_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VoiceText_

// alias to use template instance with default allocator
using VoiceText =
  chest_interfaces::msg::VoiceText_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__STRUCT_HPP_
