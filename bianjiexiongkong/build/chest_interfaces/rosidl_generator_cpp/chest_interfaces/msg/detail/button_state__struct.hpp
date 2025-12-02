// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_HPP_

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
# define DEPRECATED__chest_interfaces__msg__ButtonState __attribute__((deprecated))
#else
# define DEPRECATED__chest_interfaces__msg__ButtonState __declspec(deprecated)
#endif

namespace chest_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ButtonState_
{
  using Type = ButtonState_<ContainerAllocator>;

  explicit ButtonState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->head_tracking_enabled = false;
      this->voice_command_enabled = false;
      this->front_screen_enabled = false;
      this->front_screen_mode = 0;
      this->hmd_video_source = 0;
      this->hmd_overlay_mode = 0;
      this->hmd_enabled = false;
    }
  }

  explicit ButtonState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->head_tracking_enabled = false;
      this->voice_command_enabled = false;
      this->front_screen_enabled = false;
      this->front_screen_mode = 0;
      this->hmd_video_source = 0;
      this->hmd_overlay_mode = 0;
      this->hmd_enabled = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _head_tracking_enabled_type =
    bool;
  _head_tracking_enabled_type head_tracking_enabled;
  using _voice_command_enabled_type =
    bool;
  _voice_command_enabled_type voice_command_enabled;
  using _front_screen_enabled_type =
    bool;
  _front_screen_enabled_type front_screen_enabled;
  using _front_screen_mode_type =
    uint8_t;
  _front_screen_mode_type front_screen_mode;
  using _hmd_video_source_type =
    uint8_t;
  _hmd_video_source_type hmd_video_source;
  using _hmd_overlay_mode_type =
    uint8_t;
  _hmd_overlay_mode_type hmd_overlay_mode;
  using _hmd_enabled_type =
    bool;
  _hmd_enabled_type hmd_enabled;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__head_tracking_enabled(
    const bool & _arg)
  {
    this->head_tracking_enabled = _arg;
    return *this;
  }
  Type & set__voice_command_enabled(
    const bool & _arg)
  {
    this->voice_command_enabled = _arg;
    return *this;
  }
  Type & set__front_screen_enabled(
    const bool & _arg)
  {
    this->front_screen_enabled = _arg;
    return *this;
  }
  Type & set__front_screen_mode(
    const uint8_t & _arg)
  {
    this->front_screen_mode = _arg;
    return *this;
  }
  Type & set__hmd_video_source(
    const uint8_t & _arg)
  {
    this->hmd_video_source = _arg;
    return *this;
  }
  Type & set__hmd_overlay_mode(
    const uint8_t & _arg)
  {
    this->hmd_overlay_mode = _arg;
    return *this;
  }
  Type & set__hmd_enabled(
    const bool & _arg)
  {
    this->hmd_enabled = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRONT_SCREEN_DEFAULT =
    0u;
  static constexpr uint8_t FRONT_SCREEN_MODE_A =
    1u;
  static constexpr uint8_t FRONT_SCREEN_MODE_B =
    2u;
  static constexpr uint8_t FRONT_SCREEN_MODE_C =
    3u;
  static constexpr uint8_t HMD_OVERLAY_NONE =
    0u;
  static constexpr uint8_t HMD_OVERLAY_MINIMAL =
    1u;
  static constexpr uint8_t HMD_OVERLAY_FULL =
    2u;

  // pointer types
  using RawPtr =
    chest_interfaces::msg::ButtonState_<ContainerAllocator> *;
  using ConstRawPtr =
    const chest_interfaces::msg::ButtonState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::ButtonState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::ButtonState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chest_interfaces__msg__ButtonState
    std::shared_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chest_interfaces__msg__ButtonState
    std::shared_ptr<chest_interfaces::msg::ButtonState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ButtonState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->head_tracking_enabled != other.head_tracking_enabled) {
      return false;
    }
    if (this->voice_command_enabled != other.voice_command_enabled) {
      return false;
    }
    if (this->front_screen_enabled != other.front_screen_enabled) {
      return false;
    }
    if (this->front_screen_mode != other.front_screen_mode) {
      return false;
    }
    if (this->hmd_video_source != other.hmd_video_source) {
      return false;
    }
    if (this->hmd_overlay_mode != other.hmd_overlay_mode) {
      return false;
    }
    if (this->hmd_enabled != other.hmd_enabled) {
      return false;
    }
    return true;
  }
  bool operator!=(const ButtonState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ButtonState_

// alias to use template instance with default allocator
using ButtonState =
  chest_interfaces::msg::ButtonState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::FRONT_SCREEN_DEFAULT;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::FRONT_SCREEN_MODE_A;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::FRONT_SCREEN_MODE_B;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::FRONT_SCREEN_MODE_C;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::HMD_OVERLAY_NONE;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::HMD_OVERLAY_MINIMAL;
template<typename ContainerAllocator>
constexpr uint8_t ButtonState_<ContainerAllocator>::HMD_OVERLAY_FULL;

}  // namespace msg

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__STRUCT_HPP_
