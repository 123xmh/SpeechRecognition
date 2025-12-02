// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chest_interfaces:msg/VideoSource.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_HPP_

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
# define DEPRECATED__chest_interfaces__msg__VideoSource __attribute__((deprecated))
#else
# define DEPRECATED__chest_interfaces__msg__VideoSource __declspec(deprecated)
#endif

namespace chest_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VideoSource_
{
  using Type = VideoSource_<ContainerAllocator>;

  explicit VideoSource_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->ip_address = "";
      this->source_name = "";
      this->fps = 0ul;
      this->width = 0ul;
      this->height = 0ul;
      this->encoding = "";
      this->is_available = false;
    }
  }

  explicit VideoSource_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ip_address(_alloc),
    source_name(_alloc),
    encoding(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->ip_address = "";
      this->source_name = "";
      this->fps = 0ul;
      this->width = 0ul;
      this->height = 0ul;
      this->encoding = "";
      this->is_available = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint8_t;
  _id_type id;
  using _ip_address_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _ip_address_type ip_address;
  using _source_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _source_name_type source_name;
  using _fps_type =
    uint32_t;
  _fps_type fps;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _encoding_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _encoding_type encoding;
  using _is_available_type =
    bool;
  _is_available_type is_available;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__ip_address(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->ip_address = _arg;
    return *this;
  }
  Type & set__source_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->source_name = _arg;
    return *this;
  }
  Type & set__fps(
    const uint32_t & _arg)
  {
    this->fps = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__encoding(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->encoding = _arg;
    return *this;
  }
  Type & set__is_available(
    const bool & _arg)
  {
    this->is_available = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chest_interfaces::msg::VideoSource_<ContainerAllocator> *;
  using ConstRawPtr =
    const chest_interfaces::msg::VideoSource_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VideoSource_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::VideoSource_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chest_interfaces__msg__VideoSource
    std::shared_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chest_interfaces__msg__VideoSource
    std::shared_ptr<chest_interfaces::msg::VideoSource_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VideoSource_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->ip_address != other.ip_address) {
      return false;
    }
    if (this->source_name != other.source_name) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->encoding != other.encoding) {
      return false;
    }
    if (this->is_available != other.is_available) {
      return false;
    }
    return true;
  }
  bool operator!=(const VideoSource_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VideoSource_

// alias to use template instance with default allocator
using VideoSource =
  chest_interfaces::msg::VideoSource_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__VIDEO_SOURCE__STRUCT_HPP_
