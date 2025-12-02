// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chest_interfaces:msg/EquipmentState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__chest_interfaces__msg__EquipmentState __attribute__((deprecated))
#else
# define DEPRECATED__chest_interfaces__msg__EquipmentState __declspec(deprecated)
#endif

namespace chest_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EquipmentState_
{
  using Type = EquipmentState_<ContainerAllocator>;

  explicit EquipmentState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gimbal_pitch = 0.0f;
      this->gimbal_yaw = 0.0f;
      this->gimbal_is_active = false;
      this->longitude = 0.0;
      this->latitude = 0.0;
      this->altitude = 0.0;
      this->ground_altitude = 0.0;
      this->heading = 0.0f;
      this->speed = 0.0f;
      this->fuel_level = 0.0f;
      this->battery_level = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->yaw = 0.0f;
    }
  }

  explicit EquipmentState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gimbal_pitch = 0.0f;
      this->gimbal_yaw = 0.0f;
      this->gimbal_is_active = false;
      this->longitude = 0.0;
      this->latitude = 0.0;
      this->altitude = 0.0;
      this->ground_altitude = 0.0;
      this->heading = 0.0f;
      this->speed = 0.0f;
      this->fuel_level = 0.0f;
      this->battery_level = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->yaw = 0.0f;
    }
  }

  // field types and members
  using _ammo_types_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _ammo_types_type ammo_types;
  using _ammo_counts_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _ammo_counts_type ammo_counts;
  using _gimbal_pitch_type =
    float;
  _gimbal_pitch_type gimbal_pitch;
  using _gimbal_yaw_type =
    float;
  _gimbal_yaw_type gimbal_yaw;
  using _gimbal_is_active_type =
    bool;
  _gimbal_is_active_type gimbal_is_active;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _ground_altitude_type =
    double;
  _ground_altitude_type ground_altitude;
  using _heading_type =
    float;
  _heading_type heading;
  using _speed_type =
    float;
  _speed_type speed;
  using _fuel_level_type =
    float;
  _fuel_level_type fuel_level;
  using _battery_level_type =
    float;
  _battery_level_type battery_level;
  using _roll_type =
    float;
  _roll_type roll;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _warnings_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _warnings_type warnings;
  using _state_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _state_names_type state_names;
  using _state_values_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _state_values_type state_values;

  // setters for named parameter idiom
  Type & set__ammo_types(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->ammo_types = _arg;
    return *this;
  }
  Type & set__ammo_counts(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->ammo_counts = _arg;
    return *this;
  }
  Type & set__gimbal_pitch(
    const float & _arg)
  {
    this->gimbal_pitch = _arg;
    return *this;
  }
  Type & set__gimbal_yaw(
    const float & _arg)
  {
    this->gimbal_yaw = _arg;
    return *this;
  }
  Type & set__gimbal_is_active(
    const bool & _arg)
  {
    this->gimbal_is_active = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__ground_altitude(
    const double & _arg)
  {
    this->ground_altitude = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__fuel_level(
    const float & _arg)
  {
    this->fuel_level = _arg;
    return *this;
  }
  Type & set__battery_level(
    const float & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__warnings(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->warnings = _arg;
    return *this;
  }
  Type & set__state_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->state_names = _arg;
    return *this;
  }
  Type & set__state_values(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->state_values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chest_interfaces::msg::EquipmentState_<ContainerAllocator> *;
  using ConstRawPtr =
    const chest_interfaces::msg::EquipmentState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::EquipmentState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chest_interfaces::msg::EquipmentState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chest_interfaces__msg__EquipmentState
    std::shared_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chest_interfaces__msg__EquipmentState
    std::shared_ptr<chest_interfaces::msg::EquipmentState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EquipmentState_ & other) const
  {
    if (this->ammo_types != other.ammo_types) {
      return false;
    }
    if (this->ammo_counts != other.ammo_counts) {
      return false;
    }
    if (this->gimbal_pitch != other.gimbal_pitch) {
      return false;
    }
    if (this->gimbal_yaw != other.gimbal_yaw) {
      return false;
    }
    if (this->gimbal_is_active != other.gimbal_is_active) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->ground_altitude != other.ground_altitude) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->fuel_level != other.fuel_level) {
      return false;
    }
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->warnings != other.warnings) {
      return false;
    }
    if (this->state_names != other.state_names) {
      return false;
    }
    if (this->state_values != other.state_values) {
      return false;
    }
    return true;
  }
  bool operator!=(const EquipmentState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EquipmentState_

// alias to use template instance with default allocator
using EquipmentState =
  chest_interfaces::msg::EquipmentState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__STRUCT_HPP_
