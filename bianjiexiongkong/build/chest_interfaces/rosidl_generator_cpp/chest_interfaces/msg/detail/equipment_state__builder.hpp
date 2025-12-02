// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chest_interfaces:msg/EquipmentState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__BUILDER_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__BUILDER_HPP_

#include "chest_interfaces/msg/detail/equipment_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace chest_interfaces
{

namespace msg
{

namespace builder
{

class Init_EquipmentState_state_values
{
public:
  explicit Init_EquipmentState_state_values(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  ::chest_interfaces::msg::EquipmentState state_values(::chest_interfaces::msg::EquipmentState::_state_values_type arg)
  {
    msg_.state_values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_state_names
{
public:
  explicit Init_EquipmentState_state_names(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_state_values state_names(::chest_interfaces::msg::EquipmentState::_state_names_type arg)
  {
    msg_.state_names = std::move(arg);
    return Init_EquipmentState_state_values(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_warnings
{
public:
  explicit Init_EquipmentState_warnings(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_state_names warnings(::chest_interfaces::msg::EquipmentState::_warnings_type arg)
  {
    msg_.warnings = std::move(arg);
    return Init_EquipmentState_state_names(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_yaw
{
public:
  explicit Init_EquipmentState_yaw(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_warnings yaw(::chest_interfaces::msg::EquipmentState::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_EquipmentState_warnings(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_pitch
{
public:
  explicit Init_EquipmentState_pitch(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_yaw pitch(::chest_interfaces::msg::EquipmentState::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_EquipmentState_yaw(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_roll
{
public:
  explicit Init_EquipmentState_roll(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_pitch roll(::chest_interfaces::msg::EquipmentState::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_EquipmentState_pitch(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_battery_level
{
public:
  explicit Init_EquipmentState_battery_level(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_roll battery_level(::chest_interfaces::msg::EquipmentState::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_EquipmentState_roll(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_fuel_level
{
public:
  explicit Init_EquipmentState_fuel_level(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_battery_level fuel_level(::chest_interfaces::msg::EquipmentState::_fuel_level_type arg)
  {
    msg_.fuel_level = std::move(arg);
    return Init_EquipmentState_battery_level(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_speed
{
public:
  explicit Init_EquipmentState_speed(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_fuel_level speed(::chest_interfaces::msg::EquipmentState::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_EquipmentState_fuel_level(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_heading
{
public:
  explicit Init_EquipmentState_heading(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_speed heading(::chest_interfaces::msg::EquipmentState::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_EquipmentState_speed(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_ground_altitude
{
public:
  explicit Init_EquipmentState_ground_altitude(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_heading ground_altitude(::chest_interfaces::msg::EquipmentState::_ground_altitude_type arg)
  {
    msg_.ground_altitude = std::move(arg);
    return Init_EquipmentState_heading(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_altitude
{
public:
  explicit Init_EquipmentState_altitude(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_ground_altitude altitude(::chest_interfaces::msg::EquipmentState::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_EquipmentState_ground_altitude(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_latitude
{
public:
  explicit Init_EquipmentState_latitude(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_altitude latitude(::chest_interfaces::msg::EquipmentState::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_EquipmentState_altitude(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_longitude
{
public:
  explicit Init_EquipmentState_longitude(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_latitude longitude(::chest_interfaces::msg::EquipmentState::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_EquipmentState_latitude(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_gimbal_is_active
{
public:
  explicit Init_EquipmentState_gimbal_is_active(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_longitude gimbal_is_active(::chest_interfaces::msg::EquipmentState::_gimbal_is_active_type arg)
  {
    msg_.gimbal_is_active = std::move(arg);
    return Init_EquipmentState_longitude(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_gimbal_yaw
{
public:
  explicit Init_EquipmentState_gimbal_yaw(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_gimbal_is_active gimbal_yaw(::chest_interfaces::msg::EquipmentState::_gimbal_yaw_type arg)
  {
    msg_.gimbal_yaw = std::move(arg);
    return Init_EquipmentState_gimbal_is_active(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_gimbal_pitch
{
public:
  explicit Init_EquipmentState_gimbal_pitch(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_gimbal_yaw gimbal_pitch(::chest_interfaces::msg::EquipmentState::_gimbal_pitch_type arg)
  {
    msg_.gimbal_pitch = std::move(arg);
    return Init_EquipmentState_gimbal_yaw(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_ammo_counts
{
public:
  explicit Init_EquipmentState_ammo_counts(::chest_interfaces::msg::EquipmentState & msg)
  : msg_(msg)
  {}
  Init_EquipmentState_gimbal_pitch ammo_counts(::chest_interfaces::msg::EquipmentState::_ammo_counts_type arg)
  {
    msg_.ammo_counts = std::move(arg);
    return Init_EquipmentState_gimbal_pitch(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

class Init_EquipmentState_ammo_types
{
public:
  Init_EquipmentState_ammo_types()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EquipmentState_ammo_counts ammo_types(::chest_interfaces::msg::EquipmentState::_ammo_types_type arg)
  {
    msg_.ammo_types = std::move(arg);
    return Init_EquipmentState_ammo_counts(msg_);
  }

private:
  ::chest_interfaces::msg::EquipmentState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chest_interfaces::msg::EquipmentState>()
{
  return chest_interfaces::msg::builder::Init_EquipmentState_ammo_types();
}

}  // namespace chest_interfaces

#endif  // CHEST_INTERFACES__MSG__DETAIL__EQUIPMENT_STATE__BUILDER_HPP_
