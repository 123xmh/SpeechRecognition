// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chest_interfaces:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__TRAITS_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__TRAITS_HPP_

#include "chest_interfaces/msg/detail/button_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chest_interfaces::msg::ButtonState>()
{
  return "chest_interfaces::msg::ButtonState";
}

template<>
inline const char * name<chest_interfaces::msg::ButtonState>()
{
  return "chest_interfaces/msg/ButtonState";
}

template<>
struct has_fixed_size<chest_interfaces::msg::ButtonState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<chest_interfaces::msg::ButtonState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<chest_interfaces::msg::ButtonState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHEST_INTERFACES__MSG__DETAIL__BUTTON_STATE__TRAITS_HPP_
