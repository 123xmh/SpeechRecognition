// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chest_interfaces:msg/VoiceText.idl
// generated code does not contain a copyright notice

#ifndef CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__TRAITS_HPP_
#define CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__TRAITS_HPP_

#include "chest_interfaces/msg/detail/voice_text__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chest_interfaces::msg::VoiceText>()
{
  return "chest_interfaces::msg::VoiceText";
}

template<>
inline const char * name<chest_interfaces::msg::VoiceText>()
{
  return "chest_interfaces/msg/VoiceText";
}

template<>
struct has_fixed_size<chest_interfaces::msg::VoiceText>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chest_interfaces::msg::VoiceText>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<chest_interfaces::msg::VoiceText>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHEST_INTERFACES__MSG__DETAIL__VOICE_TEXT__TRAITS_HPP_
