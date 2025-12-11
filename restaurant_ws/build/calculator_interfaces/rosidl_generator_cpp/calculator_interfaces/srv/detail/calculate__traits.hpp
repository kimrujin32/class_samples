// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from calculator_interfaces:srv/Calculate.idl
// generated code does not contain a copyright notice

#ifndef CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__TRAITS_HPP_
#define CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "calculator_interfaces/srv/detail/calculate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace calculator_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calculate_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: operation
  {
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calculate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: operation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calculate_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace calculator_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use calculator_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const calculator_interfaces::srv::Calculate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  calculator_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use calculator_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const calculator_interfaces::srv::Calculate_Request & msg)
{
  return calculator_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<calculator_interfaces::srv::Calculate_Request>()
{
  return "calculator_interfaces::srv::Calculate_Request";
}

template<>
inline const char * name<calculator_interfaces::srv::Calculate_Request>()
{
  return "calculator_interfaces/srv/Calculate_Request";
}

template<>
struct has_fixed_size<calculator_interfaces::srv::Calculate_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<calculator_interfaces::srv::Calculate_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<calculator_interfaces::srv::Calculate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace calculator_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calculate_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calculate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calculate_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace calculator_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use calculator_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const calculator_interfaces::srv::Calculate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  calculator_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use calculator_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const calculator_interfaces::srv::Calculate_Response & msg)
{
  return calculator_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<calculator_interfaces::srv::Calculate_Response>()
{
  return "calculator_interfaces::srv::Calculate_Response";
}

template<>
inline const char * name<calculator_interfaces::srv::Calculate_Response>()
{
  return "calculator_interfaces/srv/Calculate_Response";
}

template<>
struct has_fixed_size<calculator_interfaces::srv::Calculate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<calculator_interfaces::srv::Calculate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<calculator_interfaces::srv::Calculate_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<calculator_interfaces::srv::Calculate>()
{
  return "calculator_interfaces::srv::Calculate";
}

template<>
inline const char * name<calculator_interfaces::srv::Calculate>()
{
  return "calculator_interfaces/srv/Calculate";
}

template<>
struct has_fixed_size<calculator_interfaces::srv::Calculate>
  : std::integral_constant<
    bool,
    has_fixed_size<calculator_interfaces::srv::Calculate_Request>::value &&
    has_fixed_size<calculator_interfaces::srv::Calculate_Response>::value
  >
{
};

template<>
struct has_bounded_size<calculator_interfaces::srv::Calculate>
  : std::integral_constant<
    bool,
    has_bounded_size<calculator_interfaces::srv::Calculate_Request>::value &&
    has_bounded_size<calculator_interfaces::srv::Calculate_Response>::value
  >
{
};

template<>
struct is_service<calculator_interfaces::srv::Calculate>
  : std::true_type
{
};

template<>
struct is_service_request<calculator_interfaces::srv::Calculate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<calculator_interfaces::srv::Calculate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__TRAITS_HPP_
