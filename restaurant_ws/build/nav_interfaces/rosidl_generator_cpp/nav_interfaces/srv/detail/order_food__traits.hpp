// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav_interfaces:srv/OrderFood.idl
// generated code does not contain a copyright notice

#ifndef NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__TRAITS_HPP_
#define NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nav_interfaces/srv/detail/order_food__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nav_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const OrderFood_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: items
  {
    if (msg.items.size() == 0) {
      out << "items: []";
    } else {
      out << "items: [";
      size_t pending_items = msg.items.size();
      for (auto item : msg.items) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrderFood_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: items
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.items.size() == 0) {
      out << "items: []\n";
    } else {
      out << "items:\n";
      for (auto item : msg.items) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrderFood_Request & msg, bool use_flow_style = false)
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

}  // namespace nav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use nav_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav_interfaces::srv::OrderFood_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav_interfaces::srv::OrderFood_Request & msg)
{
  return nav_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav_interfaces::srv::OrderFood_Request>()
{
  return "nav_interfaces::srv::OrderFood_Request";
}

template<>
inline const char * name<nav_interfaces::srv::OrderFood_Request>()
{
  return "nav_interfaces/srv/OrderFood_Request";
}

template<>
struct has_fixed_size<nav_interfaces::srv::OrderFood_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav_interfaces::srv::OrderFood_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nav_interfaces::srv::OrderFood_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace nav_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const OrderFood_Response & msg,
  std::ostream & out)
{
  out << "{";
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
  const OrderFood_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const OrderFood_Response & msg, bool use_flow_style = false)
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

}  // namespace nav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use nav_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav_interfaces::srv::OrderFood_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav_interfaces::srv::OrderFood_Response & msg)
{
  return nav_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav_interfaces::srv::OrderFood_Response>()
{
  return "nav_interfaces::srv::OrderFood_Response";
}

template<>
inline const char * name<nav_interfaces::srv::OrderFood_Response>()
{
  return "nav_interfaces/srv/OrderFood_Response";
}

template<>
struct has_fixed_size<nav_interfaces::srv::OrderFood_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav_interfaces::srv::OrderFood_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nav_interfaces::srv::OrderFood_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav_interfaces::srv::OrderFood>()
{
  return "nav_interfaces::srv::OrderFood";
}

template<>
inline const char * name<nav_interfaces::srv::OrderFood>()
{
  return "nav_interfaces/srv/OrderFood";
}

template<>
struct has_fixed_size<nav_interfaces::srv::OrderFood>
  : std::integral_constant<
    bool,
    has_fixed_size<nav_interfaces::srv::OrderFood_Request>::value &&
    has_fixed_size<nav_interfaces::srv::OrderFood_Response>::value
  >
{
};

template<>
struct has_bounded_size<nav_interfaces::srv::OrderFood>
  : std::integral_constant<
    bool,
    has_bounded_size<nav_interfaces::srv::OrderFood_Request>::value &&
    has_bounded_size<nav_interfaces::srv::OrderFood_Response>::value
  >
{
};

template<>
struct is_service<nav_interfaces::srv::OrderFood>
  : std::true_type
{
};

template<>
struct is_service_request<nav_interfaces::srv::OrderFood_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nav_interfaces::srv::OrderFood_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__TRAITS_HPP_
