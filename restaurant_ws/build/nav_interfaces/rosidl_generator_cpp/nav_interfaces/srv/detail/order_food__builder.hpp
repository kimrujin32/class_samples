// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav_interfaces:srv/OrderFood.idl
// generated code does not contain a copyright notice

#ifndef NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__BUILDER_HPP_
#define NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav_interfaces/srv/detail/order_food__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav_interfaces
{

namespace srv
{

namespace builder
{

class Init_OrderFood_Request_items
{
public:
  Init_OrderFood_Request_items()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav_interfaces::srv::OrderFood_Request items(::nav_interfaces::srv::OrderFood_Request::_items_type arg)
  {
    msg_.items = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav_interfaces::srv::OrderFood_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav_interfaces::srv::OrderFood_Request>()
{
  return nav_interfaces::srv::builder::Init_OrderFood_Request_items();
}

}  // namespace nav_interfaces


namespace nav_interfaces
{

namespace srv
{

namespace builder
{

class Init_OrderFood_Response_message
{
public:
  explicit Init_OrderFood_Response_message(::nav_interfaces::srv::OrderFood_Response & msg)
  : msg_(msg)
  {}
  ::nav_interfaces::srv::OrderFood_Response message(::nav_interfaces::srv::OrderFood_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav_interfaces::srv::OrderFood_Response msg_;
};

class Init_OrderFood_Response_success
{
public:
  Init_OrderFood_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderFood_Response_message success(::nav_interfaces::srv::OrderFood_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_OrderFood_Response_message(msg_);
  }

private:
  ::nav_interfaces::srv::OrderFood_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav_interfaces::srv::OrderFood_Response>()
{
  return nav_interfaces::srv::builder::Init_OrderFood_Response_success();
}

}  // namespace nav_interfaces

#endif  // NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__BUILDER_HPP_
