// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from calculator_interfaces:srv/Calculate.idl
// generated code does not contain a copyright notice

#ifndef CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__BUILDER_HPP_
#define CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "calculator_interfaces/srv/detail/calculate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace calculator_interfaces
{

namespace srv
{

namespace builder
{

class Init_Calculate_Request_operation
{
public:
  explicit Init_Calculate_Request_operation(::calculator_interfaces::srv::Calculate_Request & msg)
  : msg_(msg)
  {}
  ::calculator_interfaces::srv::Calculate_Request operation(::calculator_interfaces::srv::Calculate_Request::_operation_type arg)
  {
    msg_.operation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Request msg_;
};

class Init_Calculate_Request_b
{
public:
  explicit Init_Calculate_Request_b(::calculator_interfaces::srv::Calculate_Request & msg)
  : msg_(msg)
  {}
  Init_Calculate_Request_operation b(::calculator_interfaces::srv::Calculate_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Calculate_Request_operation(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Request msg_;
};

class Init_Calculate_Request_a
{
public:
  Init_Calculate_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Calculate_Request_b a(::calculator_interfaces::srv::Calculate_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Calculate_Request_b(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::calculator_interfaces::srv::Calculate_Request>()
{
  return calculator_interfaces::srv::builder::Init_Calculate_Request_a();
}

}  // namespace calculator_interfaces


namespace calculator_interfaces
{

namespace srv
{

namespace builder
{

class Init_Calculate_Response_message
{
public:
  explicit Init_Calculate_Response_message(::calculator_interfaces::srv::Calculate_Response & msg)
  : msg_(msg)
  {}
  ::calculator_interfaces::srv::Calculate_Response message(::calculator_interfaces::srv::Calculate_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Response msg_;
};

class Init_Calculate_Response_success
{
public:
  explicit Init_Calculate_Response_success(::calculator_interfaces::srv::Calculate_Response & msg)
  : msg_(msg)
  {}
  Init_Calculate_Response_message success(::calculator_interfaces::srv::Calculate_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Calculate_Response_message(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Response msg_;
};

class Init_Calculate_Response_result
{
public:
  Init_Calculate_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Calculate_Response_success result(::calculator_interfaces::srv::Calculate_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_Calculate_Response_success(msg_);
  }

private:
  ::calculator_interfaces::srv::Calculate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::calculator_interfaces::srv::Calculate_Response>()
{
  return calculator_interfaces::srv::builder::Init_Calculate_Response_result();
}

}  // namespace calculator_interfaces

#endif  // CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__BUILDER_HPP_
