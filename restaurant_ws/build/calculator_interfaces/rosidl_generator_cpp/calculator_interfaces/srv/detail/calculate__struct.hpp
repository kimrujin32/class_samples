// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from calculator_interfaces:srv/Calculate.idl
// generated code does not contain a copyright notice

#ifndef CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_HPP_
#define CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__calculator_interfaces__srv__Calculate_Request __attribute__((deprecated))
#else
# define DEPRECATED__calculator_interfaces__srv__Calculate_Request __declspec(deprecated)
#endif

namespace calculator_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Calculate_Request_
{
  using Type = Calculate_Request_<ContainerAllocator>;

  explicit Calculate_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->operation = "";
    }
  }

  explicit Calculate_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operation(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->operation = "";
    }
  }

  // field types and members
  using _a_type =
    double;
  _a_type a;
  using _b_type =
    double;
  _b_type b;
  using _operation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_type operation;

  // setters for named parameter idiom
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const double & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__operation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__calculator_interfaces__srv__Calculate_Request
    std::shared_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__calculator_interfaces__srv__Calculate_Request
    std::shared_ptr<calculator_interfaces::srv::Calculate_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Calculate_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->operation != other.operation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Calculate_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Calculate_Request_

// alias to use template instance with default allocator
using Calculate_Request =
  calculator_interfaces::srv::Calculate_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace calculator_interfaces


#ifndef _WIN32
# define DEPRECATED__calculator_interfaces__srv__Calculate_Response __attribute__((deprecated))
#else
# define DEPRECATED__calculator_interfaces__srv__Calculate_Response __declspec(deprecated)
#endif

namespace calculator_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Calculate_Response_
{
  using Type = Calculate_Response_<ContainerAllocator>;

  explicit Calculate_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0.0;
      this->success = false;
      this->message = "";
    }
  }

  explicit Calculate_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0.0;
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _result_type =
    double;
  _result_type result;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__result(
    const double & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__calculator_interfaces__srv__Calculate_Response
    std::shared_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__calculator_interfaces__srv__Calculate_Response
    std::shared_ptr<calculator_interfaces::srv::Calculate_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Calculate_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Calculate_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Calculate_Response_

// alias to use template instance with default allocator
using Calculate_Response =
  calculator_interfaces::srv::Calculate_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace calculator_interfaces

namespace calculator_interfaces
{

namespace srv
{

struct Calculate
{
  using Request = calculator_interfaces::srv::Calculate_Request;
  using Response = calculator_interfaces::srv::Calculate_Response;
};

}  // namespace srv

}  // namespace calculator_interfaces

#endif  // CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_HPP_
