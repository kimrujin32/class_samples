// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from calculator_interfaces:srv/Calculate.idl
// generated code does not contain a copyright notice

#ifndef CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_H_
#define CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'operation'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Calculate in the package calculator_interfaces.
typedef struct calculator_interfaces__srv__Calculate_Request
{
  double a;
  double b;
  /// 'add', 'subtract', 'multiply', 'divide'
  rosidl_runtime_c__String operation;
} calculator_interfaces__srv__Calculate_Request;

// Struct for a sequence of calculator_interfaces__srv__Calculate_Request.
typedef struct calculator_interfaces__srv__Calculate_Request__Sequence
{
  calculator_interfaces__srv__Calculate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} calculator_interfaces__srv__Calculate_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Calculate in the package calculator_interfaces.
typedef struct calculator_interfaces__srv__Calculate_Response
{
  double result;
  bool success;
  rosidl_runtime_c__String message;
} calculator_interfaces__srv__Calculate_Response;

// Struct for a sequence of calculator_interfaces__srv__Calculate_Response.
typedef struct calculator_interfaces__srv__Calculate_Response__Sequence
{
  calculator_interfaces__srv__Calculate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} calculator_interfaces__srv__Calculate_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CALCULATOR_INTERFACES__SRV__DETAIL__CALCULATE__STRUCT_H_
