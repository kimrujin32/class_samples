// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav_interfaces:srv/OrderFood.idl
// generated code does not contain a copyright notice

#ifndef NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__STRUCT_H_
#define NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'items'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/OrderFood in the package nav_interfaces.
typedef struct nav_interfaces__srv__OrderFood_Request
{
  rosidl_runtime_c__String__Sequence items;
} nav_interfaces__srv__OrderFood_Request;

// Struct for a sequence of nav_interfaces__srv__OrderFood_Request.
typedef struct nav_interfaces__srv__OrderFood_Request__Sequence
{
  nav_interfaces__srv__OrderFood_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__srv__OrderFood_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/OrderFood in the package nav_interfaces.
typedef struct nav_interfaces__srv__OrderFood_Response
{
  bool success;
  rosidl_runtime_c__String message;
} nav_interfaces__srv__OrderFood_Response;

// Struct for a sequence of nav_interfaces__srv__OrderFood_Response.
typedef struct nav_interfaces__srv__OrderFood_Response__Sequence
{
  nav_interfaces__srv__OrderFood_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__srv__OrderFood_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__STRUCT_H_
