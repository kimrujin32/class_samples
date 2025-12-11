// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from nav_interfaces:srv/OrderFood.idl
// generated code does not contain a copyright notice

#ifndef NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__FUNCTIONS_H_
#define NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "nav_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "nav_interfaces/srv/detail/order_food__struct.h"

/// Initialize srv/OrderFood message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nav_interfaces__srv__OrderFood_Request
 * )) before or use
 * nav_interfaces__srv__OrderFood_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__init(nav_interfaces__srv__OrderFood_Request * msg);

/// Finalize srv/OrderFood message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Request__fini(nav_interfaces__srv__OrderFood_Request * msg);

/// Create srv/OrderFood message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nav_interfaces__srv__OrderFood_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
nav_interfaces__srv__OrderFood_Request *
nav_interfaces__srv__OrderFood_Request__create();

/// Destroy srv/OrderFood message.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Request__destroy(nav_interfaces__srv__OrderFood_Request * msg);

/// Check for srv/OrderFood message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__are_equal(const nav_interfaces__srv__OrderFood_Request * lhs, const nav_interfaces__srv__OrderFood_Request * rhs);

/// Copy a srv/OrderFood message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__copy(
  const nav_interfaces__srv__OrderFood_Request * input,
  nav_interfaces__srv__OrderFood_Request * output);

/// Initialize array of srv/OrderFood messages.
/**
 * It allocates the memory for the number of elements and calls
 * nav_interfaces__srv__OrderFood_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__Sequence__init(nav_interfaces__srv__OrderFood_Request__Sequence * array, size_t size);

/// Finalize array of srv/OrderFood messages.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Request__Sequence__fini(nav_interfaces__srv__OrderFood_Request__Sequence * array);

/// Create array of srv/OrderFood messages.
/**
 * It allocates the memory for the array and calls
 * nav_interfaces__srv__OrderFood_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
nav_interfaces__srv__OrderFood_Request__Sequence *
nav_interfaces__srv__OrderFood_Request__Sequence__create(size_t size);

/// Destroy array of srv/OrderFood messages.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Request__Sequence__destroy(nav_interfaces__srv__OrderFood_Request__Sequence * array);

/// Check for srv/OrderFood message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__Sequence__are_equal(const nav_interfaces__srv__OrderFood_Request__Sequence * lhs, const nav_interfaces__srv__OrderFood_Request__Sequence * rhs);

/// Copy an array of srv/OrderFood messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Request__Sequence__copy(
  const nav_interfaces__srv__OrderFood_Request__Sequence * input,
  nav_interfaces__srv__OrderFood_Request__Sequence * output);

/// Initialize srv/OrderFood message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nav_interfaces__srv__OrderFood_Response
 * )) before or use
 * nav_interfaces__srv__OrderFood_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__init(nav_interfaces__srv__OrderFood_Response * msg);

/// Finalize srv/OrderFood message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Response__fini(nav_interfaces__srv__OrderFood_Response * msg);

/// Create srv/OrderFood message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nav_interfaces__srv__OrderFood_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
nav_interfaces__srv__OrderFood_Response *
nav_interfaces__srv__OrderFood_Response__create();

/// Destroy srv/OrderFood message.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Response__destroy(nav_interfaces__srv__OrderFood_Response * msg);

/// Check for srv/OrderFood message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__are_equal(const nav_interfaces__srv__OrderFood_Response * lhs, const nav_interfaces__srv__OrderFood_Response * rhs);

/// Copy a srv/OrderFood message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__copy(
  const nav_interfaces__srv__OrderFood_Response * input,
  nav_interfaces__srv__OrderFood_Response * output);

/// Initialize array of srv/OrderFood messages.
/**
 * It allocates the memory for the number of elements and calls
 * nav_interfaces__srv__OrderFood_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__Sequence__init(nav_interfaces__srv__OrderFood_Response__Sequence * array, size_t size);

/// Finalize array of srv/OrderFood messages.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Response__Sequence__fini(nav_interfaces__srv__OrderFood_Response__Sequence * array);

/// Create array of srv/OrderFood messages.
/**
 * It allocates the memory for the array and calls
 * nav_interfaces__srv__OrderFood_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
nav_interfaces__srv__OrderFood_Response__Sequence *
nav_interfaces__srv__OrderFood_Response__Sequence__create(size_t size);

/// Destroy array of srv/OrderFood messages.
/**
 * It calls
 * nav_interfaces__srv__OrderFood_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
void
nav_interfaces__srv__OrderFood_Response__Sequence__destroy(nav_interfaces__srv__OrderFood_Response__Sequence * array);

/// Check for srv/OrderFood message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__Sequence__are_equal(const nav_interfaces__srv__OrderFood_Response__Sequence * lhs, const nav_interfaces__srv__OrderFood_Response__Sequence * rhs);

/// Copy an array of srv/OrderFood messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav_interfaces
bool
nav_interfaces__srv__OrderFood_Response__Sequence__copy(
  const nav_interfaces__srv__OrderFood_Response__Sequence * input,
  nav_interfaces__srv__OrderFood_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NAV_INTERFACES__SRV__DETAIL__ORDER_FOOD__FUNCTIONS_H_
