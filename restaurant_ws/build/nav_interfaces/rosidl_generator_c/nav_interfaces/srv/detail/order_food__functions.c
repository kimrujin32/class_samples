// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav_interfaces:srv/OrderFood.idl
// generated code does not contain a copyright notice
#include "nav_interfaces/srv/detail/order_food__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `items`
#include "rosidl_runtime_c/string_functions.h"

bool
nav_interfaces__srv__OrderFood_Request__init(nav_interfaces__srv__OrderFood_Request * msg)
{
  if (!msg) {
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__Sequence__init(&msg->items, 0)) {
    nav_interfaces__srv__OrderFood_Request__fini(msg);
    return false;
  }
  return true;
}

void
nav_interfaces__srv__OrderFood_Request__fini(nav_interfaces__srv__OrderFood_Request * msg)
{
  if (!msg) {
    return;
  }
  // items
  rosidl_runtime_c__String__Sequence__fini(&msg->items);
}

bool
nav_interfaces__srv__OrderFood_Request__are_equal(const nav_interfaces__srv__OrderFood_Request * lhs, const nav_interfaces__srv__OrderFood_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->items), &(rhs->items)))
  {
    return false;
  }
  return true;
}

bool
nav_interfaces__srv__OrderFood_Request__copy(
  const nav_interfaces__srv__OrderFood_Request * input,
  nav_interfaces__srv__OrderFood_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->items), &(output->items)))
  {
    return false;
  }
  return true;
}

nav_interfaces__srv__OrderFood_Request *
nav_interfaces__srv__OrderFood_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Request * msg = (nav_interfaces__srv__OrderFood_Request *)allocator.allocate(sizeof(nav_interfaces__srv__OrderFood_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_interfaces__srv__OrderFood_Request));
  bool success = nav_interfaces__srv__OrderFood_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_interfaces__srv__OrderFood_Request__destroy(nav_interfaces__srv__OrderFood_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_interfaces__srv__OrderFood_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_interfaces__srv__OrderFood_Request__Sequence__init(nav_interfaces__srv__OrderFood_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Request * data = NULL;

  if (size) {
    data = (nav_interfaces__srv__OrderFood_Request *)allocator.zero_allocate(size, sizeof(nav_interfaces__srv__OrderFood_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_interfaces__srv__OrderFood_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_interfaces__srv__OrderFood_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_interfaces__srv__OrderFood_Request__Sequence__fini(nav_interfaces__srv__OrderFood_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_interfaces__srv__OrderFood_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_interfaces__srv__OrderFood_Request__Sequence *
nav_interfaces__srv__OrderFood_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Request__Sequence * array = (nav_interfaces__srv__OrderFood_Request__Sequence *)allocator.allocate(sizeof(nav_interfaces__srv__OrderFood_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_interfaces__srv__OrderFood_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_interfaces__srv__OrderFood_Request__Sequence__destroy(nav_interfaces__srv__OrderFood_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_interfaces__srv__OrderFood_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_interfaces__srv__OrderFood_Request__Sequence__are_equal(const nav_interfaces__srv__OrderFood_Request__Sequence * lhs, const nav_interfaces__srv__OrderFood_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_interfaces__srv__OrderFood_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_interfaces__srv__OrderFood_Request__Sequence__copy(
  const nav_interfaces__srv__OrderFood_Request__Sequence * input,
  nav_interfaces__srv__OrderFood_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_interfaces__srv__OrderFood_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_interfaces__srv__OrderFood_Request * data =
      (nav_interfaces__srv__OrderFood_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_interfaces__srv__OrderFood_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_interfaces__srv__OrderFood_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_interfaces__srv__OrderFood_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
nav_interfaces__srv__OrderFood_Response__init(nav_interfaces__srv__OrderFood_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    nav_interfaces__srv__OrderFood_Response__fini(msg);
    return false;
  }
  return true;
}

void
nav_interfaces__srv__OrderFood_Response__fini(nav_interfaces__srv__OrderFood_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
nav_interfaces__srv__OrderFood_Response__are_equal(const nav_interfaces__srv__OrderFood_Response * lhs, const nav_interfaces__srv__OrderFood_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
nav_interfaces__srv__OrderFood_Response__copy(
  const nav_interfaces__srv__OrderFood_Response * input,
  nav_interfaces__srv__OrderFood_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

nav_interfaces__srv__OrderFood_Response *
nav_interfaces__srv__OrderFood_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Response * msg = (nav_interfaces__srv__OrderFood_Response *)allocator.allocate(sizeof(nav_interfaces__srv__OrderFood_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_interfaces__srv__OrderFood_Response));
  bool success = nav_interfaces__srv__OrderFood_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nav_interfaces__srv__OrderFood_Response__destroy(nav_interfaces__srv__OrderFood_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nav_interfaces__srv__OrderFood_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nav_interfaces__srv__OrderFood_Response__Sequence__init(nav_interfaces__srv__OrderFood_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Response * data = NULL;

  if (size) {
    data = (nav_interfaces__srv__OrderFood_Response *)allocator.zero_allocate(size, sizeof(nav_interfaces__srv__OrderFood_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_interfaces__srv__OrderFood_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_interfaces__srv__OrderFood_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav_interfaces__srv__OrderFood_Response__Sequence__fini(nav_interfaces__srv__OrderFood_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_interfaces__srv__OrderFood_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav_interfaces__srv__OrderFood_Response__Sequence *
nav_interfaces__srv__OrderFood_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nav_interfaces__srv__OrderFood_Response__Sequence * array = (nav_interfaces__srv__OrderFood_Response__Sequence *)allocator.allocate(sizeof(nav_interfaces__srv__OrderFood_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nav_interfaces__srv__OrderFood_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nav_interfaces__srv__OrderFood_Response__Sequence__destroy(nav_interfaces__srv__OrderFood_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nav_interfaces__srv__OrderFood_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nav_interfaces__srv__OrderFood_Response__Sequence__are_equal(const nav_interfaces__srv__OrderFood_Response__Sequence * lhs, const nav_interfaces__srv__OrderFood_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_interfaces__srv__OrderFood_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_interfaces__srv__OrderFood_Response__Sequence__copy(
  const nav_interfaces__srv__OrderFood_Response__Sequence * input,
  nav_interfaces__srv__OrderFood_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_interfaces__srv__OrderFood_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nav_interfaces__srv__OrderFood_Response * data =
      (nav_interfaces__srv__OrderFood_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_interfaces__srv__OrderFood_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nav_interfaces__srv__OrderFood_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nav_interfaces__srv__OrderFood_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
