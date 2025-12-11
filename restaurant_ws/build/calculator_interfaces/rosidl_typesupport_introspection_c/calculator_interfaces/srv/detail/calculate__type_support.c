// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from calculator_interfaces:srv/Calculate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "calculator_interfaces/srv/detail/calculate__rosidl_typesupport_introspection_c.h"
#include "calculator_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "calculator_interfaces/srv/detail/calculate__functions.h"
#include "calculator_interfaces/srv/detail/calculate__struct.h"


// Include directives for member types
// Member `operation`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  calculator_interfaces__srv__Calculate_Request__init(message_memory);
}

void calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_fini_function(void * message_memory)
{
  calculator_interfaces__srv__Calculate_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_member_array[3] = {
  {
    "a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Request, a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "b",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Request, b),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Request, operation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_members = {
  "calculator_interfaces__srv",  // message namespace
  "Calculate_Request",  // message name
  3,  // number of fields
  sizeof(calculator_interfaces__srv__Calculate_Request),
  calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_member_array,  // message members
  calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_type_support_handle = {
  0,
  &calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_calculator_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Request)() {
  if (!calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_type_support_handle.typesupport_identifier) {
    calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &calculator_interfaces__srv__Calculate_Request__rosidl_typesupport_introspection_c__Calculate_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "calculator_interfaces/srv/detail/calculate__rosidl_typesupport_introspection_c.h"
// already included above
// #include "calculator_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "calculator_interfaces/srv/detail/calculate__functions.h"
// already included above
// #include "calculator_interfaces/srv/detail/calculate__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  calculator_interfaces__srv__Calculate_Response__init(message_memory);
}

void calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_fini_function(void * message_memory)
{
  calculator_interfaces__srv__Calculate_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_member_array[3] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(calculator_interfaces__srv__Calculate_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_members = {
  "calculator_interfaces__srv",  // message namespace
  "Calculate_Response",  // message name
  3,  // number of fields
  sizeof(calculator_interfaces__srv__Calculate_Response),
  calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_member_array,  // message members
  calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_type_support_handle = {
  0,
  &calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_calculator_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Response)() {
  if (!calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_type_support_handle.typesupport_identifier) {
    calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &calculator_interfaces__srv__Calculate_Response__rosidl_typesupport_introspection_c__Calculate_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "calculator_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "calculator_interfaces/srv/detail/calculate__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_members = {
  "calculator_interfaces__srv",  // service namespace
  "Calculate",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_Request_message_type_support_handle,
  NULL  // response message
  // calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_Response_message_type_support_handle
};

static rosidl_service_type_support_t calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_type_support_handle = {
  0,
  &calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_calculator_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate)() {
  if (!calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_type_support_handle.typesupport_identifier) {
    calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, calculator_interfaces, srv, Calculate_Response)()->data;
  }

  return &calculator_interfaces__srv__detail__calculate__rosidl_typesupport_introspection_c__Calculate_service_type_support_handle;
}
