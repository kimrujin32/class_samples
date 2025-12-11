// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav_interfaces:action/Navigate.idl
// generated code does not contain a copyright notice

#ifndef NAV_INTERFACES__ACTION__DETAIL__NAVIGATE__STRUCT_H_
#define NAV_INTERFACES__ACTION__DETAIL__NAVIGATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'location'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_Goal
{
  /// Target location name
  rosidl_runtime_c__String location;
  /// Time to wait at location in seconds
  float wait_time;
} nav_interfaces__action__Navigate_Goal;

// Struct for a sequence of nav_interfaces__action__Navigate_Goal.
typedef struct nav_interfaces__action__Navigate_Goal__Sequence
{
  nav_interfaces__action__Navigate_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'final_location'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_Result
{
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String final_location;
  float total_time;
} nav_interfaces__action__Navigate_Result;

// Struct for a sequence of nav_interfaces__action__Navigate_Result.
typedef struct nav_interfaces__action__Navigate_Result__Sequence
{
  nav_interfaces__action__Navigate_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'current_location'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_Feedback
{
  rosidl_runtime_c__String current_state;
  float time_elapsed;
  rosidl_runtime_c__String current_location;
} nav_interfaces__action__Navigate_Feedback;

// Struct for a sequence of nav_interfaces__action__Navigate_Feedback.
typedef struct nav_interfaces__action__Navigate_Feedback__Sequence
{
  nav_interfaces__action__Navigate_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "nav_interfaces/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav_interfaces__action__Navigate_Goal goal;
} nav_interfaces__action__Navigate_SendGoal_Request;

// Struct for a sequence of nav_interfaces__action__Navigate_SendGoal_Request.
typedef struct nav_interfaces__action__Navigate_SendGoal_Request__Sequence
{
  nav_interfaces__action__Navigate_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} nav_interfaces__action__Navigate_SendGoal_Response;

// Struct for a sequence of nav_interfaces__action__Navigate_SendGoal_Response.
typedef struct nav_interfaces__action__Navigate_SendGoal_Response__Sequence
{
  nav_interfaces__action__Navigate_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} nav_interfaces__action__Navigate_GetResult_Request;

// Struct for a sequence of nav_interfaces__action__Navigate_GetResult_Request.
typedef struct nav_interfaces__action__Navigate_GetResult_Request__Sequence
{
  nav_interfaces__action__Navigate_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "nav_interfaces/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_GetResult_Response
{
  int8_t status;
  nav_interfaces__action__Navigate_Result result;
} nav_interfaces__action__Navigate_GetResult_Response;

// Struct for a sequence of nav_interfaces__action__Navigate_GetResult_Response.
typedef struct nav_interfaces__action__Navigate_GetResult_Response__Sequence
{
  nav_interfaces__action__Navigate_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "nav_interfaces/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package nav_interfaces.
typedef struct nav_interfaces__action__Navigate_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav_interfaces__action__Navigate_Feedback feedback;
} nav_interfaces__action__Navigate_FeedbackMessage;

// Struct for a sequence of nav_interfaces__action__Navigate_FeedbackMessage.
typedef struct nav_interfaces__action__Navigate_FeedbackMessage__Sequence
{
  nav_interfaces__action__Navigate_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav_interfaces__action__Navigate_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV_INTERFACES__ACTION__DETAIL__NAVIGATE__STRUCT_H_
