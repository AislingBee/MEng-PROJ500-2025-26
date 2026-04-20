// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_test:msg/MotorFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motors'
#include "motor_test/msg/detail/motor_feedback_entry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__motor_test__msg__MotorFeedback __attribute__((deprecated))
#else
# define DEPRECATED__motor_test__msg__MotorFeedback __declspec(deprecated)
#endif

namespace motor_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorFeedback_
{
  using Type = MotorFeedback_<ContainerAllocator>;

  explicit MotorFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _motors_type =
    std::vector<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>>;
  _motors_type motors;

  // setters for named parameter idiom
  Type & set__motors(
    const std::vector<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>> & _arg)
  {
    this->motors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_test::msg::MotorFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_test::msg::MotorFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_test__msg__MotorFeedback
    std::shared_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_test__msg__MotorFeedback
    std::shared_ptr<motor_test::msg::MotorFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorFeedback_ & other) const
  {
    if (this->motors != other.motors) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorFeedback_

// alias to use template instance with default allocator
using MotorFeedback =
  motor_test::msg::MotorFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK__STRUCT_HPP_
