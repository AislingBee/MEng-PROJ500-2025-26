// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_test:msg/MotorFeedbackEntry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_feedback_entry.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_test__msg__MotorFeedbackEntry __attribute__((deprecated))
#else
# define DEPRECATED__motor_test__msg__MotorFeedbackEntry __declspec(deprecated)
#endif

namespace motor_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorFeedbackEntry_
{
  using Type = MotorFeedbackEntry_<ContainerAllocator>;

  explicit MotorFeedbackEntry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->q = 0.0;
      this->q_dot = 0.0;
    }
  }

  explicit MotorFeedbackEntry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->q = 0.0;
      this->q_dot = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _q_type =
    double;
  _q_type q;
  using _q_dot_type =
    double;
  _q_dot_type q_dot;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__q(
    const double & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__q_dot(
    const double & _arg)
  {
    this->q_dot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_test__msg__MotorFeedbackEntry
    std::shared_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_test__msg__MotorFeedbackEntry
    std::shared_ptr<motor_test::msg::MotorFeedbackEntry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorFeedbackEntry_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->q_dot != other.q_dot) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorFeedbackEntry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorFeedbackEntry_

// alias to use template instance with default allocator
using MotorFeedbackEntry =
  motor_test::msg::MotorFeedbackEntry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_FEEDBACK_ENTRY__STRUCT_HPP_
