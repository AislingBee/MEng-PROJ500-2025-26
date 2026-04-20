// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_test:msg/MotorParam.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_test/msg/motor_param.hpp"


#ifndef MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_HPP_
#define MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_test__msg__MotorParam __attribute__((deprecated))
#else
# define DEPRECATED__motor_test__msg__MotorParam __declspec(deprecated)
#endif

namespace motor_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorParam_
{
  using Type = MotorParam_<ContainerAllocator>;

  explicit MotorParam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q = 0.0;
      this->kp = 0.0;
      this->kd = 0.0;
      this->tau = 0.0;
    }
  }

  explicit MotorParam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q = 0.0;
      this->kp = 0.0;
      this->kd = 0.0;
      this->tau = 0.0;
    }
  }

  // field types and members
  using _q_type =
    double;
  _q_type q;
  using _kp_type =
    double;
  _kp_type kp;
  using _kd_type =
    double;
  _kd_type kd;
  using _tau_type =
    double;
  _tau_type tau;

  // setters for named parameter idiom
  Type & set__q(
    const double & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__kp(
    const double & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const double & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__tau(
    const double & _arg)
  {
    this->tau = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_test::msg::MotorParam_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_test::msg::MotorParam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_test::msg::MotorParam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_test::msg::MotorParam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorParam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorParam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_test::msg::MotorParam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_test::msg::MotorParam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_test::msg::MotorParam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_test::msg::MotorParam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_test__msg__MotorParam
    std::shared_ptr<motor_test::msg::MotorParam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_test__msg__MotorParam
    std::shared_ptr<motor_test::msg::MotorParam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorParam_ & other) const
  {
    if (this->q != other.q) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->tau != other.tau) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorParam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorParam_

// alias to use template instance with default allocator
using MotorParam =
  motor_test::msg::MotorParam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_test

#endif  // MOTOR_TEST__MSG__DETAIL__MOTOR_PARAM__STRUCT_HPP_
