cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorParam.msg"
  "msg/MotorFeedbackEntry.msg"
  "msg/MotorFeedback.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(motor_sub src/motor_sub.cpp)
ament_target_dependencies(motor_sub rclcpp std_msgs)
rosidl_target_interfaces(motor_sub ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(motor_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS motor_sub DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  motor_test/motor_pub.py
  motor_test/motor_can_bridge.py
  motor_test/motor_feedback_listener.py
  motor_test/motor_can_publisher.py
  motor_test/motor_controller_test.py
  motor_test/serial_can_bridge.py
  motor_test/ethernet_can_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  motor_test/common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_LIST_DIR}/../../descriptions
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
