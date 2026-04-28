cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorParam.msg"
  "msg/MotorFeedbackEntry.msg"
  "msg/MotorFeedback.msg"
  "msg/RobotCommand.msg"
  "msg/RobotObservation.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Determine the Python site-packages install path so common.py is importable
# as motor_control.common alongside the generated message stubs.
execute_process(
  COMMAND python3 -c
    "import sys; print('lib/python{}.{}/site-packages'.format(sys.version_info.major, sys.version_info.minor))"
  OUTPUT_VARIABLE motor_control_PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
install(FILES motor_control/common.py
  DESTINATION ${motor_control_PYTHON_INSTALL_DIR}/motor_control
)

install(PROGRAMS
  motor_control/bridges/ethernet_can_bridge.py
  motor_control/bridges/serial_can_bridge.py
  motor_control/bridges/robot_command_bridge.py
  motor_control/bridges/robot_observation_bridge.py
  motor_control/sensors/imu_publisher.py
  motor_control/sensors/motor_feedback_listener.py
  motor_control/testing/motor_controller_test.py
  motor_control/testing/motor_pub.py
  motor_control/testing/single_motor_control.py
  motor_control/testing/multi_motor_control.py
  motor_control/testing/multi_state_motor_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_LIST_DIR}/../../config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.json"
)

ament_package()
