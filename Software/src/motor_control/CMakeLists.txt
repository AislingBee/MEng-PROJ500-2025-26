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
find_package(std_srvs REQUIRED)
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
  OUTPUT_VARIABLE MOTOR_CONTROL_PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
install(FILES
  motor_control/common.py
  motor_control/rcu_protocol.py
  DESTINATION ${MOTOR_CONTROL_PYTHON_INSTALL_DIR}/motor_control
)

set(MOTOR_CONTROL_PY_SCRIPTS
  motor_control/pretty_robot_moves.py
  motor_control/ros_topic_gui.py
  motor_control/bridges/rcu_udp_bridge.py
  motor_control/bridges/robot_command_bridge.py
  motor_control/bridges/robot_observation_bridge.py
  motor_control/sensors/imu_publisher.py
  motor_control/sensors/motor_feedback_listener.py
  motor_control/testing/motor_controller_test.py
  motor_control/testing/motor_pub.py
  motor_control/testing/single_motor_test.py
  motor_control/testing/multi_motor_test.py
  motor_control/testing/multi_state_motor_test.py
  motor_control/testing/rcu_bench_command_test.py
  motor_control/testing/hold_position_test.py
  motor_control/testing/enable_motors.py
  motor_control/testing/sequential_motor_zero.py
)

# Ensure scripts are executable even when source checkouts lose +x bits.
set(MOTOR_CONTROL_PY_SCRIPTS_ABS "")
foreach(_script IN LISTS MOTOR_CONTROL_PY_SCRIPTS)
  set(_script_abs "${CMAKE_CURRENT_SOURCE_DIR}/${_script}")
  if(EXISTS "${_script_abs}")
    list(APPEND MOTOR_CONTROL_PY_SCRIPTS_ABS "${_script_abs}")
  else()
    message(WARNING "Skipping chmod for missing script: ${_script_abs}")
  endif()
endforeach()

if(MOTOR_CONTROL_PY_SCRIPTS_ABS)
  file(CHMOD ${MOTOR_CONTROL_PY_SCRIPTS_ABS}
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endif()

install(PROGRAMS
  ${MOTOR_CONTROL_PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  motor_control/pretty_robot_moves_joint_limits.json
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
