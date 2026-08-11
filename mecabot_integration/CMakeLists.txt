cmake_minimum_required(VERSION 3.8)
project(mecabot_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Dependencies opzoeken
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(wheeltec_robot_msg REQUIRED)

# =========================================================================
#  1. C++ EXECUTABLES COMPILEREN
# =========================================================================

# A. Robot Controller Executable (rt_robot_controller)
add_executable(rt_robot_controller src/robot_controller.cpp)
ament_target_dependencies(rt_robot_controller
  rclcpp
  std_msgs
  geometry_msgs
)

# B. Behavior Tree Executable (rt_bt_node)
add_executable(rt_bt_node src/mainRT.cpp)
ament_target_dependencies(rt_bt_node
  rclcpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

# Install C++ targets naar de lib map van het package
install(TARGETS
  rt_robot_controller
  rt_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================================
#  2. PYTHON SCRIPTS & LAUNCH / CONFIG MAPS INSTALLEREN
# =========================================================================

# Installeer de Python scripts en maak ze uitvoerbaar voor 'ros2 run'
install(PROGRAMS
  scripts/quiz_socket_bridge.py
  scripts/robot_state_manager.py
  scripts/status_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installeer de launch- en config-mappen naar de share map voor 'ros2 launch'
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================================
#  3. TESTING & EXPORT
# =========================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

