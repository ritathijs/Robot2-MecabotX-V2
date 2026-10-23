cmake_minimum_required(VERSION 3.8)
project(mecabot_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(rt_robot_controller src/robot_controller.cpp)
ament_target_dependencies(rt_robot_controller
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(rt_bt_node src/mainRT.cpp)
ament_target_dependencies(rt_bt_node
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
  behaviortree_cpp_v3
)

install(TARGETS
  rt_robot_controller
  rt_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/quiz_socket_bridge.py
  scripts/robot_state_manager.py
  scripts/rt_drive_to_coord.py
  scripts/rt_auto_recharger.py
  scripts/sim_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
