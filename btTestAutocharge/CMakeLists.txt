cmake_minimum_required(VERSION 3.8)
project(btTestAutocharge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --------------------
# Executable
# --------------------
add_executable(bt_main
  src/main.cpp
)

ament_target_dependencies(bt_main
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
  nav_msgs
)

# --------------------
# Install
# --------------------
install(TARGETS
  bt_main
  DESTINATION lib/${PROJECT_NAME}
)

# (OPTIONEEL maar netjes)
install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

