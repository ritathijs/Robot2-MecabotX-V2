cmake_minimum_required(VERSION 3.8)
project(mecabot_bt)

# ------------------------------
# Vind packages
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)




# include headers (als je ooit .hpp toevoegt)
include_directories(include)

# ------------------------------
# Maak executable
# ------------------------------
add_executable(bt_node src/main.cpp)

ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
)

# ------------------------------
# Installeer de executable
# ------------------------------
install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------
# Installeer de XML bomen
# ------------------------------
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

# ------------------------------
# Eind
# ------------------------------
ament_package()

