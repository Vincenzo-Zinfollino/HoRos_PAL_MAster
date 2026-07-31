cmake_minimum_required(VERSION 3.8)
project(tiago_manipulation_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ricerca dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

# --- MACRO TASK 1: PICK AND NAVIGATE ---
add_executable(pick_macro_task_node src/pick_macro_task_node.cpp)
ament_target_dependencies(pick_macro_task_node
  rclcpp tf2_ros geometry_msgs shape_msgs sensor_msgs std_msgs gazebo_msgs moveit_msgs moveit_ros_planning_interface moveit_core
)

# --- MACRO TASK 2: PLACE ON SHELF ---
add_executable(place_macro_task_node src/place_macro_task_node.cpp)
ament_target_dependencies(place_macro_task_node
  rclcpp tf2_ros geometry_msgs shape_msgs sensor_msgs std_msgs gazebo_msgs moveit_msgs moveit_ros_planning_interface moveit_core
)

# Installazione binari
install(TARGETS
  pick_macro_task_node
  place_macro_task_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()