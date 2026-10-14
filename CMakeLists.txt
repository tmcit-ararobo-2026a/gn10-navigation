cmake_minimum_required(VERSION 3.18)
project(gn10_navigation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
option(BUILD_RUNTIME_NODES "Build planning and following runtime nodes" ON)

include_directories(
  include
)

if(BUILD_RUNTIME_NODES)
# C++ ノードのビルド
add_executable(costmap_generator_node
  src/map_loader.cpp
  src/costmap_generator_node.cpp
)


ament_target_dependencies(costmap_generator_node
  ament_index_cpp
  rclcpp
  nav_msgs
  nlohmann_json
)

add_executable(path_planner_node
  src/path_planner_node.cpp
)

ament_target_dependencies(path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# インストール設定 (Launch, Config, Node)
install(TARGETS 
  costmap_generator_node
  path_planner_node
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

endif()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
