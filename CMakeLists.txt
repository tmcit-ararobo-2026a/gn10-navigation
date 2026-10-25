cmake_minimum_required(VERSION 3.18)
project(gn10_navigation LANGUAGES C CXX)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UpdatePoint.srv"
  DEPENDENCIES builtin_interfaces
)

include_directories(
  include
)

# Target repository library
add_library(target_repository
  src/target_repository.cpp
)
target_include_directories(target_repository PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(target_repository PUBLIC
  GN10_NAVIGATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(target_repository
  yaml-cpp
)

# 1. target_manager_node
add_executable(target_manager_node
  src/target_manager_node.cpp
)
ament_target_dependencies(target_manager_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
)
target_link_libraries(target_manager_node
  target_repository
  yaml-cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(target_manager_node "${cpp_typesupport_target}")

# 2. costmap_generator_node
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

# 3. path_planner_node
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

# 4. path_follower_node
add_executable(path_follower_node
  src/path_follower_node.cpp
)
ament_target_dependencies(path_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# インストール設定 (Launch, Config, Node)
install(TARGETS 
  target_repository
  target_manager_node
  costmap_generator_node
  path_planner_node
  path_follower_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/grid_safety_test.cpp")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(grid_safety_test test/grid_safety_test.cpp)
  target_include_directories(grid_safety_test PRIVATE include)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
