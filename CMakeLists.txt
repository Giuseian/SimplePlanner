cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(include)

add_executable(simple_planner_node 
  src/main.cpp
  src/simple_planner_node.cpp
  src/map_utils.cpp
  src/cost_utils.cpp
  src/a_star.cpp)

ament_target_dependencies(simple_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

install(TARGETS
  simple_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
