cmake_minimum_required(VERSION 3.16)
project(go2_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)

add_executable(map_matcher_node src/map_matcher_node.cpp)

target_include_directories(map_matcher_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_matcher_node ${PCL_LIBRARIES})

ament_target_dependencies(map_matcher_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

install(TARGETS map_matcher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
