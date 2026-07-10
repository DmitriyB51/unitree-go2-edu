cmake_minimum_required(VERSION 3.16)
project(plio2sm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-poses REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

add_executable(plio2sm src/plio2sm.cpp)

ament_target_dependencies(plio2sm
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  nav_msgs
  sensor_msgs
)

target_link_libraries(plio2sm
  mrpt::maps
  mrpt::obs
  mrpt::poses
  mrpt::ros2bridge
)

install(TARGETS plio2sm DESTINATION lib/${PROJECT_NAME})

ament_package()
