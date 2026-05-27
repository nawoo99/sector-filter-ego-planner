cmake_minimum_required(VERSION 3.8)
project(sector_filter)

# C++ 표준 (ROS2 Humble 권장: C++17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(pcl_conversions REQUIRED)

# PCL include/defs
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sector_filter_node src/sector_filter_node.cpp)
ament_target_dependencies(sector_filter_node
  rclcpp sensor_msgs std_msgs pcl_conversions
)
target_link_libraries(sector_filter_node ${PCL_LIBRARIES})

install(TARGETS sector_filter_node
  DESTINATION lib/${PROJECT_NAME})

# launch/config 폴더가 없으면 OPTIONAL로 무시
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
