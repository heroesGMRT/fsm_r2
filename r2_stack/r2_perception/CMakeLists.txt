cmake_minimum_required(VERSION 3.8)
project(r2_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(r2_interfaces REQUIRED)

add_executable(kfs_localizer_node src/kfs_localizer_node.cpp)
target_include_directories(kfs_localizer_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(kfs_localizer_node ${PCL_LIBRARIES})
ament_target_dependencies(kfs_localizer_node
  rclcpp rclcpp_action sensor_msgs geometry_msgs
  pcl_conversions r2_interfaces
)

install(TARGETS kfs_localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
