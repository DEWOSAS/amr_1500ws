cmake_minimum_required(VERSION 3.8)
project(facobot_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(facobot_msg REQUIRED)  
find_package(geometry_msgs REQUIRED)

ament_package()
