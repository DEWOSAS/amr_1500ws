cmake_minimum_required(VERSION 3.8)
project(map_to_jpeg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(map_to_image_node src/map_to_image_node.cpp)
ament_target_dependencies(map_to_image_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  OpenCV
  tf2_geometry_msgs 
)

add_executable(image_to_map_node src/image_to_map_node.cpp)
ament_target_dependencies(image_to_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  OpenCV
  tf2_geometry_msgs 
)

add_executable(map_to_image_node_ori src/map_to_image_node_ori.cpp)
ament_target_dependencies(map_to_image_node_ori
   rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)


install(TARGETS
  map_to_image_node
  image_to_map_node
  map_to_image_node_ori
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()