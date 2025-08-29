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
find_package(rosidl_default_generators REQUIRED)
# ...existing code...
# ติดตั้ง Python script
install(PROGRAMS
  script/amr_web_server.py
  script/plc_communication.py
  script/camera_to_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SafetyConfig.srv"
   DEPENDENCIES std_msgs
)

# ติดตั้งโฟลเดอร์ web ไปยัง share path
install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

# ...existing code...

ament_package()
