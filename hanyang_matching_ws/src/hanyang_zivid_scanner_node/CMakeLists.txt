cmake_minimum_required(VERSION 3.5)
project(hanyang_zivid_scanner_node)

# find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs message_generation)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_numpy REQUIRED)


# rosidl_generate_interfaces(${PROJECT_NAME}
#   DEPENDENCIES std_msgs sensor_msgs
# )

install(PROGRAMS
  src/zivid_node.py
  src/zivid_topic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

