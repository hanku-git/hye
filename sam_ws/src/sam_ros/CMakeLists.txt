cmake_minimum_required(VERSION 3.5)
project(sam_ros)

# find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs message_generation)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Result.msg"
  "srv/BoolSrv.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

install(PROGRAMS
  nodes/sam_node.py
  nodes/sam_node_hyundai.py
  nodes/sam2_node_hyundai.py
  nodes/bolt_classifier.py
  nodes/hinge_classifier.py
  models/model_128_bolt.py
  models/model_128_hinge.py
  models/model_hinge_angle.py
  models/model.py
  sam_util/utils.py
  sam_util/visualize.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  msg
  srv
  sam_util
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

