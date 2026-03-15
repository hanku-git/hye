cmake_minimum_required(VERSION 3.5)
project(hanyang_matching_visualizer)

# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   roscpp
#   std_msgs
#   cv_bridge
#   image_transport
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PCL 1.12 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

# set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.12)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBS})
add_definitions(${PCL_DEFINITIONS})

# generate_messages(DEPENDENCIES std_msgs)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES hanyang_matching_visualizer
#   CATKIN_DEPENDS std_msgs roscpp cv_bridge image_transport
#   DEPENDS Eigen

# )

include_directories(
  include
)

# ament_target_dependencies(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
#   rclcpp
#   std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
