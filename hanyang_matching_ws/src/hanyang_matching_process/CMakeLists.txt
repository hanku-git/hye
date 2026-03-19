cmake_minimum_required(VERSION 3.5)
project(hanyang_matching_process)

# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ------------
# HELIOS
# ------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   roscpp
#   std_msgs
#   cv_bridge
#   image_transport
#   hanyang_matching_msgs
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED date_time)
find_package(image_transport REQUIRED) 
find_package(hanyang_matching_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(zivid_interfaces REQUIRED)
# find_package(arena_sdk REQUIRED)

find_package(VTK REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

message("VTK_FOUND: ${VTK_FOUND}")
message("VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")
message("VTK_LIBRARIES: ${VTK_LIBRARIES}")

#set(LPSOLVER_INCLUDE_DIRS /home/irl/ThirdParty/lp_solve_5.5.2.5_dev_ux64/include)
#set(LPSOLVER_LIBRARY_DIRS /home/irl/ThirdParty/lp_solve_5.5.2.5_dev_ux64)
#include_directories(${LPSOLVER_INCLUDE_DIRS})
#link_directories(${LPSOLVER_LIBRARY_DIRS})
#link_directories(/usr/local/lib/lp_solve)
#link_directories(/usr/local/lib)
#link_directories(/usr/lib)
#link_directories(/usr/lib/lp_solve)

# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12 /usr/local/include/pcl-1.12)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)
# set(PCL_LIBRARY_DIRS /usr/local/lib)

# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12 /usr/local/include/pcl-1.12)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)
# set(PCL_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu)

find_library(COLAMD_LIB colamd)
if (NOT COLAMD_LIB)
  message(FATAL_ERROR "Could not find colamd library")
else()
  message(STATUS "Found colamd: ${COLAMD_LIB}")
endif()

find_library(LPSOLVE_LIB lpsolve55)
if (NOT LPSOLVE_LIB)
  message(FATAL_ERROR "Could not find lpsolve55 library")
else()
  message(STATUS "Found lpsolve55: ${LPSOLVE_LIB}")
endif()

# include_directories(/opt/ros/humble/include)
include_directories(include)
include_directories(${IMAGE_TRANSPORT_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})
# link_directories(${OpenCV_LIBS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

# generate_messages(DEPENDENCIES std_msgs)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES hanyang_matching_process
#   CATKIN_DEPENDS std_msgs roscpp cv_bridge image_transport
#   DEPENDS Eigen

# )


file (GLOB ${PROJECT_NAME}_SRCS
  src/*.cpp
)

set(SOURCE1
  src/module/KUPCLMath.cpp
  src/module/image2cloud_processing.cpp
)

set(HEADER1
  include/module/KUPCLMath.h
  include/module/image2cloud_processing.h
)

set(SOURCE2
  src/module/path_planning.cpp
  src/module/template_matching.cpp
  src/module/pcl_processing.cpp
  src/module/image2cloud_processing.cpp
  src/module/optimization_module.cpp
  src/module/KUPCLMath.cpp
  src/control/RobotModel.cpp
  src/state_estimation/contact_state_estimator.cpp
  src/pbPlots/pbPlots.cpp
  src/pbPlots/supportLib.cpp
)

set(HEADER2
  include/module/data_class/object_data.h
  include/module/data_class/planning_data.h
  include/module/data_class/optimization_data.h
  include/module/path_planning.h
  include/module/pcl_processing.h
  include/module/KUPCLMath.h
  include/module/image2cloud_processing.h
  include/module/optimization_module.h
  include/module/template_matching.h
  include/control/RobotModel.hpp
  include/state_estimation/contact_state_estimator.h
  include/pbPlots/pbPlots.hpp
  include/pbPlots/supportLib.hpp
  include/define.h
  include/precompiled_header.h
)

add_executable(zivid_scan_node src/zivid_scan_node.cpp ${HEADER1} ${SOURCE1})
target_link_libraries(zivid_scan_node ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(zivid_scan_node 
  rclcpp 
  image_transport
  hanyang_matching_msgs 
  pcl_ros
  # arena_sdk
  pcl_msgs
  std_msgs
  std_srvs
  sensor_msgs
  # zivid_interfaces
  cv_bridge)

add_executable(matching_node src/matching_node.cpp ${HEADER2} ${SOURCE2})
target_link_libraries(matching_node
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${LPSOLVE_LIB}
  superlu
  /usr/lib/x86_64-linux-gnu/libcolamd.so 
)

ament_target_dependencies(matching_node 
  rclcpp 
  image_transport
  hanyang_matching_msgs 
  pcl_ros
  pcl_msgs
  std_msgs
  std_srvs
  # zivid_interfaces
  sensor_msgs
  cv_bridge
  )

# add_executable(helios_pub_node src/helios_pub_node.cpp)
# target_link_libraries(helios_pub_node ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
# ament_target_dependencies(helios_pub_node 
#   rclcpp 
#   arena_sdk
#   sensor_msgs
#   cv_bridge)

install(TARGETS
  zivid_scan_node
  matching_node
  # helios_pub_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()