cmake_minimum_required(VERSION 3.5)
project(sample_code LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grp_control_msg REQUIRED)
find_package(msg_srv_sample REQUIRED)

add_executable(sub_client subscribe_client.cpp)
ament_target_dependencies(sub_client rclcpp msg_srv_sample grp_control_msg)

add_executable(pub_server publish_server.cpp)
ament_target_dependencies(pub_server rclcpp msg_srv_sample grp_control_msg)

install(TARGETS
    sub_client
    pub_server
    DESTINATION lib/${PROJECT_NAME})

ament_package()