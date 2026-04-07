cmake_minimum_required(VERSION 3.16)
project(koras_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
# Isaac Sim Python 3.11 명시
set(Python3_EXECUTABLE /home/hanku/isaacsim/kit/python/bin/python3.11)
set(Python3_INCLUDE_DIR /home/hanku/isaacsim/kit/python/include/python3.11)
set(Python3_LIBRARY /home/hanku/isaacsim/kit/python/lib/libpython3.11.so)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# pybind11 (Isaac Sim Python 3.11 pip 설치버전)
set(pybind11_DIR /home/hanku/isaacsim/kit/python/lib/python3.11/site-packages/pybind11/share/cmake/pybind11)
find_package(pybind11 REQUIRED CONFIG)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Include paths: our include/ first (overrides robot_define.hpp)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SRC_DIR}
    ${JSONCPP_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
)

pybind11_add_module(koras_kinematics
    ${SRC_DIR}/bindings.cpp
    ${SRC_DIR}/model_matrix.cpp
    ${SRC_DIR}/robot_parameter.cpp
    ${SRC_DIR}/robot_model.cpp
    ${SRC_DIR}/dynamics_function.cpp
)

target_link_libraries(koras_kinematics PRIVATE ${JSONCPP_LIBRARIES})

# Install .so to koras_kinematics prefix (colcon 관리)
install(TARGETS koras_kinematics
    LIBRARY DESTINATION lib/python3.11/site-packages/dsr_isaac_bridge
)

# dsr_isaac_bridge install 경로에도 복사 (run_bridge에서 직접 import 가능)
install(TARGETS koras_kinematics
    LIBRARY DESTINATION ../dsr_isaac_bridge/lib/python3.11/site-packages/dsr_isaac_bridge
)

ament_package()
