set(PC_LABEL "2L082L24CALJ")
set(PRODUCT_NAME "kr_sys")
set(PRODUCT_VERSION "2.0.1")
set(SSD_SERIAL_NUMBER "2L082L24CALJ")
set(PRODUCT_SERIAL_KEY "9OXR72639SIZ0UF9")

set(USERNAME $ENV{USER})  # 현재 로그인된 사용자 이름 가져오기 (Linux/macOS)

cmake_minimum_required(VERSION 3.8)
# cmake_policy(SET CMP0091 NEW)
# cmake_policy(SET CMP0077 NEW)
# cmake_policy(SET CMP0072 NEW) # new in 3.11. The NEW behavior for this policy is to set OpenGL_GL_PREFERENCE to GLVND.
# cmake_policy(SET CMP0068 NEW) # new in 3.9. The NEW behavior of this policy is to ignore the RPATH settings for install_name on macOS.

project(hanyang_eng_koras_system)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message(STATUS "ccache found and enabled")
    set(CMAKE_C_COMPILER_LAUNCHER ccache)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
endif()

# Default to C++17 or 20
if(NOT CMAKE_CXX_STANDARD)
  #set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CALIBRATION_SKIP ON)

# Pass the variable to the source file using add_definitions

if(CALIBRATION_SKIP)
  add_definitions(-DCALIBRATION_SKIP)
endif()

#add_definitions(-DQT_NO_KEYWORDS)

# # Python 관련 코드에서만 `QT_NO_KEYWORDS` 적용
# set_source_files_properties(
#   include/qnode.hpp
#   PROPERTIES COMPILE_DEFINITIONS QT_NO_KEYWORDS
# )


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# set(dsr_msgs2_DIR "/home/${USERNAME}/doosan_ros2_ws/install/dsr_msgs2/share/dsr_msgs2/cmake")

# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
# set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
# set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
# set(CMAKE_DISABLE_SOURCE_CHANGES  ON)
# set(OpenGL_GL_PREFERENCE LEGACY)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(BUILD_SHARED_LIBS "Build as shared library" OFF)
# set(CMAKE_FIND_LIBRARY_SUFFIXES .lib)

#get_directory_property(_has_parent PARENT_DIRECTORY)
#if(_has_parent)
  #set(is_root_project OFF)
#else()
  #set(is_root_project ON)
#endif()

# set(QT_NODES_DEVELOPER_DEFAULTS "${is_root_project}" CACHE BOOL "Turns on default settings for development of QtNodes")

# option(BUILD_TESTING "Build tests" "${QT_NODES_DEVELOPER_DEFAULTS}")
# option(BUILD_EXAMPLES "Build Examples" "${QT_NODES_DEVELOPER_DEFAULTS}")
# option(BUILD_DOCS "Build Documentation" "${QT_NODES_DEVELOPER_DEFAULTS}")
# option(BUILD_SHARED_LIBS "Build as shared library" ON)
# option(BUILD_DEBUG_POSTFIX_D "Append d suffix to debug libraries" OFF)
# option(QT_NODES_FORCE_TEST_COLOR "Force colorized unit test output" OFF)

# enable_testing()

# if(QT_NODES_DEVELOPER_DEFAULTS)
#   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
#   set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
#   set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
# endif()

# if(BUILD_DEBUG_POSTFIX_D)
#   set(CMAKE_DEBUG_POSTFIX "d")
#   set(CMAKE_RELEASE_POSTFIX "")
#   set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
#   set(CMAKE_MINSIZEREL_POSTFIX "s")
# endif()

#cmath pi for windows
# if(MSVC)
#    add_definitions(-D_USE_MATH_DEFINES)
# endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kcr_control_msg REQUIRED)
find_package(grp_control_msg REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hanyang_matching_msgs REQUIRED)
# find_package(llm_msgs REQUIRED)
find_package(PkgConfig REQUIRED)


pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenSSL REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
#add_subdirectory(external)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
  # 불필요한 검사 비활성화
  set(AMENT_LINT_AUTO_EXCLUDE "ament_uncrustify")
endif()

#set(Qt_DIR "C:/QT6/6.4.3/Src")6.4.3\mingw_64
#set(CMAKE_PREFIX_PATH C:/QT6/6.4.3/mingw_64)
#if("${CMAKE_CXX_SIMULATE_ID}" STREQUAL "MSVC")
#if(MSVC)
#  set(QT_DIR "C:/QT6/6.4.3/msvc2019_64/lib/cmake/Qt6")
#endif()
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets UiTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets Gui OpenGL UiTools)

add_definitions(${Qt5_DEFINITIONS})

message(STATUS "QT_VERSION: ${QT_VERSION}, QT_DIR: ${QT_DIR}")
if (${QT_VERSION} VERSION_LESS 5.11.0)
  message(FATAL_ERROR "Requires qt version >= 5.11.0, Your current version is ${QT_VERSION}")
endif()

if (${QT_VERSION_MAJOR} EQUAL 6)
  qt_add_resources(RESOURCES ./resources/resources.qrc)
else()
  qt5_add_resources(RESOURCES ./resources/resources.qrc)
endif()

# Unfortunately, as we have a split include/src, AUTOMOC doesn't work.
# We'll have to manually specify some files

set(CPP_SOURCE_FILES
  src/QtNodes/AbstractGraphModel.cpp
  src/QtNodes/AbstractNodeGeometry.cpp
  src/QtNodes/BasicGraphicsScene.cpp
  src/QtNodes/ConnectionGraphicsObject.cpp
  src/QtNodes/ConnectionPainter.cpp
  src/QtNodes/ConnectionState.cpp
  src/QtNodes/ConnectionStyle.cpp
  src/QtNodes/DataFlowGraphModel.cpp
  src/QtNodes/DataFlowGraphicsScene.cpp
  src/QtNodes/DefaultHorizontalNodeGeometry.cpp
  src/QtNodes/DefaultVerticalNodeGeometry.cpp
  src/QtNodes/Definitions.cpp
  src/QtNodes/GraphicsView.cpp
  src/QtNodes/GraphicsViewStyle.cpp
  src/QtNodes/NodeDelegateModelRegistry.cpp
  src/QtNodes/NodeConnectionInteraction.cpp
  src/QtNodes/NodeDelegateModel.cpp
  src/QtNodes/NodeGraphicsObject.cpp
  src/QtNodes/DefaultNodePainter.cpp
  src/QtNodes/NodeState.cpp
  src/QtNodes/NodeStyle.cpp
  src/QtNodes/StyleCollection.cpp
  src/QtNodes/UndoCommands.cpp
  src/QtNodes/locateNode.cpp
)

set(HPP_HEADER_FILES
  include/QtNodes/internal/AbstractGraphModel.hpp
  include/QtNodes/internal/AbstractNodeGeometry.hpp
  include/QtNodes/internal/AbstractNodePainter.hpp
  include/QtNodes/internal/BasicGraphicsScene.hpp
  include/QtNodes/internal/Compiler.hpp
  include/QtNodes/internal/ConnectionGraphicsObject.hpp
  include/QtNodes/internal/ConnectionIdHash.hpp
  include/QtNodes/internal/ConnectionIdUtils.hpp
  include/QtNodes/internal/ConnectionState.hpp
  include/QtNodes/internal/ConnectionStyle.hpp
  include/QtNodes/internal/DataFlowGraphicsScene.hpp
  include/QtNodes/internal/DataFlowGraphModel.hpp
  include/QtNodes/internal/DefaultNodePainter.hpp
  include/QtNodes/internal/Definitions.hpp
  include/QtNodes/internal/Export.hpp
  include/QtNodes/internal/GraphicsView.hpp
  include/QtNodes/internal/GraphicsViewStyle.hpp
  include/QtNodes/internal/locateNode.hpp
  include/QtNodes/internal/NodeData.hpp
  include/QtNodes/internal/NodeDelegateModel.hpp
  include/QtNodes/internal/NodeDelegateModelRegistry.hpp
  include/QtNodes/internal/NodeGraphicsObject.hpp
  include/QtNodes/internal/NodeState.hpp
  include/QtNodes/internal/NodeStyle.hpp
  include/QtNodes/internal/OperatingSystem.hpp
  include/QtNodes/internal/QStringStdHash.hpp
  include/QtNodes/internal/QUuidStdHash.hpp
  include/QtNodes/internal/Serializable.hpp
  include/QtNodes/internal/Style.hpp
  include/QtNodes/internal/StyleCollection.hpp
  src/QtNodes/ConnectionPainter.hpp
  src/QtNodes/DefaultHorizontalNodeGeometry.hpp
  src/QtNodes/DefaultVerticalNodeGeometry.hpp
  src/QtNodes/NodeConnectionInteraction.hpp
  src/QtNodes/UndoCommands.hpp
)

# If we want to give the option to build a static library,
#set BUILD_SHARED_LIBS option to OFF
#option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

add_library(QtNodes
  ${CPP_SOURCE_FILES}
  ${HPP_HEADER_FILES}
  ${RESOURCES}
)

add_library(QtNodes::QtNodes ALIAS QtNodes)

# if(MSVC)        # Microsoft Visual C++ Compiler
#   set_property(TARGET QtNodes PROPERTY
#   MSVC_RUNTIME_LIBRARY "MultiThreaded")
# endif()

target_include_directories(QtNodes
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/QtNodes/internal>
)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(vcpkg_install_dir $ENV{VCPKG_ROOT})
endif()


target_link_libraries(QtNodes
  PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::UiTools
    #rviz_common::rviz_common
    #rviz_rendering::rviz_rendering
)

target_compile_definitions(QtNodes INTERFACE
  PUBLIC
    NODE_EDITOR_SHARED
    #NODE_EDITOR_STATIC
  PRIVATE
    NODE_EDITOR_EXPORTS
    #NODE_DEBUG_DRAWING
    #QT_NO_KEYWORDS
)

target_compile_options(QtNodes
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /wd4127 /EHsc /utf-8>
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wall -Wextra -Werror>
)
if(NOT "${CMAKE_CXX_SIMULATE_ID}" STREQUAL "MSVC")
  # Clang-Cl on MSVC identifies as "Clang" but behaves more like MSVC:
  target_compile_options(QtNodes
    PRIVATE
      $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
  )
endif()
if(MSVC)
    add_compile_options(/Zc:__cplusplus)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
    set_property(TARGET QtNodes PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")
endif()

if(QT_NODES_DEVELOPER_DEFAULTS)
  target_compile_features(QtNodes PUBLIC cxx_std_17)
  set_target_properties(QtNodes PROPERTIES CXX_EXTENSIONS OFF)
endif()

set_target_properties(QtNodes
  PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

######
# Moc
##

file(GLOB_RECURSE HEADERS_TO_MOC include/QtNodes/internal/*.hpp)

if (${QT_VERSION_MAJOR} EQUAL 6)
  qt_wrap_cpp(nodes_moc
      ${HEADERS_TO_MOC}
      TARGET QtNodes
    OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
  )
else()
  qt5_wrap_cpp(nodes_moc
  ${HEADERS_TO_MOC}
  TARGET QtNodes
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
  )
endif()

target_sources(QtNodes PRIVATE ${nodes_moc})

###########
# Examples
##

#if(BUILD_EXAMPLES)
#  add_subdirectory(examples)
#endif()

#if(BUILD_DOCS)
#  add_subdirectory(docs)
#endif()

##################
# Automated Tests
##

if(BUILD_TESTING)
  #add_subdirectory(test)
endif()

###############
# Installation
##

include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/QtNodes)

install(TARGETS QtNodes
  EXPORT QtNodesTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT QtNodesTargets
  FILE QtNodesTargets.cmake
  NAMESPACE QtNodes::
  DESTINATION ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/QtNodesConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/QtNodesConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/QtNodesConfig.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

########################################################################################################################
# kr_sys excutable definition
##

# public.pem 파일이 존재하는지 확인
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/public.pem")
    message(FATAL_ERROR "public.pem not found. Please provide a public key file for verification.")
endif()

# 디스크 용량을 확인하고 설치 디스크를 선택하는 스크립트
# execute_process(
#     COMMAND bash -c "
#     largest_disk=$(lsblk -ndo NAME,SIZE,TYPE | grep -w disk | sort -k2 -h | tail -n1 | awk '{print \"/dev/\" $1}') &&
#     disk_size=$(lsblk -ndbo SIZE -e 11 $largest_disk | awk '{print $1}') &&
#     min_size=$((1 * 1024 * 1024 * 1024)) && # 1GB minimum size
#     if [ $disk_size -ge $min_size ]; then
#         echo $largest_disk
#     else
#         echo ''
#     fi
#     "
#     OUTPUT_VARIABLE INSTALL_DISK
#     OUTPUT_STRIP_TRAILING_WHITESPACE
#     RESULT_VARIABLE SCRIPT_RESULT
# )

# if(NOT SCRIPT_RESULT EQUAL 0 OR INSTALL_DISK STREQUAL "")
#     message(FATAL_ERROR "No suitable disk found or failed to determine disk size.")
# endif()

# message("Selected disk for installation: ${INSTALL_DISK}")

# 시리얼 넘버 추출 스크립트 실행
# execute_process(
#    COMMAND bash -c "
#    serial=$(udevadm info --query=all --name=${INSTALL_DISK} | grep 'ID_SERIAL_SHORT' | awk -F= '{print $2}') &&
#    echo $serial
#    "
#    OUTPUT_VARIABLE EXPECTED_SERIAL
#    OUTPUT_STRIP_TRAILING_WHITESPACE
#    RESULT_VARIABLE SCRIPT_RESULT
#)

# PC 라벨 정보를 소스 코드 파일에 저장
set(PC_LABEL_CPP_PATH "${CMAKE_SOURCE_DIR}/src/nocopy/pc_label.cpp")
message("Creating pc_label.cpp at ${PC_LABEL_CPP_PATH}")


if(EXISTS ${PC_LABEL_CPP_PATH})
    message(WARNING "File ${PC_LABEL_CPP_PATH} already exists and will be overwritten.")
endif()

file(WRITE ${PC_LABEL_CPP_PATH} "#include \"pc_label.h\"\n#include <string>\nconst std::string pc_label = \"${PC_LABEL}\";\n")


if(SSD_SERIAL_NUMBER STREQUAL "")
    message(FATAL_ERROR "Failed to extract serial number: ${SSD_SERIAL_NUMBER}")
endif()

message("apply ssd serial number: ${SSD_SERIAL_NUMBER}")

# SSD 시리얼 넘버를 소스 코드 파일에 저장
set(SERIAL_NUMBER_CPP_PATH "${CMAKE_SOURCE_DIR}/src/nocopy/serial_number.cpp")
message("Creating serial_number.cpp at ${SERIAL_NUMBER_CPP_PATH}")


if(EXISTS ${SERIAL_NUMBER_CPP_PATH})
    message(WARNING "File ${SERIAL_NUMBER_CPP_PATH} already exists and will be overwritten.")
endif()

file(WRITE ${SERIAL_NUMBER_CPP_PATH} "#include \"serial_number.h\"\n#include <string>\nconst std::string expected_serial = \"${SSD_SERIAL_NUMBER}\";\n")

# 제품키 저장
set(PRODUCT_KEY_CPP_PATH "${CMAKE_SOURCE_DIR}/src/nocopy/product_key.cpp")
message("Creating product_key.cpp at ${PRODUCT_KEY_CPP_PATH}")


if(EXISTS ${PRODUCT_KEY_CPP_PATH})
    message(WARNING "File ${PRODUCT_KEY_CPP_PATH} already exists and will be overwritten.")
endif()

file(WRITE ${PRODUCT_KEY_CPP_PATH} "#include \"product_key.h\"\n#include <string>\nconst std::string product_key = \"${PRODUCT_SERIAL_KEY}\";\n")

# 환경 변수 설정
set(ENV{SECUREAPP_DATA_PATH} "${CMAKE_BINARY_DIR}/${PRODUCT_NAME}")
set(ENV{SECUREAPP_SIGNATURE_PATH} "${CMAKE_BINARY_DIR}/example.sig")
set(ENV{SECUREAPP_PUBLIC_KEY_PATH} "${CMAKE_BINARY_DIR}/public.pem")

message("Environment variables set:")
message("  SECUREAPP_DATA_PATH=$ENV{SECUREAPP_DATA_PATH}")
message("  SECUREAPP_SIGNATURE_PATH=$ENV{SECUREAPP_SIGNATURE_PATH}")
message("  SECUREAPP_PUBLIC_KEY_PATH=$ENV{SECUREAPP_PUBLIC_KEY_PATH}")


# option(DRFL_CONTROL "Enable DRFL control" OFF)
set(DRFL_LIB_DIR ${PROJECT_SOURCE_DIR}/include/libdrfl/lib/libdrfl)
set(DRFL_CONTROL TRUE)
set(DRFL_ROS2 TRUE)
message(STATUS "DRFL_CONTROL: ${DRFL_CONTROL}")
if(DRFL_CONTROL)
  add_definitions(-DDRFL_CONTROL)
  include_directories(
    ${PROJECT_SOURCE_DIR}/include/libdrfl/include
    )
    message(STATUS "FAIRINO_LIB_DIR: ${FAIRINO_LIB_DIR}")
endif()


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config

  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/bin_picking
  ${PROJECT_SOURCE_DIR}/include/bin_picking/data_class
  ${PROJECT_SOURCE_DIR}/include/bin_picking/dialog
  ${PROJECT_SOURCE_DIR}/include/bin_picking/math
  ${PROJECT_SOURCE_DIR}/include/bin_picking/nlohmann
  ${PROJECT_SOURCE_DIR}/include/python_ui_node
  ${PROJECT_SOURCE_DIR}/include/modbus

  ${PROJECT_SOURCE_DIR}/include/task
  ${PROJECT_SOURCE_DIR}/include/node_model
  ${PROJECT_SOURCE_DIR}/include/node_model/data_type

  ${PROJECT_SOURCE_DIR}/include/custom_widget

  ${PROJECT_SOURCE_DIR}/include/nocopy
#   ${PROJECT_SOURCE_DIR}/include/llm_control
#   ${PROJECT_SOURCE_DIR}/include/llm_control/dialog

  ${PROJECT_SOURCE_DIR}/ui
  ${rviz_common_INCLUDE_DIR}

  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/sqlite3

  ${PROJECT_SOURCE_DIR}/include/food
  ${PROJECT_SOURCE_DIR}/include/tag
)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/ui)

if(CALIBRATION_SKIP)
else()
  include_directories(
  ${PROJECT_SOURCE_DIR}/include/robot_calibration
  ${PROJECT_SOURCE_DIR}/include/robot_calibration/control
  ${PROJECT_SOURCE_DIR}/include/robot_calibration/dialog
  )
endif()

file (GLOB ${PROJECT_NAME}_SRCS
  ui/*.ui
  src/*.cpp
)

file (GLOB CALIBRATION_SRCS
  src/robot_calibration/*.cpp
  src/robot_calibration/control/*.cpp
  src/robot_calibration/dialog/*.cpp
  include/robot_calibration/control/*.hpp
  include/robot_calibration/*.hpp
  include/robot_calibration/*.h
  include/robot_calibration/dialog/*.hpp
  include/robot_calibration/dialog/*.h
)

file (GLOB BINPICKING_SRCS

  src/*.cpp
  src/bin_picking/*.cpp
  src/bin_picking/dialog/*.cpp
  src/bin_picking/math/*.cpp
  src/python_ui_node/*.cpp
  src/food/*.cpp
  src/tag/*.cpp
  src/modbus/*.cpp
  src/node_model/*.cpp
  src/task/*.cpp

  src/custom_widget/*.cpp

  src/nocopy/*.cpp

  include/*.hpp

  include/bin_picking/*.hpp
  include/bin_picking/data_class/*.hpp
  include/bin_picking/dialog/*.hpp
  include/bin_picking/math/*.hpp
  include/python_ui_node/*.hpp
  include/food/*.hpp
  include/tag/*.hpp
  include/modbus/*.hpp

  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner.cpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner.hpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_hyundai_llm.cpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_hyundai_llm.hpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_cooking.cpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_cooking.hpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_hanyang_eng.cpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner_hanyang_eng.hpp
  ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/ui_define.hpp

  include/node_model/*.hpp
  include/task/*.hpp

  include/custom_widget/*.hpp


  ${PROJECT_SOURCE_DIR}/sqlite3/sqlite3.h
  include/nocopy/*.h
)

SET(CALIBRATION_UI_TARGETS
  include/robot_calibration/dialog/cal_dialog.hpp
)

#FOR MOC AND UIC
SET(BINP_UI_TARGETS
  include/mainwindow_custom.hpp
  include/mainwindow_control.hpp
  include/mainwindow_gripper.hpp
  include/mainwindow_initial_setup.hpp
  include/mainwindow_widgets_monitoring.hpp
  include/mainwindow_widgets_hyundai_llm.hpp
  include/mainwindow_widgets_koras_cooking.hpp
  include/mainwindow_widgets_hanyang_eng.hpp
  include/mainwindow_widgets_camera_view.hpp
  include/mainwindow_widgets_launch_package.hpp
  include/mainwindow_widgets_hanyang_launch_package.hpp
  include/mainwindow_widgets_robot_handling.hpp
  include/bin_picking/bin_picking_node.hpp
  include/bin_picking/dialog/bin_matching_dialog.hpp
  include/bin_picking/dialog/bin_picking_dialog.hpp
  include/bin_picking/dialog/robot2camera_calibration_dialog.hpp
  # include/bin_picking/dialog/scanning_dialog.hpp
  include/python_ui_node/py_ui_bridge_node.hpp
)

# set(UI_CONFIG_SRCS
#   ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner.cpp
#   ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/task_planner.hpp
#   ${PROJECT_SOURCE_DIR}/../robot_control_config/ui_config/ui_define.hpp
# )

SET(CALIBRATION_MOC_TARGETS
  include/robot_calibration/dialog/cal_dialog.hpp
)

set(BINP_MOC_TARGETS
  include/mainwindow_custom.hpp
  include/mainwindow_node.hpp
  include/mainwindow_control.hpp
  include/mainwindow_gripper.hpp
  include/mainwindow_initial_setup.hpp
  include/mainwindow_widgets_monitoring.hpp
  include/mainwindow_widgets_hyundai_llm.hpp
  include/mainwindow_widgets_koras_cooking.hpp
  include/mainwindow_widgets_hanyang_eng.hpp
  include/mainwindow_widgets_camera_view.hpp
  include/mainwindow_widgets_launch_package.hpp
  include/mainwindow_widgets_hanyang_launch_package.hpp
  include/mainwindow_widgets_robot_handling.hpp
  
  #include/widget_robot_state.hpp

  include/qnode.hpp

  include/task/taskManager.hpp

  include/node_model/TaskStartModel.hpp
  include/node_model/KCRCommandModel.hpp
  include/node_model/BPCommandModel.hpp
  include/node_model/PLCCommandModel.hpp
  include/node_model/data_type/doubleData.hpp

  include/bin_picking/bin_picking_node.hpp
  include/bin_picking/dialog/bin_matching_dialog.hpp
  include/bin_picking/dialog/bin_picking_dialog.hpp
  include/bin_picking/dialog/robot2camera_calibration_dialog.hpp

#   include/llm_control/dialog/llm_dialog.hpp
  # include/bin_picking/dialog/scanning_dialog.hpp

  include/python_ui_node/py_ui_bridge_node.hpp

  include/custom_widget/toggle_widget.hpp
  include/food/food_ai_dialog.hpp
  include/tag/food_tag_dialog.hpp
  include/cooing_automation/qnode_cooking.hpp
)

#file(GLOB MY_FILES "images/*.png")
#file(COPY ${MY_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Generate code from ui files

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOMOC_MOC_OPTIONS -DQT_NO_KEYWORDS)
#set_source_files_properties(taskScriptBuilder.hpp PROPERTIES SKIP_AUTOMOC false)

# Set the source directory
set(JSONCPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/INCLUDE)
# Recursively collect all .hpp files
file(GLOB_RECURSE JSONCPP_HEADERS ${JSONCPP_DIR}/*.hpp)

# add Sqlite Object to library
add_library(sqlite_object OBJECT IMPORTED)

set_property(TARGET sqlite_object PROPERTY IMPORTED_OBJECTS
  # Quotes are required because of the ()
  "${PROJECT_SOURCE_DIR}/sqlite3/sqlite3.o"
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/sqlite3/sqlite3
  DESTINATION bin
)

#Pybind 11
execute_process(
    COMMAND ${Python_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED CONFIG)
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
#Pybind 11

#lua script engine
set(LUA_LIB_DIR ${PROJECT_SOURCE_DIR}/include/liblua/lib/liblua)
include_directories(${PROJECT_SOURCE_DIR}/include/liblua/include/liblua)
include_directories(${PROJECT_SOURCE_DIR}/include/liblua/include)

include_directories(
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

if(CALIBRATION_SKIP)
  add_executable(${PRODUCT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${BINPICKING_SRCS}
  ${BINP_UI_TARGETS}
  #   ${LLM_SRCS}
  #   ${LLM_UI_TARGETS}
  $<TARGET_OBJECTS:sqlite_object>
  )
else()
  add_executable(${PRODUCT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${BINPICKING_SRCS}
  ${BINP_UI_TARGETS}
  ${CALIBRATION_SRCS}
  ${CALIBRATION_UI_TARGETS}
#   ${LLM_SRCS}
#   ${LLM_UI_TARGETS}
  $<TARGET_OBJECTS:sqlite_object>
  )
endif()


set(ROS2_DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "kcr_control_msg"
  "dsr_msgs2"
  "grp_control_msg"
  "hanyang_matching_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "urdf"
  "rviz_common"
  "rviz_default_plugins"
  "rviz_ogre_vendor"
  "rviz_rendering"
  "rviz_default_plugins"
  "OpenCV"
  "cv_bridge"
)
set(ROS2_DEPENDENCIES_FOR_WIN
  "kcr_control_msg"
  "dsr_msgs2"
  "grp_control_msg"
  "hanyang_matching_msgs"
)

if(MSVC)
  target_link_libraries(${PRODUCT_NAME}
    QtNodes
    ${JSONCPP_LIBRARY}
    ${STA_LIBS}
    Ws2_32.lib
    )
    ament_target_dependencies(${PRODUCT_NAME} ${ROS2_DEPENDENCIES_FOR_WIN})
else()
  target_link_libraries(${PRODUCT_NAME}
    QtNodes
    ${JSONCPP_LIBRARIES}
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
    ${OpenCV_LIBS}
    ${sensor_msgs_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    OpenSSL::SSL
    OpenSSL::Crypto
    pybind11::embed
    ${PYTHON_LIBRARIES}

#    ${DRFL_LIB_DIR}/libDRFL.a
#    ${DRFL_LIB_DIR}/libPocoFoundation.so.80
#    ${DRFL_LIB_DIR}/libPocoNet.so.80

    ${LUA_LIB_DIR}/liblua.a

    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
  )
  ament_target_dependencies(${PRODUCT_NAME} ${ROS2_DEPENDENCIES})
endif()
#install(DIRECTORY ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT headers)

# 라이브러리 설치 및 심볼릭 링크 설정
set(DRFL_LIB_DIR /home/${USERNAME}/ws/hye_ws/robot_a/main01/doosan_ros2_ws/src/doosan-robot2/common2)
install(FILES
${DRFL_LIB_DIR}/lib/humble/x86_64/libDRFL.a
${DRFL_LIB_DIR}/include/DRFL.h
DESTINATION lib
)

install(FILES
${LUA_LIB_DIR}/liblua.a
DESTINATION lib
)

if(NOT DEFINED ENV{QT_QPA_PLATFORM_PLUGIN_PATH})
  set(ENV{QT_QPA_PLATFORM_PLUGIN_PATH} ${CMAKE_CURRENT_BINARY_DIR}/qtin)
endif()

target_compile_options(${PRODUCT_NAME}
  PUBLIC
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /wd4127 /EHsc /utf-8>
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wall -Wextra -Werror>
)
if(NOT "${CMAKE_CXX_SIMULATE_ID}" STREQUAL "MSVC")
  # Clang-Cl on MSVC identifies as "Clang" but behaves more like MSVC:
  target_compile_options(${PRODUCT_NAME}
    PUBLIC
      $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
  )
endif()
if(MSVC)
    add_compile_options(/Zc:__cplusplus)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

if(binpicking_DEVELOPER_DEFAULTS)
  target_compile_features(${PRODUCT_NAME} PUBLIC cxx_std_17)
  set_target_properties(${PRODUCT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
endif()


#########################################################

# Install launch files.
#set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PRODUCT_NAME})

install(TARGETS ${PRODUCT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

if(MSVC)
  install(FILES ${STA_LIBS}
          DESTINATION bin
  )
  install(FILES ${JSONCPP_LIBRARY}
          DESTINATION lib
  )
  install(FILES ${QT_PLUGINS}
        DESTINATION lib/qtin
  )
endif(MSVC)

add_custom_command(TARGET ${PRODUCT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PROJECT_SOURCE_DIR}/resources
  ${PROJECT_BINARY_DIR}/resources
  COMMENT "copying resources into binary directory")

# ssd 복사 방지 관련 파일 설치
# public.pem 파일을 빌드 디렉터리로 복사
add_custom_command(
    TARGET ${PRODUCT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/public.pem ${CMAKE_BINARY_DIR}/public.pem
)

# postinst script 및 extract_serial.sh 설치
install(PROGRAMS scripts/postinst
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/extract_serial.sh
  DESTINATION lib/${PROJECT_NAME})

install(FILES public.pem
  DESTINATION share/${PROJECT_NAME})

ament_package()
