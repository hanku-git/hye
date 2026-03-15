cmake_minimum_required(VERSION 3.5)
project(bin_picking_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(catkin REQUIRED COMPONENTS
#   std_msgs 
#   message_generation
#   geometry_msgs
#   sensor_msgs
# )

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

# add_message_files(
#   FILES
#   RobotState.msg
#   GripperCmd.msg
#   DetectedPose.msg
#   Result_mrcnn.msg  
#   MaskCloud.msg
#   BoundingBox.msg
#   BoundingBoxes.msg
#   BpCommand.msg
#   BpUiCommandLearning.msg
#   gPos.msg
# )

# add_service_files(
#   FILES
#   Capture.srv
#   CaptureAssistantSuggestSettings.srv
#   SetTargetJointPosition.srv
#   SetTargetPose.srv
#   PartScanning.srv
#   PartDetection.srv
#   GraspingPose.srv
#   grpPose.srv
#   JogCommand.srv
#   StopRobot.srv
#   Relay.srv
#   SetCommand.srv
#   ImpedanceControl.srv
#   ZividDoScan.srv
#   DoTemplateMatching.srv
#   DoPoseEstimation.srv
#   DoTaskRecognition.srv
#   RSGetImage.srv
#   RLAlign.srv
#   RLInsert.srv
#   RLInsertAction.srv
#   BoolSrv.srv
#   DoPCLTest.srv
#   InsertdoneMask.srv
#   doneMask.srv
# )
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/GripperCmd.msg"
  "msg/DetectedPose.msg"
  "msg/Resultmrcnn.msg" 
  "msg/Resultsam.msg"
  "msg/MaskCloud.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/BpUiCommand.msg"
  "msg/BpUiCommandLearning.msg"
  "msg/GripperMsg.msg"
  "msg/DoTemplateMatchingMsg.msg"
  "msg/MatchingResultMsg.msg"
  "msg/Gpos.msg"
  "msg/ScanCommandMsg.msg"
  "msg/BinPickingResults.msg"
  "msg/LearningInput.msg"
  "msg/LlmCmd.msg"
  
  "srv/Blending.srv"
  "srv/Capture.srv"
  "srv/CaptureAssistantSuggestSettings.srv"
  "srv/SetTargetJointPosition.srv"
  "srv/SetTargetPose.srv"
  "srv/PartScanning.srv"
  "srv/PartDetection.srv"
  "srv/GraspingPose.srv"
  "srv/GrpPose.srv"
  "srv/GripperCommand.srv"
  "srv/JogCommand.srv"
  "srv/StopRobot.srv"
  "srv/Relay.srv"
  "srv/SendCommand.srv"
  "srv/SetCommand.srv"
  "srv/SetTcp.srv"
  "srv/ImpedanceControl.srv"
  "srv/ZividDoScan.srv"
  "srv/DoTemplateMatching.srv"
  "srv/DoPoseEstimation.srv"
  "srv/DoTaskRecognition.srv"
  "srv/RSGetImage.srv"
  "srv/RLAlign.srv"
  "srv/RLInsert.srv"
  "srv/RLInsertAction.srv"
  "srv/BoolSrv.srv"
  "srv/DoPclTest.srv"
  "srv/InsertdoneMask.srv"
  "srv/DoneMask.srv"
  "srv/DoLearningWorkspace.srv"
  

  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# catkin_package(
#   CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
# )
ament_export_dependencies(rosidl_default_runtime)
ament_package()
