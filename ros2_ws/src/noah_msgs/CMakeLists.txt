cmake_minimum_required(VERSION 3.8)
project(noah_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# Msg 文件
set(msg_files
  "msg/AvatarInfo.msg"
  "msg/Abnomals.msg"
  "msg/CascadeLifecycleActivation.msg"
  "msg/CascadeLifecycleState.msg"
  "msg/ChassisControl.msg"
  "msg/ChassisDetail.msg"
  "msg/CommonCanData.msg"
  "msg/GNSSValue.msg"
  "msg/JoystickInput.msg"
  # "msg/LidarObstacles.msg"
  "msg/LinktrackAoaNode0.msg"
  "msg/LinktrackAoaNodeframe0.msg"
  "msg/LinktrackNode0.msg"
  "msg/LinktrackNodeframe0.msg"  
  "msg/LogicSwitchModel.msg"
  "msg/PlanParam.msg"
  "msg/RmtCtrlAdapter.msg"
  "msg/SystemEnable.msg"
  "msg/TankBatDetail.msg"
  "msg/TankFeedbackDetail.msg"
  "msg/TankFunctionControl.msg"
  "msg/WorkPath.msg"
  "msg/SteeringWheelConf.msg"
  "msg/SteeringWheelData.msg"
  "msg/StatusValue.msg"
  "msg/ActionValue.msg"
  "msg/AngleVolSensor.msg"
  "msg/TurnAngleInfo.msg"
  "msg/RopeSensorStateInfo.msg"
  "msg/RopeSensorConf.msg"
  "msg/TrackerMonitor.msg"
  "msg/SyncStatus.msg"
  "msg/PhotographFeedbackData.msg"
  "msg/PhotographPlanData.msg"
  "msg/InspectionInfo.msg"
  "msg/UploadFile.msg"
  "msg/RecordPathFeedBack.msg"
  "msg/ObstacleInfo.msg"
  "msg/ObstacleScan.msg"
  "msg/ObstacleWarning.msg"
  "msg/SignalInfo.msg"
  "msg/Warning.msg"
  "msg/TaskStatus.msg"
)

# Srv 文件
set(srv_files
  "srv/Test.srv"
  "srv/ChangeLineOfAction.srv"
  "srv/ChangeLineParam.srv"
  "srv/ChangeLineStatus.srv"
  "srv/PlanPathRequest.srv"
  "srv/TrackerStatus.srv"
  "srv/GetSysEnabled.srv"
  "srv/ReqStatusValue.srv"
  "srv/ReqActionValue.srv"
  "srv/DBQuery.srv"
  "srv/DBSqlProc.srv"
  "srv/NavigationStatus.srv"
  "srv/NavRequest.srv"
)

#Action 文件
set(action_files

)


set(dependencies
  builtin_interfaces 
  lifecycle_msgs 
  geometry_msgs 
  std_msgs 
  std_srvs
  nav_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${dependencies}
  ADD_LINTER_TESTS
)


ament_export_dependencies(rosidl_default_runtime builtin_interfaces std_srvs std_msgs)

ament_package()

