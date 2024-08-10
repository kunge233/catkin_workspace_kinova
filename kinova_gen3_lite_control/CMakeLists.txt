cmake_minimum_required(VERSION 3.0.2)
project(kinova_gen3_lite_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
  actionlib
  kortex_driver
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(yaml-cpp REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TargetPose.srv
  GripperValue.srv
  ActionTrigger.srv
  NamedPose.srv
  GetPose.srv
  AddRemoveCollisionObject.srv
  PrintCollisionObjects.srv
  MoveArmToPose.srv
  Pick.srv
  Place.srv
  DetachAndRemoveObject.srv
  ManagePose.srv
  PlanAndSaveTrajectory.srv
  ManageSavedPlans.srv
  StepAdjust.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinova_gen3_lite_control
  CATKIN_DEPENDS
    roscpp
    rospy 
    std_msgs 
    geometry_msgs 
    moveit_msgs
    kortex_driver 
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(arm_controller src/arm_controller.cpp)
add_executable(pick_place_controller src/pick_place_controller.cpp)

## Add cmake target dependencies of the executable
add_dependencies(arm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pick_place_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_controller
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
target_link_libraries(pick_place_controller
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   yaml-cpp
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS arm_controller pick_place_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/arm_controller_moveit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Ensure scripts are executable
execute_process(
  COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/scripts -name "*.py" -exec chmod +x {} \;
)
