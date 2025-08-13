cmake_minimum_required(VERSION 3.0.2)
project(hand_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   DetectionInfo.msg
   DetectionInfoArray.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   move_hand_single.srv
   move_hand_both.srv
   gripOnline.srv
   home_service.srv
   SetTargetClass.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(hand_planner_lib
  src/HandManager.cpp
  src/S5_hand.cpp
  src/S5_hand_jacobian.cpp
  src/MinimumJerkInterpolation.cpp 
  src/head_code.cpp
  # Add other .cpp files if they are part of the library
)

target_link_libraries(hand_planner_lib
  ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hand_planner_lib
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Eigen3
)


# add_executable(hand_manager_node src/HandManager.cpp src/S5_hand.cpp src/S5_hand_jacobian.cpp src/MinimumJerkInterpolation.cpp)
# target_link_libraries(hand_manager_node ${catkin_LIBRARIES})
# add_dependencies(hand_manager_node hand_planner_gencpp)

add_executable(head_sub src/head_code.cpp)
target_link_libraries(head_sub ${catkin_LIBRARIES})
