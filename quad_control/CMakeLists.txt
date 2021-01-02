cmake_minimum_required(VERSION 2.8.3)
project(quad_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tf
  tf_conversions
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
  PlanRequest.msg
  Trajectory.msg
  UAVPose.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    wind.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#	 nav.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
#   actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surveillance_task
#  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp roslib sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/surveillance_task.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(APPlanner2D src/APPlanner2D.cpp src/MapAnalyzer.cpp)
target_link_libraries(APPlanner2D ${catkin_LIBRARIES})

add_executable(Controller src/ControllerNode.cpp src/Controller.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})

add_executable(PController src/PControllerNode.cpp src/PController.cpp src/Controller.cpp)
target_link_libraries(PController ${catkin_LIBRARIES})

add_executable(plannerTest src/plannerTest.cpp)
target_link_libraries(plannerTest ${catkin_LIBRARIES})

add_library(wind_plugin src/WindPlugin.cc)
target_link_libraries(wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
