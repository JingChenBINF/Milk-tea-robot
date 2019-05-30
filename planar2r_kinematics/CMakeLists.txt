cmake_minimum_required(VERSION 2.8.3)
project(planar2r_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
	message_generation
	tf
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#   visualization_msgs
# )

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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planar2r_kinematics
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planar2r_kinematics
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(../../../IR_library/include)
LINK_DIRECTORIES(../../../IR_library/lib)

add_executable(planar2r_kinematics_forwardKinematics src/forwardKinematics.cpp)
target_link_libraries(planar2r_kinematics_forwardKinematics ${catkin_LIBRARIES} IRLibrary)

add_executable(planar2r_kinematics_geometry_shapes src/geometry_shapes.cpp)
target_link_libraries(planar2r_kinematics_geometry_shapes ${catkin_LIBRARIES})

add_executable(planar2r_kinematics_genConfig src/genConfig.cpp)
target_link_libraries(planar2r_kinematics_genConfig ${catkin_LIBRARIES} IRLibrary)
####################
## Add genConfig build instructions ##
####################
# add_executable(planar2r_kinematics_genConfig src/genConfig.cpp)
# target_link_libraries(planar2r_kinematics_genConfig ${catkin_LIBRARIES} IRLibrary)

add_executable(planar2r_kinematics_simpleTraj src/simpleTraj.cpp)
target_link_libraries(planar2r_kinematics_simpleTraj ${catkin_LIBRARIES} IRLibrary)
