cmake_minimum_required(VERSION 2.8.3)
project(planar3r_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fopenmp)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND) 
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
  geometry_msgs
  roscpp
  rospy
  roslib
  std_msgs
  visualization_msgs
	message_generation
	tf
)

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
#  LIBRARIES planar2r_kinematics
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(../../../IR_library/include)
LINK_DIRECTORIES(../../../IR_library/lib)


add_executable(planar3r_kinematics_forwardKinematics src/forwardKinematics.cpp)
target_link_libraries(planar3r_kinematics_forwardKinematics ${catkin_LIBRARIES} IRLibrary ${OpenMP_LIBS})

add_executable(planar3r_kinematics_geometry_shapes src/geometry_shapes.cpp)
target_link_libraries(planar3r_kinematics_geometry_shapes ${catkin_LIBRARIES})

add_executable(planar3r_kinematics_simpleTraj src/simpleTraj.cpp)
target_link_libraries(planar3r_kinematics_simpleTraj ${catkin_LIBRARIES} IRLibrary ${OpenMP_LIBS})
