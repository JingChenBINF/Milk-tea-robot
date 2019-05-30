cmake_minimum_required(VERSION 2.8.3)
project(planar3r_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


