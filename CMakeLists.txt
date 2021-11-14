cmake_minimum_required(VERSION 3.10.1)
#cmake_minimum_required(VERSION 2.8.3)
project(lrf_odom)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE RelWithDebInfo) #None, Debug, Release, RelWithDebInfo, MinSizeRel

message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #rospy
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
  #message_generation
  #message_filters
  #genmsg
)

add_definitions(
  -Wfatal-errors # makes gcc stop compiling after the first error. this greatly reduces the size of error messages
)


# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
#message( "======== ======== ======== ======== ======== " )
#message( "old: ${csm_LIBRARY_DIRS}" )
#set(csm_LIBRARY_DIRS  "/usr/local/lib/")
#set(csm_INCLUDE_DIRS  "/usr/local/include/csm/")
#message( "new: ${csm_LIBRARY_DIRS}" )
#message( "======== ======== ======== ======== ======== " )
 

#find_package(eigen)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)


# Set include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(
  ${csm_LIBRARY_DIRS}
)

# Declare info that other packages need to import library generated here
catkin_package()

#Create node
add_executable(
	lrf_odom_node
	src/main.cpp
	src/sensor.cpp
)
#target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${csm_LIBRARIES} )
target_link_libraries( lrf_odom_node ${catkin_LIBRARIES} ${csm_LIBRARIES} Eigen3::Eigen )

#add_dependencies(sensor_ex_calib
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

