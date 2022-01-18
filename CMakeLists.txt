cmake_minimum_required(VERSION 3.0.2)
project(msg_handler)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  nav_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)


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
#  LIBRARIES msg_handler
#  CATKIN_DEPENDS rospy
 DEPENDS system_lib Boost
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(msg_restamper src/msg_restamper.cpp)
target_link_libraries(msg_restamper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(odom_header_frame_converter src/odom_header_frame_converter.cpp)
target_link_libraries(odom_header_frame_converter ${catkin_LIBRARIES})

add_executable(odom_initializer src/odom_initializer.cpp)
target_link_libraries(odom_initializer ${catkin_LIBRARIES})

add_executable(vo_tester src/vo_tester.cpp)
target_link_libraries(vo_tester ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 