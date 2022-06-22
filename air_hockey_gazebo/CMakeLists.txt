cmake_minimum_required(VERSION 3.0.2)
project(air_hockey_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  gazebo_ros
  sensor_msgs
)

find_package(gazebo REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS gazebo_ros
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(puck_tf_plugin SHARED src/puck_tf_publisher_plugin.cc)
add_library(universal_joint_plugin SHARED src/universal_joint_plugin.cc)
add_executable(universal_joint_state_publisher src/universal_joint_state_publisher.cpp)

target_link_libraries(puck_tf_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

target_link_libraries(universal_joint_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pinocchio::pinocchio)

target_link_libraries(universal_joint_state_publisher
    ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY world
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )

