cmake_minimum_required(VERSION 3.0.2)
project(air_hockey_referee)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   tf2
   tf2_ros
   std_msgs
   gazebo_msgs
   message_generation
   )
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GameStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   StartGame.srv
   StopGame.srv
   PauseGame.srv
   ResetRobot.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   INCLUDE_DIRS include
   #  LIBRARIES air_hockey_referee
   CATKIN_DEPENDS roscpp tf2 tf2_ros std_msgs message_runtime gazebo_msgs
   #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}
   src/main.cpp
   src/referee.cpp
   src/gazebo_referee.cpp
   src/real_world_referee.cpp)


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Eigen3_LIBRARIES}
   )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY msg
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY srv
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
