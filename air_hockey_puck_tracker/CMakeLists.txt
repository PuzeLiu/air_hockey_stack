cmake_minimum_required(VERSION 3.0.2)
project(air_hockey_puck_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy rosbag tf2_ros geometry_msgs message_generation)
find_package(Eigen3 REQUIRED)
find_package(Kalman REQUIRED)

add_message_files(
    FILES
    InnovationMsg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs actionlib_msgs geometry_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES puck_tracker
    CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros message_runtime
    #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

add_library(puck_tracker
    src/PuckTracker.cpp
    src/SystemModel.cpp
    src/ObservationModel.cpp
    src/CollisionModel.cpp
    src/EKF_Wrapper.cpp
    src/VisualizationInterface.cpp
    )

add_dependencies(puck_tracker
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(puck_tracker
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${Kalman_LIBRARIES}
    )

add_executable(tracker_node
    src/main.cpp)

add_dependencies(tracker_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(tracker_node
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${Kalman_LIBRARIES}
    puck_tracker
    )



#############
## Install ##
#############

install(TARGETS tracker_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    )

