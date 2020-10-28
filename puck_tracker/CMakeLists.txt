cmake_minimum_required(VERSION 3.0.2)
project(puck_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

find_package(Eigen3 REQUIRED)
find_package(Kalman REQUIRED)



###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )


add_executable(tracker_node
        src/main.cpp)

add_dependencies(tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tracker_node
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${Kalman_LIBRARIES}
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

