cmake_minimum_required(VERSION 3.0.2)
project(air_hockey_baseline_agent)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   geometry_msgs
   sensor_msgs
   trajectory_msgs
   tf2
   tf2_ros
   tf2_geometry_msgs
   air_hockey_puck_tracker
   air_hockey_referee
   )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(NLopt REQUIRED)
find_package(pinocchio REQUIRED)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES air_hockey_puck_tracker
   CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs air_hockey_puck_tracker air_hockey_referee
)

###########
## Build ##
###########

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

set(SOURCES
   src/agent.cpp
   src/system_state.cpp
   src/utils.cpp
   src/null_space_optimizer.cpp
   src/hitting_point_optimizer.cpp
   src/observer.cpp
   src/trajectory_generator.cpp
   src/transformations.cpp
   src/planner/bezier_hit.cpp
   src/planner/combinatorial_hit.cpp
   src/planner/combinatorial_hit_new.cpp
   src/planner/stable_dynamics_motion.cpp
   src/planner/cubic_linear_motion.cpp
   src/tactics/cut.cpp
   src/tactics/home.cpp
   src/tactics/init.cpp
   src/tactics/move_puck.cpp
   src/tactics/prepare.cpp
   src/tactics/ready.cpp
   src/tactics/repel.cpp
   src/tactics/smash.cpp
   src/tactics/tactics.cpp
   )

add_executable(${PROJECT_NAME}
   src/main.cpp
   ${SOURCES})

add_executable(hitting_point_optimizer_test
   src/utils.cpp
   src/hitting_point_optimizer.cpp
   src/test/hitting_point_optimizer_test.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   osqp::osqp
   OsqpEigen
   NLopt::nlopt
   Clp
   CoinUtils
   pinocchio::pinocchio
   )
target_link_libraries(hitting_point_optimizer_test
   ${catkin_LIBRARIES}
   NLopt::nlopt
   Clp
   CoinUtils
   pinocchio::pinocchio
   )


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
   )

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
   )

install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
   )


