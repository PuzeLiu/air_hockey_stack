#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>

#ifndef SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_UTILS_SRC_TRAJECTORY_VISUALIZER_H_
#define SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_UTILS_SRC_TRAJECTORY_VISUALIZER_H_

class TrajectoryVisualizer
{
 public:
    TrajectoryVisualizer(ros::NodeHandle nh, int bufferSize);

    void update();

    void cartesianTrajecotoryCB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
    void jointTrajecotoryCB(const control_msgs::JointTrajectoryControllerState& msg);

 private:
    std::string getControllerName();
    void publishDesiredPath();
    void publishActualPath();

    ros::NodeHandle nh;
    int bufferSize;
    std::string sourceFrame, targetFrame;
    ros::Subscriber cartesianTrajectorySubscriber, jointTrajectorySubscriber;
    ros::Publisher desiredCartesianPathPublisher, actualCartesianPathPublisher;
    nav_msgs::Path desiredPath, actualPath;

    trajectory_msgs::MultiDOFJointTrajectory cartesianTrajectoryMsg;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    bool newCartesianTrajMsg;
};

#endif //SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_UTILS_SRC_TRAJECTORY_VISUALIZER_H_
