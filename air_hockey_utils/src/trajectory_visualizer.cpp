#include "air_hockey_utils/trajectory_visualizer.h"

TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle nh_, int buffer_size) : nh(nh_.getNamespace()),
                                                                                   tfListener(tfBuffer)
{
    bufferSize = buffer_size;
    cartesianTrajectorySubscriber = nh.subscribe("cartesian_trajectory", 1,
        &TrajectoryVisualizer::cartesianTrajecotoryCB, this);
    desiredCartesianPathPublisher = nh.advertise<nav_msgs::Path>("desired_path", 2);
    actualCartesianPathPublisher = nh.advertise<nav_msgs::Path>("actual_path", 2);

    cartesianTrajecotoryMsg.points.push_back(trajectory_msgs::MultiDOFJointTrajectoryPoint());
    newCartesianTrajMsg = false;

    if (nh.getNamespace() == "/iiwa_front")
    {
        targetFrame = "F_striker_tip";
        sourceFrame = "F_link_0";
    }
    else if (nh.getNamespace() == "/iiwa_back")
    {
        targetFrame = "B_striker_tip";
        sourceFrame = "B_link_0";
    }
    else
    {
        ROS_FATAL_STREAM("Unknown namespace for trajectory visualization: " << nh.getNamespace());
    }
}

void TrajectoryVisualizer::update()
{
    if (ros::Time::now() > cartesianTrajecotoryMsg.header.stamp and newCartesianTrajMsg)
    {
        newCartesianTrajMsg = false;
        if (desiredPath.poses.size() + cartesianTrajecotoryMsg.points.size() > bufferSize)
        {
            desiredPath.poses.erase(desiredPath.poses.begin(),desiredPath.poses.begin() +
                    int(std::min(desiredPath.poses.size() + cartesianTrajecotoryMsg.points.size() - bufferSize,
                        desiredPath.poses.size())));
        }
        publishDesiredPath();
    }

    publishActualPath();

}

void TrajectoryVisualizer::cartesianTrajecotoryCB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
    newCartesianTrajMsg = true;
    cartesianTrajecotoryMsg = *msg.get();
}

void TrajectoryVisualizer::jointTrajecotoryCB(const control_msgs::JointTrajectoryControllerState& msg)
{
}

std::string TrajectoryVisualizer::getControllerName()
{
    std::string controllerName;
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics))
    {
        for (int i = 0; i < topics.size(); ++i)
        {
            if (topics[i].name.find("trajectory_controller/state") != std::string::npos)
            {
                controllerName = topics[i].name.erase(topics[i].name.size() - 6);
            }
        }
        if (controllerName == "")
        {
            ROS_FATAL_STREAM("Could not find controller");
            exit(-1);
        }
    }
    return controllerName;
}

void TrajectoryVisualizer::publishDesiredPath()
{
    geometry_msgs::PoseStamped poseTmp;
    desiredPath.header.stamp = cartesianTrajecotoryMsg.header.stamp;
    desiredPath.header.frame_id = sourceFrame;
    for (int i = 0; i < cartesianTrajecotoryMsg.points.size(); ++i)
    {
        poseTmp.header.stamp =
            cartesianTrajecotoryMsg.header.stamp + cartesianTrajecotoryMsg.points[i].time_from_start;
        poseTmp.pose.position.x = cartesianTrajecotoryMsg.points[i].transforms[0].translation.x;
        poseTmp.pose.position.y = cartesianTrajecotoryMsg.points[i].transforms[0].translation.y;
        poseTmp.pose.position.z = cartesianTrajecotoryMsg.points[i].transforms[0].translation.z;
        desiredPath.poses.push_back(poseTmp);
    }
    desiredCartesianPathPublisher.publish(desiredPath);
}

void TrajectoryVisualizer::publishActualPath()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform(sourceFrame, targetFrame, ros::Time::now(), ros::Duration(0.1));
        geometry_msgs::PoseStamped poseTmp;
        actualPath.header.stamp = ros::Time::now();
        actualPath.header.frame_id = sourceFrame;
        poseTmp.header.stamp = transformStamped.header.stamp;
        poseTmp.pose.position.x = transformStamped.transform.translation.x;
        poseTmp.pose.position.y = transformStamped.transform.translation.y;
        poseTmp.pose.position.z = transformStamped.transform.translation.z;
        actualPath.poses.push_back(poseTmp);
        if (actualPath.poses.size() > 100) {
            actualPath.poses.erase(actualPath.poses.begin());
        }
        actualCartesianPathPublisher.publish(actualPath);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM_ONCE(ex.what());
        ros::Duration(1.0).sleep();
    }

}