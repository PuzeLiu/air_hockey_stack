//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/JointState.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/message_filter.h>


namespace tactical_agent{

struct ObservationState{
    sensor_msgs::JointState jointState;
    geometry_msgs::TransformStamped puckPos;
    geometry_msgs::TwistStamped puckVel;
};

class Observer {
public:
    Observer();
    ~Observer();
    const ObservationState& getObservation();

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void puckStateCallback(const geometry_msgs::TransformStamped msg);

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::JointState> jointSub_;
//    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::MessageFilter<geometry_msgs::TransformStamped> tfFilter_;

    bool isInitialized;

    ObservationState states_;
    geometry_msgs::TransformStamped tfPrev_;
    double tf_dt_{};

    tf2::Quaternion quat_, quatPrev_, quatDiff_;
    tf2::Vector3 vecDiff_;
};
}


#endif //SRC_TACTICAL_AGENT_OBSERVATION_H
