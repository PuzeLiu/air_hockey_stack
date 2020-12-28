//
// Created by puze on 27.12.20.
//

#include <agent.h>
#include "observer.h"

using namespace tactical_agent;

Observer::Observer(ros::NodeHandle nh, ros::Rate rate) : nh_(nh), rate_(rate), tfBuffer_(),
                                                         tfListener_(tfBuffer_) {
    jointSub_ = nh_.subscribe("joint_states", 1, &Observer::jointStateCallback, this);
    ROS_INFO_STREAM("Namepsace: " << nh_.getNamespace());

    isInitialized = false;

    thread_ = boost::thread(&Observer::startObservation, this);
}

Observer::~Observer() {
    thread_.join();
}

void Observer::jointStateCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg) {
    observationStates_.jointState = *msg;
}

void Observer::startObservation() {
    ros::Duration(1.).sleep();
    while (nh_.ok()) {
        try {
            puckStateCallback(tfBuffer_.lookupTransform("Puck", "world", ros::Time(0)));
        }
        catch (tf2::TransformException e) {
        }
        ros::spinOnce();
        rate_.sleep();
    }
}

void Observer::puckStateCallback(const geometry_msgs::TransformStamped msg) {
    if (isInitialized) {
        observationStates_.puckPos = msg;
        observationStates_.puckVel.header = msg.header;
        tfPrev_ = msg;
    }
    else {
        tf_dt_ = (msg.header.stamp - tfPrev_.header.stamp).toSec();

        quat_.setValue(msg.transform.rotation.x,
                       msg.transform.rotation.y,
                       msg.transform.rotation.z,
                       msg.transform.rotation.w);
        quatPrev_.setValue(tfPrev_.transform.rotation.x,
                           tfPrev_.transform.rotation.y,
                           tfPrev_.transform.rotation.z,
                           tfPrev_.transform.rotation.w);
        quatDiff_ = quat_ * tf2::inverse(quatPrev_);
        vecDiff_ = quatDiff_.getAxis() * (quatDiff_.getAngle() / tf_dt_);

        observationStates_.puckVel.header = msg.header;
        observationStates_.puckVel.twist.linear.x =
                (msg.transform.translation.x - tfPrev_.transform.translation.x) / tf_dt_;
        observationStates_.puckVel.twist.linear.y =
                (msg.transform.translation.y - tfPrev_.transform.translation.y) / tf_dt_;
        observationStates_.puckVel.twist.linear.z =
                (msg.transform.translation.z - tfPrev_.transform.translation.z) / tf_dt_;
        observationStates_.puckVel.twist.angular.x = vecDiff_.x();
        observationStates_.puckVel.twist.angular.y = vecDiff_.y();
        observationStates_.puckVel.twist.angular.z = vecDiff_.z();
        observationStates_.puckPos = msg;

        tfPrev_ = msg;
    }
}