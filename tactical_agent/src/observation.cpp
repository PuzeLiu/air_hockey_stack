//
// Created by puze on 27.12.20.
//

#include "observation.h"
using namespace tactical_agent;
Observer::Observer() : nh_("/"),
                       tfBuffer_(),
                       jointSub_(nh_, "joint_states", 1),
                       tfFilter_(tfBuffer_, "Puck", 1, nh_)
{
    ROS_INFO_STREAM("Namepsace: " << nh_.getNamespace());

    isInitialized = false;
}

Observer::~Observer() {

}

void Observer::jointStateCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg) {
    states_.jointState = *msg;
}

const ObservationState& Observer::getObservation() {
    if (nh_.ok()){
        try {
            puckStateCallback(tfBuffer_.lookupTransform("Puck", "world", ros::Time::now()));
        }
        catch (tf2::TransformException e){
        }
        ros::spinOnce();
    }

    return states_;
}

void Observer::puckStateCallback(const geometry_msgs::TransformStamped msg) {
    states_.puckVel.header = msg.header;
    if (isInitialized){
        tfPrev_ = msg;
    } else{
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

        states_.puckVel.twist.linear.x = (msg.transform.translation.x - tfPrev_.transform.translation.x) / tf_dt_;
        states_.puckVel.twist.linear.y = (msg.transform.translation.y - tfPrev_.transform.translation.y) / tf_dt_;
        states_.puckVel.twist.linear.z = (msg.transform.translation.z - tfPrev_.transform.translation.z) / tf_dt_;
        states_.puckVel.twist.angular.x = vecDiff_.x();
        states_.puckVel.twist.angular.y = vecDiff_.y();
        states_.puckVel.twist.angular.z = vecDiff_.z();

        states_.puckPos = msg;
        tfPrev_ = msg;
    }
}
