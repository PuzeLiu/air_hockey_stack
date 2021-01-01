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

    if (nh_.getNamespace() == "/iiwa_front"){
        robotBaseName = "F_link_0";
    } else if (nh_.getNamespace() == "/iiwa_back"){
        robotBaseName = "B_link_0";
    } else{
        ROS_INFO_STREAM("node should run in namespace: /iiwa_front or /iiwa_back");
    }
}

Observer::~Observer() {
    thread_.join();
}

void Observer::jointStateCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg) {
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        observationState_.jointPosition[i] = msg->position[i];
        observationState_.jointVelocity[i] = msg->velocity[i];
    }
}

void Observer::startObservation() {
    ros::Duration(1.).sleep();
    while (nh_.ok()) {
        try {
            puckStateCallback(tfBuffer_.lookupTransform(robotBaseName, "Puck", ros::Time(0)));
        }
        catch (tf2::TransformException e) {
        }
        ros::spinOnce();
        rate_.sleep();
        observationStatePrev_ = observationState_;
    }
}

void Observer::puckStateCallback(const geometry_msgs::TransformStamped msg) {
    if (!isInitialized) {
        observationState_.puckPosition.x() = msg.transform.translation.x;
        observationState_.puckPosition.y() = msg.transform.translation.y;
        observationState_.puckPosition.z() = msg.transform.translation.z;
        quat_.x() = msg.transform.rotation.x;
        quat_.y() = msg.transform.rotation.y;
        quat_.z() = msg.transform.rotation.z;
        quat_.w() = msg.transform.rotation.w;
        observationState_.puckYaw = quat_.toRotationMatrix().eulerAngles(0, 1, 2)[2];
        observationState_.time = msg.header.stamp.toSec();
        isInitialized = true;
    }
    else {
        observationState_.puckPosition.x() = msg.transform.translation.x;
        observationState_.puckPosition.y() = msg.transform.translation.y;
        observationState_.puckPosition.z() = msg.transform.translation.z;
        quat_.x() = msg.transform.rotation.x;
        quat_.y() = msg.transform.rotation.y;
        quat_.z() = msg.transform.rotation.z;
        quat_.w() = msg.transform.rotation.w;
        observationState_.puckYaw = quat_.toRotationMatrix().eulerAngles(0, 1, 2)[2];
        if (observationState_.time > msg.header.stamp.toSec()){
            ROS_WARN_STREAM("Detect Time Jump Back, reset observation");
            isInitialized = false;
        }
        observationState_.time = msg.header.stamp.toSec();

        dt_ = observationState_.time - observationStatePrev_.time;

        observationState_.puckVelocity = (observationState_.puckPosition - observationStatePrev_.puckPosition) / dt_;
        observationState_.puckRotVelocity = (observationState_.puckYaw - observationStatePrev_.puckYaw) / dt_;
    }
}