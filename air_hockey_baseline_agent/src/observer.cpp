#include "observer.h"

using namespace AirHockey;

Observer::Observer(ros::NodeHandle nh, ros::Rate rate) : nh_(nh), rate_(rate), puckTracker_(nh){
    jointSub_ = nh_.subscribe("joint_position_trajectory_controller/state", 1, &Observer::jointStateCallback, this);
    ROS_INFO_STREAM("Namepsace: " << nh_.getNamespace());
}

void Observer::start(){
    puckTracker_.start();
}

Observer::~Observer() {
}

void Observer::jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        observationState_.jointPosition[i] = msg->actual.positions[i];
        observationState_.jointVelocity[i] = msg->actual.velocities[i];
        observationState_.jointDesiredPosition[i] = msg->desired.positions[i];
        observationState_.jointDesiredVelocity[i] = msg->desired.velocities[i];
    }
}

const ObservationState& Observer::getObservation() {
    if (nh_.ok()){
        observationState_.puckPredictedState = puckTracker_.getPredictedState();
        ros::spinOnce();
    }
    return observationState_;
}