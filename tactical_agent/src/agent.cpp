#include "agent.h"

using namespace tactical_agent;

Agent::Agent(ros::NodeHandle nh) : nh_(nh), observer_(nh, ros::Rate(10)) {
    universalJointHeight_ = 0.195;
    kinematics_ = new iiwas_kinematics::Kinematics(Vector3d(0.0, 0.0, 0.515),
                                                   Quaterniond(1.0, 0.0, 0.0, 0.0));
    bezierHit_ = new BezierHit(Vector2d(0.6, -0.45),
                               Vector2d(2.43, 0.45),
                               universalJointHeight_);
}

Agent::~Agent() {
}

void Agent::update() {
    observationStates_ = observer_.getObservation();
    updateTactic();
    generateTrajectory();
}

void Agent::updateTactic() {
    tacticState_ = Tactics::SMASH;
}

bool Agent::generateTrajectory() {
    if (tacticState_ == Tactics::SMASH){
//        iiwas_kinematics::Kinematics::JointArrayType qCur = iiwas_kinematics::Kinematics::JointArrayType(observationStates_.jointState);
        Vector3d xCur, puckCur;
//        kinematics_->forwardKinematics(qCur, xCur);
        puckCur = Vector3d(observationStates_.puckPos.transform.translation);
//        ROS_INFO_STREAM("EE: "<< xCur);
        ROS_INFO_STREAM("Puck: "<< puckCur);
    }
    return false;
}
