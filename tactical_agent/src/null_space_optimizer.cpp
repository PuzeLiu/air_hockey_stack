//
// Created by puze on 28.12.20.
//

#include "null_space_optimizer.h"
using namespace tactical_agent;
NullSpaceOptimizer::NullSpaceOptimizer(iiwas_kinematics::Kinematics kinematics, bool closeLoop): kinematics_(kinematics) {
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.data()->setNumberOfVariables(4);
    solver_.data()->setNumberOfConstraints(iiwas_kinematics::NUM_OF_JOINTS);
    P_.resize(4, 4);
    A_.resize(7, 4);
    P_.setIdentity();
    A_.setZero();
    dimNullSpace_ = 4;
}
