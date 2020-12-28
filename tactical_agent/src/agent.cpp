#include "agent.h"

using namespace tactical_agent;

Agent::Agent(ros::NodeHandle nh) : observer_(nh, ros::Rate(10)) {
}

Agent::~Agent() {
}

void Agent::update() {
    observationStates_ = observer_.getObservation();
    updateTactic();

}

void Agent::updateTactic() {
    tacticState_ = Tactics::SMASH;
}
