#include "agent.h"

using namespace tactical_agent;

Agent::Agent() {
    observer_ = boost::make_shared<Observer>();
}

Agent::~Agent() {

}

void Agent::update() {
    observation_ = observer_->getObservation();

}
