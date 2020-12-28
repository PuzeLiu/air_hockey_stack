//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include <boost/shared_ptr.hpp>
#include "observer.h"

using namespace std;

namespace tactical_agent{
    enum Tactics{
        READY = 0, //!< go to home position
        CUT = 1,   //!< defend the incoming puck to opponent's court regardless of direction
        STOP = 2,  //!< stop the incoming puck in our court
        SMASH = 3 //!< hit the static or slow-moving puck
    };

    class Agent{
    public:
        Agent(ros::NodeHandle nh);
        ~Agent();

        void update();

    private:
        void updateTactic();

    private:
        Tactics tacticState_;
        int dim_;
        Observer observer_;

        ObservationState observationStates_;
    };
}

#endif //SRC_TACTICAL_AGENT_H
