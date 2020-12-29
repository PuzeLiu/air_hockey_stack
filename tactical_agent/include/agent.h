//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include <boost/shared_ptr.hpp>
#include "observer.h"
#include "planner/bezierHit.h"
#include "iiwas_kinematics.h"

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
        bool generateTrajectory();

    private:
        ros::NodeHandle nh_;
        Tactics tacticState_;
        ObservationState observationStates_;
        double universalJointHeight_; //! Table height (0.1) + mallet height (0.095)

        Observer observer_;
        iiwas_kinematics::Kinematics* kinematics_;
        BezierHit* bezierHit_;

    };
}

#endif //SRC_TACTICAL_AGENT_H
