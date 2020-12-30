//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include "observer.h"
#include "iiwas_kinematics.h"

#include "planner/bezierHit.h"

#include "null_space_optimizer.h"

#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"

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
        Agent(ros::NodeHandle nh, double rate);
        ~Agent();

        void update();
        void gotoHome();

    private:

        void updateTactic();
        bool generateTrajectory();

    private:
        ros::NodeHandle nh_;
        ros::Publisher jointTrajectoryPub_;
        double rate_;
        Tactics tacticState_;
        ObservationState observationState_;

        iiwas_kinematics::Kinematics::JointArrayType qHome_;
        Vector2d xHome_;
        trajectory_msgs::MultiDOFJointTrajectory cartTrajectory_;
        trajectory_msgs::JointTrajectory jointTrajectory_;
        trajectory_msgs::JointTrajectoryPoint jointViaPoint_;

        double universalJointHeight_; //! Table height (0.1) + mallet height (0.095) - base height(0.03)
        double puckRadius_;
        double malletRadius_;
        Vector2d xGoal_;

        Observer observer_;
        iiwas_kinematics::Kinematics* kinematics_;
        BezierHit* bezierHit_;
        NullSpaceOptimizer* optimizer_;

    };
}

#endif //SRC_TACTICAL_AGENT_H
