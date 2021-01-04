//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include <random>

#include "iiwas_kinematics.h"

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "../../air_hockey_baseline_agent/include/null_space_optimizer.h"
#include "../../air_hockey_baseline_agent/include/observer.h"
#include "../../air_hockey_baseline_agent/include/planner/bezier_hit.h"
#include "../../air_hockey_baseline_agent/include/planner/combinatorial_hit.h"
#include "../../air_hockey_baseline_agent/include/planner/stable_dynamics_motion.h"

using namespace std;

namespace AirHockey{
    enum Tactics{
        READY = 0,    //!< go to home position
        CUT = 1,      //!< defend the incoming puck to opponent's court regardless of direction
        PREPARE = 2,  //!< adjust the puck's position when smash fails
        SMASH = 3     //!< hit the static or slow-moving puck
    };

    class Agent{
    public:
        Agent(ros::NodeHandle nh, double rate);
        ~Agent();

        void update();
        void gotoInit();

    private:
        void updateTactic();
        bool generateTrajectory();
        double updateGoal(Vector2d puckPos);
        void setTactic(Tactics tactic);

    private:
        ros::NodeHandle nh_;
        ros::Publisher jointTrajectoryPub_;
        ros::Publisher cartTrajectoryPub_;
        ros::Rate rate_;
        Tactics tacticState_;
        bool tacticChanged_;
        ObservationState observationState_;

        iiwas_kinematics::Kinematics::JointArrayType qHome_;
        Vector2d xHome_;
        trajectory_msgs::MultiDOFJointTrajectory cartTrajectory_;
        trajectory_msgs::JointTrajectory jointTrajectory_;
        trajectory_msgs::JointTrajectoryPoint jointViaPoint_;

        double universalJointHeight_; //! Table height (0.1) + mallet height (0.095) - base height(0.03)
        double puckRadius_;
        double malletRadius_;
        Matrix2d tableEdge_;
        Vector2d xGoal_;

        Observer observer_;
        iiwas_kinematics::Kinematics* kinematics_;

        BezierHit* bezierHit_;
        CombinatorialHit* combinatorialHit_;
        StableDynamicsMotion* stableDynamicsMotion_;

        NullSpaceOptimizer* optimizer_;

        double smashCount_;
        std::mt19937 rng_;
        std::uniform_int_distribution<int> dist_;
    };
}

#endif //SRC_TACTICAL_AGENT_H
