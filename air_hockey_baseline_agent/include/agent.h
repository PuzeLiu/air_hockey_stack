//
// Created by puze on 27.12.20.
//

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include <random>

#include "iiwas_kinematics.h"

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "null_space_optimizer.h"
#include "observer.h"
#include "planner/bezier_hit.h"
#include "planner/combinatorial_hit.h"
#include "planner/stable_dynamics_motion.h"
#include "planner/cubic_linear_motion.h"

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

        void start();

        void update();
        void gotoInit();

    private:
        void loadParam();
        void updateTactic();
        bool generateTrajectory();
        bool startHit();
        bool startCut();
        bool startReady();
        bool startPrepare();

        double updateGoal(Vector2d puckPos);
        void setTactic(Tactics tactic);
        void applyTransform(trajectory_msgs::MultiDOFJointTrajectory& cartesianTrajectory);
        void applyInverseTransform(Vector3d& v_in);
        void applyInverseRotation(Vector3d& v_in);

    private:
        ros::NodeHandle nh_;
        ros::Publisher jointTrajectoryPub_;
        ros::Publisher cartTrajectoryPub_;
        ros::Rate rate_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        geometry_msgs::TransformStamped tfRobot2Table_, tfRobot2TableInverse_;
        Tactics tacticState_;
        bool tacticChanged_;
        ObservationState observationState_;

        iiwas_kinematics::Kinematics::JointArrayType qHome_;
        Vector2d xHome_, xGoal_, xCutPrev_;
        trajectory_msgs::MultiDOFJointTrajectory cartTrajectory_;
        trajectory_msgs::JointTrajectory jointTrajectory_;
        trajectory_msgs::JointTrajectoryPoint jointViaPoint_;

        double universalJointHeight_;
        double puckRadius_, malletRadius_;
        double tableLength_, tableWidth_;

        double defendLine_;
        double vHitMax_, vDefendMin_, tDefendMin_;
        Vector2d hitRange_;

        Observer* observer_;
        iiwas_kinematics::Kinematics* kinematics_;

        CombinatorialHit* combinatorialHit_;
        CubicLinearMotion* cubicLinearMotion_;

        NullSpaceOptimizer* optimizer_;

        ros::Time trajStopTime_;

        double smashCount_;
        double maxPredictionTime_;
        std::mt19937 rng_;
        std::uniform_int_distribution<int> dist_;
    };
}

#endif //SRC_TACTICAL_AGENT_H
