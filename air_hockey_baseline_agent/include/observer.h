#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <iiwas_kinematics.h>

#include <control_msgs/JointTrajectoryControllerState.h>

#include "air_hockey_puck_tracker/PuckTracker.hpp"
#include "air_hockey_referee/GameStatus.h"

using namespace iiwas_kinematics;

namespace AirHockey {

    struct ObservationState {
        Kinematics::JointArrayType jointPosition;
        Kinematics::JointArrayType jointVelocity;
        Kinematics::JointArrayType jointDesiredPosition;
        Kinematics::JointArrayType jointDesiredVelocity;
        State puckEstimatedState;
        PuckPredictedState puckPredictedState;
        air_hockey_referee::GameStatus gameStatus;
    };

    class Observer {
    public:
        Observer(ros::NodeHandle nh, std::string controllerName, ros::Rate rate, double defendLine);

        ~Observer();

        void start();

        const ObservationState& getObservation();

    private:
        void jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
        void refereeStatusCallback(const air_hockey_referee::GameStatus::ConstPtr &msg);

    private:
        ObservationState observationState_, observationStatePrev_;

        ros::NodeHandle nh_;
        ros::Rate rate_;
        ros::Subscriber jointSub_;
        ros::Subscriber refereeSub_;
        AirHockey::PuckTracker  puckTracker_;
    };
}

#endif //SRC_TACTICAL_AGENT_OBSERVATION_H
