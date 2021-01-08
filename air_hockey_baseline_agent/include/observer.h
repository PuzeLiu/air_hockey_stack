#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <iiwas_kinematics.h>

//#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Int8.h>

#include "air_hockey_puck_tracker/PuckTracker.hpp"

using namespace iiwas_kinematics;

namespace AirHockey {

    struct ObservationState {
        Kinematics::JointArrayType jointPosition;
        Kinematics::JointArrayType jointVelocity;
        Kinematics::JointArrayType jointDesiredPosition;
        Kinematics::JointArrayType jointDesiredVelocity;
        PuckPredictedState puckPredictedState;
        int8_t gameStatus;
    };

    class Observer {
    public:
        Observer(ros::NodeHandle nh, ros::Rate rate = ros::Rate(100));

        ~Observer();

        void start();

        const ObservationState& getObservation();

    private:
        void jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
        void refereeStatusCallback(const std_msgs::Int8::ConstPtr &msg);

        void getObservationCallback();

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
