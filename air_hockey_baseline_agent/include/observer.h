#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <iiwas_kinematics.h>

//#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <boost/thread.hpp>

using namespace iiwas_kinematics;

namespace AirHockey {
//    boost::mutex stateMutex;

    struct ObservationState {
        Kinematics::JointArrayType jointPosition;
        Kinematics::JointArrayType jointVelocity;
        Kinematics::JointArrayType jointDesiredPosition;
        Kinematics::JointArrayType jointDesiredVelocity;
        Vector3d puckPosition;
        double puckYaw;
        Vector3d puckVelocity;
        double puckRotVelocity;
        double time;
//        sensor_msgs::JointState jointState;
//        geometry_msgs::TransformStamped puckPos;
//        geometry_msgs::TwistStamped puckVel;
    };

    class Observer {
    public:
        Observer(ros::NodeHandle nh, ros::Rate rate = ros::Rate(100));

        ~Observer();

        void startObservation();

        inline const ObservationState& getObservation() {return observationState_;};

    private:
//        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);

        void puckStateCallback(const geometry_msgs::TransformStamped msg);

    public:


    private:
        boost::thread thread_;
        ObservationState observationState_, observationStatePrev_;

        ros::NodeHandle nh_;
        ros::Rate rate_;
        ros::Subscriber jointSub_;
        tf2_ros::TransformListener tfListener_;
        tf2_ros::Buffer tfBuffer_;

        bool isInitialized;

        double dt_;
        std::string robotBaseName;

        Quaterniond quat_;
    };
}

#endif //SRC_TACTICAL_AGENT_OBSERVATION_H
