#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>


namespace tactical_agent {
//    boost::mutex stateMutex;

    struct ObservationState {
        sensor_msgs::JointState jointState;
        geometry_msgs::TransformStamped puckPos;
        geometry_msgs::TwistStamped puckVel;
    };

    class Observer {
    public:
        Observer(ros::NodeHandle nh, ros::Rate rate = ros::Rate(100));

        ~Observer();

        void startObservation();

        inline const ObservationState& getObservation() {return observationStates_;};

    private:
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

        void puckStateCallback(const geometry_msgs::TransformStamped msg);

    public:


    private:
        boost::thread thread_;
        ObservationState observationStates_;

        ros::NodeHandle nh_;
        ros::Rate rate_;
        ros::Subscriber jointSub_;
        tf2_ros::TransformListener tfListener_;
        tf2_ros::Buffer tfBuffer_;

        bool isInitialized;

        geometry_msgs::TransformStamped tfPrev_;
        double tf_dt_;
        std::string robotBaseName;

        tf2::Quaternion quat_, quatPrev_, quatDiff_;
        tf2::Vector3 vecDiff_;
    };
}

#endif //SRC_TACTICAL_AGENT_OBSERVATION_H
