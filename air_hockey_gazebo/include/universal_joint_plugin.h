#ifndef SRC_UNIVERSAL_JOINT_PLUGIN_H
#define SRC_UNIVERSAL_JOINT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ignition/math.hh>
#include "iiwas_kinematics.h"

namespace gazebo {
    class UniversalJointPlugin : public ModelPlugin {
    public:
        UniversalJointPlugin();
        ~UniversalJointPlugin() override;
        void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf ) override;
        void OnUpdate ();

    private:
        ros::NodeHandle nh_;
        ros::Publisher jointStatePub_;
        sensor_msgs::JointState universalJointState_;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        // Pointer to the model
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        physics::JointControllerPtr jointController_;

        std::string robotNamespace_;
        std::string jointName1_;
        std::string jointName2_;
        std::vector<std::string> iiwaJointNames_;

        iiwas_kinematics::Kinematics* kinematics_;
        iiwas_kinematics::Kinematics::JointArrayType qCur_;
        Vector3d posCur_;
        Quaterniond quatCur_;

        int debugCounter_;
        bool started_;
    };

    GZ_REGISTER_MODEL_PLUGIN(UniversalJointPlugin)
}


#endif //SRC_UNIVERSAL_JOINT_PLUGIN_H
