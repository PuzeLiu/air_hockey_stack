#ifndef SRC_UNIVERSAL_JOINT_PLUGIN_H
#define SRC_UNIVERSAL_JOINT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ignition/math.hh>

namespace gazebo {
    class UniversalJointPlugin : public ModelPlugin {
    public:
        UniversalJointPlugin();
        ~UniversalJointPlugin() override;
        void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf ) override;
        void OnUpdate ();

    private:
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        // Pointer to the model
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        ignition::math::Pose3d pose_;

        std::string robotNamespace_, robotBaseLinkName_;
        std::string eeLinkName_;
        std::string jointName1_;
        std::string jointName2_;
    };
    GZ_REGISTER_MODEL_PLUGIN(UniversalJointPlugin)
}


#endif //SRC_UNIVERSAL_JOINT_PLUGIN_H
