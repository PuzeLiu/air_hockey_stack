#ifndef SRC_UNIVERSAL_JOINT_PLUGIN_H
#define SRC_UNIVERSAL_JOINT_PLUGIN_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ignition/math.hh>

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

        int debugCounter_;
        bool started_;

    protected:
	    pinocchio::Model pinoModel_;
	    pinocchio::Data pinoData_;
	    Eigen::VectorXd qCur_;
	    Eigen::Vector3d base_pos;
	    int frame_id;
    };

    GZ_REGISTER_MODEL_PLUGIN(UniversalJointPlugin)
}


#endif //SRC_UNIVERSAL_JOINT_PLUGIN_H
