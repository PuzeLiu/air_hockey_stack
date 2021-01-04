//
// Created by puze on 20.12.20.
//

#ifndef PUCK_TF_PUBLISHER_PLUGIN_HH
#define PUCK_TF_PUBLISHER_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/JointState.h>

namespace gazebo {
    class PuckTFPublisher : public ModelPlugin
    {
    public:
        PuckTFPublisher();
        ~PuckTFPublisher() override;
        void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf ) override;
        void OnUpdate ();
        void publishTF();

    private:
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        // Pointer to the model
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        ignition::math::Pose3d pose_;

        // ROS STUFF
        boost::shared_ptr<ros::NodeHandle> rosnode_;
        tf2_ros::TransformBroadcaster broadcaster_;
        geometry_msgs::TransformStamped transformMsg_;

        std::string tf_prefix_;
        std::string robot_namespace_;

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PuckTFPublisher)
}

#endif //PUCK_TF_PUBLISHER_PLUGIN_HH
