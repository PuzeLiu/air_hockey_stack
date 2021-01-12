#include "puck_tf_publisher_plugin.h"

using namespace gazebo;

PuckTFPublisher::PuckTFPublisher() {
    ROS_INFO_STREAM_NAMED("puck_tf_publisher", "Start PuckTFPublisher Constructor");
    transformMsg_.header.frame_id = "world";
    transformMsg_.child_frame_id = "Puck";
    update_rate_ = 0.;
    update_period_ = 0.;
}

PuckTFPublisher::~PuckTFPublisher() {
    rosnode_->shutdown();
}

void PuckTFPublisher::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    ROS_INFO_STREAM_NAMED("puck_tf_publisher", "Start Puck_TF_Publisher");
    // Store the pointer to the model
    this->model_ = _model;
    this->world_ = model_->GetWorld();

    this->robot_namespace_ = model_->GetName ();
    if ( !_sdf->HasElement ( "robotNamespace" ) ) {
        ROS_INFO_NAMED("puck_tf_publisher", "PuckTFPublisher Plugin missing <robotNamespace>, defaults to \"%s\"",
                       this->robot_namespace_.c_str() );
    } else {
        this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
        if ( this->robot_namespace_.empty() ) this->robot_namespace_ = model_->GetName ();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

    this->update_rate_ = 100.0;
    if ( !_sdf->HasElement ( "updateRate" ) ) {
        ROS_WARN_NAMED("puck_tf_publisher", "PuckTFPublisher Plugin (ns = %s) missing <updateRate>, defaults to %f",
                       this->robot_namespace_.c_str(), this->update_rate_ );
    } else {
        this->update_rate_ = _sdf->GetElement ( "updateRate" )->Get<double>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) {
        this->update_period_ = 1.0 / this->update_rate_;
    } else {
        this->update_period_ = 0.0;
    }

    last_update_time_ = this->world_->SimTime();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin (boost::bind ( &PuckTFPublisher::OnUpdate, this));
}

void PuckTFPublisher::OnUpdate() {
    common::Time current_time = this->world_->SimTime();
    if (current_time < last_update_time_)
    {
        ROS_WARN_NAMED("puck_tf_publisher", "Negative update time difference detected.");
        last_update_time_ = current_time;
    }

    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ && ros::Time::now() > transformMsg_.header.stamp) {
        publishTF();
        last_update_time_+= common::Time ( update_period_ );
    }
}

void PuckTFPublisher::publishTF() {
    pose_ = this->model_->WorldPose();
    transformMsg_.header.stamp = ros::Time::now();
    transformMsg_.transform.translation.x = pose_.Pos().X();
    transformMsg_.transform.translation.y = pose_.Pos().Y();
    transformMsg_.transform.translation.z = pose_.Pos().Z();
    transformMsg_.transform.rotation.x = pose_.Rot().X();
    transformMsg_.transform.rotation.y = pose_.Rot().Y();
    transformMsg_.transform.rotation.z = pose_.Rot().Z();
    transformMsg_.transform.rotation.w = pose_.Rot().W();
    broadcaster_.sendTransform(transformMsg_);
}
