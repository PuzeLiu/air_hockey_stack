#include "universal_joint_plugin.h"

using namespace gazebo;

UniversalJointPlugin::UniversalJointPlugin(): nh_("/"){


}

UniversalJointPlugin::~UniversalJointPlugin() {

}

void UniversalJointPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    ROS_INFO_STREAM_NAMED("Universal Joint Plugin" , "Start Universal Joint Plugin");
    this->model_ = _model;
    this->world_ = model_->GetWorld();
    this->robotNamespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();

    if (this->robotNamespace_ == "/iiwa_front/"){
        jointStatePub_ = nh_.advertise<sensor_msgs::JointState>(this->robotNamespace_ + "joint_states", 1);
        robotBaseLinkName_ = "front_base";
        eeLinkName_ = "F_link_ee";
        jointName1_ = "iiwa_front::F_striker_joint_1";
        jointName2_ = "iiwa_front::F_striker_joint_2";
        universalJointState_.name.push_back("F_striker_joint_1");
        universalJointState_.name.push_back("F_striker_joint_2");
    } else if (this->robotNamespace_ == "/iiwa_back/"){
        jointStatePub_ = nh_.advertise<sensor_msgs::JointState>(this->robotNamespace_ + "joint_states", 1);
        robotBaseLinkName_ = "back_base";
        eeLinkName_ = "B_link_ee";
        jointName1_ = "iiwa_back::B_striker_joint_1";
        jointName2_ = "iiwa_back::B_striker_joint_2";
        universalJointState_.name.push_back("B_striker_joint_1");
        universalJointState_.name.push_back("B_striker_joint_2");
    } else {
        ROS_ERROR_STREAM_NAMED("Universal Joint Plugin" , "No namespace for the iiwa. Use /iiwa_front or /iiwa_back.");
    }

    universalJointState_.position.resize(2);
    universalJointState_.velocity.push_back(0.);
    universalJointState_.velocity.push_back(0.);
    universalJointState_.effort.push_back(0.);
    universalJointState_.effort.push_back(0.);

    debugCounter_ = 0;
    started_ = false;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin (boost::bind ( &UniversalJointPlugin::OnUpdate, this));
}

void UniversalJointPlugin::OnUpdate() {
    if (!started_){

        this->jointController_.reset(new physics::JointController(this->model_));
        gazebo::common::PID pid(10, 0, 0.01);

        this->jointController_->AddJoint(model_->GetJoint(jointName1_));
        this->jointController_->SetPositionPID(jointName1_, pid);
        this->jointController_->AddJoint(model_->GetJoint(jointName2_));
        this->jointController_->SetPositionPID(jointName2_, pid);
        started_ = true;
    } else{
        ignition::math::Pose3d poseBase;
        poseBase = this->model_->GetLink(robotBaseLinkName_)->WorldPose();
        poseBase.Pos().Z() += 0.015;
        auto poseEETip2World = this->model_->GetLink(eeLinkName_)->WorldPose();
        auto pose = poseBase.Inverse() * poseEETip2World;
        auto quat = pose.Rot();

        double q1, q2;
        if (pow(quat.W(), 2) - pow(quat.X(), 2) - pow(quat.Y(), 2) + pow(quat.Z(), 2) != 0){
            q1 = atan2(-2 * (quat.W() * quat.Y() - quat.X() * quat.Z()),
                       (pow(quat.W(), 2) - pow(quat.X(), 2) - pow(quat.Y(), 2) + pow(quat.Z(), 2)));
        }

        if (q1 < -M_PI_2){
            q1 += M_PI;
        } else if (q1 > M_PI_2){
            q1 -= M_PI;
        }

        q2 = asin(ignition::math::clamp<double>((quat.W() * quat.X() - quat.Y(), quat.Z()) * 2, -1, 1));

        this->jointController_->SetPositionTarget(jointName1_, q1);
        this->jointController_->SetPositionTarget(jointName2_, q2);
        this->jointController_->Update();

        universalJointState_.header.stamp = ros::Time::now();
        universalJointState_.position[0] = this->model_->GetJoint(jointName1_)->Position();
        universalJointState_.position[1] = this->model_->GetJoint(jointName2_)->Position();
        jointStatePub_.publish(universalJointState_);

        debugCounter_ +=1;
        if (robotNamespace_ == "/iiwa_front/" && debugCounter_ > 100){
            debugCounter_ = 0;
            ROS_DEBUG_STREAM("[Desired ]" << " q1: " << q1 << " q2: " << q2);
            ROS_DEBUG_STREAM("[Actual  ]" << " q1: " << this->model_->GetJoint(jointName1_)->Position() << " q2: " << this->model_->GetJoint(jointName2_)->Position());
            ROS_DEBUG_STREAM("[Command ]" << " q1: " << this->jointController_->GetPositionPIDs()[jointName1_].GetCmd());
            ROS_DEBUG_STREAM("[Command Out]" << " q1: " << this->model_->GetJointController()->GetVelocityPIDs()[jointName1_].GetCmd() << " q2: " << this->jointController_->GetPositionPIDs()[jointName2_].GetCmd());
            ROS_DEBUG_STREAM("[Velocity]" << " q1: " << this->jointController_->GetVelocities()[jointName1_] << " q2: " << this->jointController_->GetVelocities()[jointName2_]);
        }
    }
}
