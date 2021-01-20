//
// Created by puze on 18.01.21.
//

#include "universal_joint_plugin.h"

using namespace gazebo;

UniversalJointPlugin::UniversalJointPlugin() {

}

UniversalJointPlugin::~UniversalJointPlugin() {

}

void UniversalJointPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    ROS_INFO_STREAM_NAMED("Universal Joint Plugin" , "Start Universal Joint Plugin");
    this->model_ = _model;
    this->world_ = model_->GetWorld();
    this->robotNamespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();

    if (this->robotNamespace_ == "/iiwa_front/"){
        robotBaseLinkName_ = "front_base";
        eeLinkName_ = "F_link_ee";
        jointName1_ = "iiwa_front::F_striker_joint_1";
        jointName2_ = "iiwa_front::F_striker_joint_2";
    } else if (this->robotNamespace_ == "/iiwa_back/"){
        robotBaseLinkName_ = "back_base";
        eeLinkName_ = "B_link_ee";
        jointName1_ = "iiwa_back::B_striker_joint_1";
        jointName2_ = "iiwa_back::B_striker_joint_2";
    } else {
        ROS_ERROR_STREAM_NAMED("Universal Joint Plugin" , "No namespace for the iiwa. Use /iiwa_front or /iiwa_back.");
    }

    gazebo::common::PID pid(1, 0, 0.0);
    this->model_->GetJointController()->SetPositionPID(jointName1_, pid);
    this->model_->GetJointController()->SetVelocityPID(jointName1_, gazebo::common::PID(0, 0, 0));
    this->model_->GetJointController()->SetPositionPID(jointName2_, pid);
    this->model_->GetJointController()->SetVelocityPID(jointName2_, gazebo::common::PID(0, 0, 0));
    debugCounter_ = 0;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin (boost::bind ( &UniversalJointPlugin::OnUpdate, this));
}

void UniversalJointPlugin::OnUpdate() {
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

    this->model_->GetJointController()->SetPositionTarget(jointName1_, q1);
    this->model_->GetJointController()->SetPositionTarget(jointName2_, q2);
    model_->GetJointController()->Update();
//    debugCounter_ +=1;
//    if (robotNamespace_ == "/iiwa_front/" && debugCounter_ > 100){
//        debugCounter_ = 0;
//        ROS_INFO_STREAM("[Desired ]" << " q1: " << q1 << " q2: " << q2);
//        ROS_INFO_STREAM("[Actual  ]" << " q1: " << model_->GetJoint(jointName1_)->Position(0) << " q2: " << model_->GetJoint(jointName2_)->Position(0));
//        ROS_INFO_STREAM("[Command ]" << " q1: " << model_->GetJoint(jointName1_)->GetForce(0) << " q2: " << model_->GetJoint(jointName2_)->GetForce(0));
//        ROS_INFO_STREAM("[Velocity]" << " q1: " << model_->GetJoint(jointName1_)->GetVelocity(0) << " q2: " << model_->GetJoint(jointName2_)->GetVelocity(0));
//    }

}
