#include "universal_joint_plugin.h"

using namespace gazebo;
using namespace Eigen;

constexpr double pi() { return std::atan(1)*4;}

UniversalJointPlugin::UniversalJointPlugin():nh_("/"){

}

UniversalJointPlugin::~UniversalJointPlugin() {

}

void UniversalJointPlugin::Load(gazebo::physics::ModelPtr _model,
		sdf::ElementPtr _sdf) {
	ROS_INFO_STREAM_NAMED("Universal Joint Plugin",
			"Start Universal Joint Plugin");
	this->model_ = _model;
	this->world_ = model_->GetWorld();
	this->robotNamespace_ =
			_sdf->GetElement("robotNamespace")->Get<std::string>();

	std::string prefix;
	if (this->robotNamespace_ == "/iiwa_front/") {
//		jointStatePub_ = nh_.advertise<sensor_msgs::JointState>(
//				this->robotNamespace_ + "joint_states", 1);
		prefix = "F";
		jointName1_ = "iiwa_front::F_striker_joint_1";
		jointName2_ = "iiwa_front::F_striker_joint_2";
		universalJointState_.name.push_back("F_striker_joint_1");
		universalJointState_.name.push_back("F_striker_joint_2");
	} else if (this->robotNamespace_ == "/iiwa_back/") {
//		jointStatePub_ = nh_.advertise<sensor_msgs::JointState>(
//				this->robotNamespace_ + "joint_states", 1);
		prefix = "B";
		jointName1_ = "iiwa_back::B_striker_joint_1";
		jointName2_ = "iiwa_back::B_striker_joint_2";
		universalJointState_.name.push_back("B_striker_joint_1");
		universalJointState_.name.push_back("B_striker_joint_2");
	} else {
		ROS_ERROR_STREAM_NAMED("Universal Joint Plugin",
				"No namespace for the iiwa. Use /iiwa_front or /iiwa_back.");
	}

	std::string description_xml;
//	if(!nh_.getParam(robotNamespace_ + "iiwa_only_description", description_xml)){
	if (!nh_.getParam(robotNamespace_ + "robot_description", description_xml)) {
		ROS_ERROR_STREAM_NAMED("Universal Joint Plugin", robotNamespace_ << " did not find the urdf model");
	}
//	}

	pinocchio::urdf::buildModelFromXML(description_xml, pinoModel_);
	pinoData_ = pinocchio::Data(this->pinoModel_);
	frame_id = pinoModel_.getFrameId(prefix + "_link_ee");

	iiwaJointNames_.push_back(prefix + "_joint_1");
	iiwaJointNames_.push_back(prefix + "_joint_2");
	iiwaJointNames_.push_back(prefix + "_joint_3");
	iiwaJointNames_.push_back(prefix + "_joint_4");
	iiwaJointNames_.push_back(prefix + "_joint_5");
	iiwaJointNames_.push_back(prefix + "_joint_6");
	iiwaJointNames_.push_back(prefix + "_joint_7");
	qCur_.resize(pinoModel_.nq);
	qCur_.setZero();

    pinocchio::forwardKinematics(pinoModel_, pinoData_, qCur_);
    pinocchio::updateFramePlacements(pinoModel_, pinoData_);
    base_pos = pinoData_.oMf[frame_id].translation();

	universalJointState_.position.resize(2);
	universalJointState_.velocity.push_back(0.);
	universalJointState_.velocity.push_back(0.);
	universalJointState_.effort.push_back(0.);
	universalJointState_.effort.push_back(0.);

	debugCounter_ = 0;
	started_ = false;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&UniversalJointPlugin::OnUpdate, this));
}

void UniversalJointPlugin::OnUpdate() {
	if (!started_) {

		this->jointController_.reset(
				new physics::JointController(this->model_));
		gazebo::common::PID pid(40, 0.1, 0.01);

		this->jointController_->AddJoint(model_->GetJoint(jointName1_));
		this->jointController_->SetPositionPID(jointName1_, pid);
		this->jointController_->AddJoint(model_->GetJoint(jointName2_));
		this->jointController_->SetPositionPID(jointName2_, pid);
		started_ = true;
	} else {
		if (this->model_->GetJoints().size() != 0) {
			for (int i = 0; i < iiwaJointNames_.size(); ++i) {
				qCur_[i] = this->model_->GetJoint(iiwaJointNames_[i])->Position();
			}

			pinocchio::forwardKinematics(pinoModel_, pinoData_, qCur_);
			pinocchio::updateFramePlacements(pinoModel_, pinoData_);

			auto rot_mat = pinoData_.oMf[frame_id].rotation();
			auto pos = pinoData_.oMf[frame_id].translation();

			auto v_x = rot_mat.col(0);
            auto v_y = rot_mat.col(1);

            // Compute y rotation, joint 8
            Eigen::Vector2d n_y;
            n_y << v_y(0), v_y(1);
            auto a = v_y[1] / n_y.norm();
            auto b = v_y[0] / n_y.norm();

            Eigen::Vector3d target_x;
            target_x << a, -b, 0;

            auto q1 = acos(v_x.dot(target_x));

            if (q1 > pi() / 2) {
                target_x *= -1.;
                q1 = acos(v_x.dot(target_x));
            }

            // Rotate x by y rotation
            Eigen::Matrix3d w;
            w << 0., -v_y[2], v_y[1],
                v_y[2], 0., -v_y[0],
                -v_y[1], v_y[0], 0.;

            auto r = Eigen::Matrix3d::Identity() + w * sin(q1) + w.cwiseProduct(w) * (1 - cos(q1));
            v_x = r * v_x;

            // Compute x rotation, joint 9
            Eigen::Vector2d n_x;
            n_x << v_x(0), v_x(1);
            a = v_x[1] / n_x.norm();
            b = v_x[0] / n_x.norm();

            Eigen::Vector3d target_y;
            target_y << a, -b, 0;

            auto q2 = acos(v_y.dot(target_y)); // skipped rounding to 4 decimal places

            if (q2 > pi() / 2) { // possible typo
                target_y *= -1.;
                q2 = acos(v_y.dot(target_y)); // skipped rounding to 4 decimal places
            }

            // Adjust the sign of the angle based on the y position of the ee
            q2 = (pos(1) - base_pos(1) > 0.) ? q2 : -q2;


			this->jointController_->SetPositionTarget(jointName1_, q1);
			this->jointController_->SetPositionTarget(jointName2_, q2);
			this->jointController_->Update();

			universalJointState_.header.stamp = ros::Time::now();
			universalJointState_.position[0] = this->model_->GetJoint(
					jointName1_)->Position();
			universalJointState_.position[1] = this->model_->GetJoint(
					jointName2_)->Position();
//			jointStatePub_.publish(universalJointState_);

		}
	}
}
