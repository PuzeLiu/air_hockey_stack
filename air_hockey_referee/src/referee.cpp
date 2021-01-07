//
// Created by puze on 07.01.21.
//

#include "air_hockey_referee/referee.h"

AirHockey::Referee::Referee(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_), air_hockey_table_(1.0, 1/120) {
    statusPub_ = nh_.advertise<std_msgs::Int8>("status", 1);
    try {
        tfTable_ = tfBuffer_.lookupTransform("world", "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception){
        ROS_ERROR_STREAM("Could not transform from world to Table: " << exception.what());
        ros::shutdown();
    }
    air_hockey_table_.setTransform(tfTable_);

    status_ = GameStatus::STOP;
    stampPrev_ = ros::Time::now();
}

AirHockey::Referee::~Referee() {

}

void AirHockey::Referee::update() {
    tfPuck_ = tfBuffer_.lookupTransform("world", "Puck", ros::Time(0), ros::Duration(1.0));

    if (ros::Time::now() - tfPuck_.header.stamp > ros::Duration(1.0)){
        std_msgs::Int8 msg;
        status_ = GameStatus::STOP;
        msg.data = status_;
        statusPub_.publish(msg);
    } else if (tfPuck_.header.stamp - stampPrev_ > ros::Duration(0.0)){
        Measurement measurement;

        measurement.x() = tfPuck_.transform.translation.x;
        measurement.y() = tfPuck_.transform.translation.y;
        tf2::Quaternion quat;
        quat.setX(tfPuck_.transform.rotation.x);
        quat.setY(tfPuck_.transform.rotation.y);
        quat.setZ(tfPuck_.transform.rotation.z);
        quat.setW(tfPuck_.transform.rotation.w);
        tf2::Matrix3x3 rotMat(quat);
        double roll, pitch, yaw;
        rotMat.getEulerYPR(yaw, pitch, roll);
        measurement.theta() = yaw;

        if (status_ == GameStatus::START && air_hockey_table_.isOutsideBoundary(measurement)){
            std_msgs::Int8 msg;
            status_ = GameStatus::GOAL;
            msg.data = status_;
            statusPub_.publish(msg);
        } else if (status_ != GameStatus::START && !air_hockey_table_.isOutsideBoundary(measurement)){
            std_msgs::Int8 msg;
            status_ = GameStatus::START;
            msg.data = status_;
            statusPub_.publish(msg);
        }
    }
}


