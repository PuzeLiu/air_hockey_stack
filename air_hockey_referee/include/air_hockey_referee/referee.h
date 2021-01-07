//
// Created by puze on 07.01.21.
//

#ifndef SRC_REFEREE_H
#define SRC_REFEREE_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/transform_listener.h>
#include "air_hockey_puck_tracker/CollisionModel.hpp"

namespace AirHockey{
    enum GameStatus {
        STOP = 0,
        START = 1,
        GOAL = 2,
    };
class Referee {
public:
    Referee(ros::NodeHandle nh);
    ~Referee();

    void update();
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Publisher statusPub_;

    geometry_msgs::TransformStamped tfTable_, tfPuck_;

    AirHockeyTable air_hockey_table_;

    int status_;
    ros::Time stampPrev_;
};
}

#endif //SRC_REFEREE_H
