//
// Created by puze on 08.12.20.
//
#include <ros/ros.h>
#include "bezier_curve.h"
#include "null_space_optimization/CartersianTrajectory.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bezier_planner");
    ros::NodeHandle nh("/");
    ros::Rate rate(100);

    ros::Publisher publisher = nh.advertise<null_space_optimization::CartersianTrajectory>("cartesian_command", 1);
    Vector2d boundLower, boundUpper;
    boundLower << 0.5, -0.5;
    boundUpper << 2.46, 0.5;

    BezierCurve2D bezierCurve2D(boundLower, boundUpper, 0.3);

    Vector2d xStart, xHit, vHit;
    xStart << 0.6, 0.0;
    xHit << 0.7, 0.2;
    vHit << 1.5, 0.0;
    bezierCurve2D.fit(xStart, xHit, vHit);
    ROS_INFO_STREAM("Hitting Time: " << bezierCurve2D.getHitTime() << " Stop Time: " << bezierCurve2D.getStopTime());

    double t = 0.;
    null_space_optimization::CartersianTrajectory msg;
    msg.time_step_size = rate.expectedCycleTime().toSec();
    while (t <= bezierCurve2D.getStopTime() && ros::ok()){
        bezierCurve2D.getPoint(t, msg);
        msg.header.stamp = ros::Time::now();
        publisher.publish(msg);
        t += rate.expectedCycleTime().toSec();
        rate.sleep();
    }
    ROS_INFO_STREAM("FINISH");
    return 0;
}
