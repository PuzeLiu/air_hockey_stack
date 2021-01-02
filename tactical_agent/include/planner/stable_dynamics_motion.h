#ifndef SRC_STABLE_DYNAMICS_MOTION_H
#define SRC_STABLE_DYNAMICS_MOTION_H

#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <boost/algorithm/clamp.hpp>

using namespace Eigen;

namespace tactical_agent{
class StableDynamicsMotion {
public:
    StableDynamicsMotion(Vector2d stiffnessUpperBound, Vector2d stiffnessLowerBound, double rate, double height);
    ~StableDynamicsMotion();

    bool plan(const Vector2d& xCur, const Vector2d& vCur, const Vector2d& xGoal,
              trajectory_msgs::MultiDOFJointTrajectory& cartTraj, double duration = 0.1);

    void scaleStiffness(const Vector2d& scale);
    void setStiffness(const Vector2d& stiffness);

private:
    Vector2d stiffnessUpperBound_, stiffnessLowerBound_;
    Vector2d stiffness_, damping_;
    double stepSize_;
    double height_;

    Vector2d xTmp_, vTmp_;

    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
};
}

#endif //SRC_STABLE_DYNAMICS_MOTION_H
