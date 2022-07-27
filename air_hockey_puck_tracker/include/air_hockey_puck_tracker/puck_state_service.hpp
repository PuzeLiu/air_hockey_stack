#include "ros/ros.h"
#include "air_hockey_puck_tracker/GetPuckState.h"
#include "air_hockey_puck_tracker/PuckTracker.hpp"


using namespace air_hockey_puck_tracker;
using namespace air_hockey_baseline_agent;

class PuckStateHandler {
public:
    PuckStateHandler(ros::NodeHandle nh) {
        nh_ = nh;
        pt_ = new PuckTracker(nh, 0.0);
        pt_->start();
    }

    bool getPuckState(GetPuckState::Request  &req,
                      GetPuckState::Response &res)
    {
        float time = req.time;
        pps_ = pt_->getPredictedState(true, false, false, time);
        ros::Time stamp = pps_.stamp;
        PuckState state = pps_.state;
        res.prediction.header.stamp = pps_.stamp;
        res.prediction.header.frame_id = pps_.frame_id;
        res.prediction.predictionTime = time;
        res.prediction.x = state.x();
        res.prediction.y = state.y();
        res.prediction.dx = state.dx();
        res.prediction.dy = state.dy();
        res.prediction.theta = state.theta();
        res.prediction.dtheta = state.dtheta();
        return true;
    }

private:
    ros::NodeHandle nh_;
    PuckTracker* pt_;
    PuckPredictedState pps_;
};
