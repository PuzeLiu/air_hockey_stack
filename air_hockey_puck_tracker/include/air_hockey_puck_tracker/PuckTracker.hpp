/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SRC_PUCK_TRACKER_HPP
#define SRC_PUCK_TRACKER_HPP

#include <boost/thread.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "SystemModel.hpp"
#include "ObservationModel.hpp"
#include "CollisionModel.hpp"
#include "EKF_Wrapper.hpp"
#include "VisualizationInterface.hpp"
#include "ParticleFilter.hpp"
#include "ParticleVisualizationInterface.hpp"

namespace air_hockey_baseline_agent {
    struct PuckPredictedState{
        ros::Time stamp;
        PuckState state;
        int numOfCollisions;
        double predictedTime;
    };

    class PuckTracker {
    private: friend class VisualizationInterface;
    public:
        PuckTracker(ros::NodeHandle nh, double defendLine=0.0);

        ~PuckTracker();

        void start();

        const PuckPredictedState& getPredictedState(bool visualize = true, bool delayed=false);

        const PuckState& getEstimatedState(bool visualize = false);

        void reset();

        inline double getMaxPredictionTime() {
        	return maxPredictionSteps_ * rate_->expectedCycleTime().toSec();
        }

    private:
        void loadParams();

        void setCovariance();

        void startTracking();

        void getPrediction(double &predictedTime, int &nCollision);

	    bool getMeasurement();

	    bool updateOpponentMallet();

	    bool checkGating();

    private:
        ros::NodeHandle nh_;
        ros::Rate* rate_;
        ros::Time stamp_;

        tf2_ros::TransformListener tfListener_;
        tf2_ros::Buffer tfBuffer_;
        std::string opponentMalletName_, tableRefName_;
        geometry_msgs::TransformStamped tfPuck_, tfOpponentMallet_;

        EKF_Wrapper *kalmanFilter_;
        EKF_Wrapper *puckPredictor_;
        SystemModel *systemModel_;
        ObservationModel *observationModel_;
        CollisionModel *collisionModel_;
        VisualizationInterface *visualizer_;
        ParticleFilter *particleFilter_;
        ParticleVisualizationInterface *particleVisualizationInterface_;

        Control u_;
        int maxPredictionSteps_;
        Measurement measurement_;

        PuckPredictedState predictedState_;
        double resetThreshold;
        double defendingLine_;

        bool doPrediction_;
        bool useParticleFilter_;

        boost::thread thread_;

        std::vector<PuckState> stateBuffer_;
    };
}

#endif //SRC_PUCK_TRACKER_HPP
