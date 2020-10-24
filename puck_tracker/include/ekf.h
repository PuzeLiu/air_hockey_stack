/*
 * MIT License
 * Copyright (c) 2020 Davide Tateo, Puze Liu
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


#ifndef AIR_HOCKEY_STACK_PUCK_TRACKER_DYNAMICS_H_
#define AIR_HOCKEY_STACK_PUCK_TRACKER_DYNAMICS_H_

#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

using namespace mrpt::bayes;
using namespace mrpt::math;
using namespace std;

struct DynamicsParam{
    double mu_air; // air drag frction
    double mu_lateral; // lateral friction coefficient
    double m = 0.01; // mass
    double g = 9.81; // gravitational acceleration
};

struct ModelNoise{
    double dynamic_model_std_xy;
    double dynamic_model_std_vxy;
    double sensor_noise_std_xy;
    double sensor_noise_std_vxy;
};

class EKF : public mrpt::bayes::CKalmanFilterCapable
        <4 /* x y vx vy */, 4 /* x y vx vy */, 0, 1 /* Atime */> {
public:
	EKF(const DynamicsParam& dynamicsParam_, ModelNoise& modelNoise_, double deltaTime_);
	~EKF() override;
	void getState(KFVector& xkk, KFMatrix& pkk){
	    xkk = m_xkk;
	    pkk = m_pkk;
	};

	void doProcess();

    void init();

protected:
    bool hasInitObservation;
    double m_deltaTime;
    DynamicsParam m_dynaParam;
    ModelNoise m_modelNoise;
    KFArray_OBS m_okk;
    tf::TransformListener m_listener;

    tf::StampedTransform m_tfTmp;
    tf::StampedTransform m_tfPrev;
    geometry_msgs::Twist m_twist;

    void OnGetAction(KFArray_ACT& out_u) const override;

    void OnTransitionModel(const KFArray_ACT& in_u, KFArray_VEH& inout_x,
            bool& out_skipPrediction) const override;

    void OnTransitionJacobian(KFMatrix_VxV& out_F) const override;

    void OnTransitionNoise(KFMatrix_VxV& out_Q) const override;

    void OnGetObservationNoise(KFMatrix_OxO& out_R) const override;

    void OnGetObservationsAndDataAssociation(
            vector_KFArray_OBS& out_z, std::vector<int>& out_data_association,
            const vector_KFArray_OBS& in_all_predictions, const KFMatrix& in_S,
            const std::vector<size_t>& in_lm_indices_in_S,
            const KFMatrix_OxO& in_R) override;

    void OnObservationModel(
            const std::vector<size_t>& idx_landmarks_to_predict,
            vector_KFArray_OBS& out_predictions) const override;

    void OnObservationJacobians(
            [[maybe_unused]] size_t idx_landmark_to_predict,
            [[maybe_unused]] KFMatrix_OxV& Hx,
            [[maybe_unused]] KFMatrix_OxF& Hy) const override;

    void getObservation();

};

#endif /* AIR_HOCKEY_STACK_PUCK_TRACKER_SRC_DYNAMICS_H_ */
