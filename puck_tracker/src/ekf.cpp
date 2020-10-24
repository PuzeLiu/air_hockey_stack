/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
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


#include "ekf.h"


EKF::EKF(const DynamicsParam& dynamicsParam_, ModelNoise& modelNoise_, double deltaTime_):
          m_dynaParam(dynamicsParam_), m_modelNoise(modelNoise_)
{
	KF_options.method = kfEKFNaive;

    // Initial State
	m_xkk.resize(get_vehicle_size());

	// Initial cov
	m_pkk.setIdentity(4);

    hasInitObservation = false;

    m_deltaTime = deltaTime_;
}

EKF::~EKF() {}

void EKF::OnGetAction(KFArray_ACT& out_u) const { out_u[0] = m_deltaTime;}

void EKF::OnTransitionModel(const KFArray_ACT &in_u, KFArray_VEH &inout_x, bool &out_skipPrediction) const {
    inout_x[0] += in_u[0] * inout_x[2];
    inout_x[1] += in_u[0] * inout_x[3];
    inout_x[2] -= in_u[0] * (m_dynaParam.mu_air / m_dynaParam.m * inout_x[2]);
    inout_x[3] -= in_u[0] * (m_dynaParam.mu_air / m_dynaParam.m * inout_x[3]);
    double v = sqrt(pow(inout_x[2], 2) + pow(inout_x[3], 2));
    if (v>0.){
        inout_x[2] -= in_u[0] * m_dynaParam.mu_lateral * m_dynaParam.g * inout_x[2] / v;
        inout_x[3] -= in_u[0] * m_dynaParam.mu_lateral * m_dynaParam.g * inout_x[3] / v;
    }
}

void EKF::OnTransitionJacobian(KFMatrix_VxV &out_F) const
{
    out_F.setIdentity();

    out_F(0, 2) = m_deltaTime;
    out_F(1, 3) = m_deltaTime;
    out_F(2, 2) = 1 - m_deltaTime * m_dynaParam.mu_air / m_dynaParam.m;
    out_F(3, 3) = 1 - m_deltaTime * m_dynaParam.mu_air / m_dynaParam.m;
}

void EKF::OnTransitionNoise(KFMatrix_VxV &out_Q) const {
    out_Q(0, 0) = out_Q(1, 1) = pow(m_modelNoise.dynamic_model_std_xy, 2.0);
    out_Q(2, 2) = out_Q(3, 3) = pow(m_modelNoise.dynamic_model_std_vxy, 2.0);
}

void EKF::OnGetObservationNoise(KFMatrix_OxO &out_R) const {
    out_R(0, 0) = out_R(1, 1) = pow(m_modelNoise.sensor_noise_std_xy, 2.0);
    out_R(2, 2) = out_R(3, 3) = pow(m_modelNoise.sensor_noise_std_vxy, 2.0);
}

void EKF::OnGetObservationsAndDataAssociation(
        vector_KFArray_OBS& out_z, std::vector<int>& out_data_association,
        const vector_KFArray_OBS& in_all_predictions, const KFMatrix& in_S,
        const std::vector<size_t>& in_lm_indices_in_S,
        const KFMatrix_OxO& in_R) {

    out_z.resize(1);
    out_z[0] = m_okk;
    out_data_association.clear();
}

void EKF::OnObservationModel(const std::vector<size_t> &idx_landmarks_to_predict,
                             vector_KFArray_OBS &out_predictions) const {
    out_predictions.resize(1);
    out_predictions[0][0] = m_xkk[0];
    out_predictions[0][1] = m_xkk[1];
    out_predictions[0][2] = m_xkk[2];
    out_predictions[0][3] = m_xkk[3];
}

void EKF::OnObservationJacobians(size_t idx_landmark_to_predict, KFMatrix_OxV &Hx, KFMatrix_OxF &Hy) const {
    Hx.setIdentity();
}

void EKF::getObservation() {
    try{
        m_listener.lookupTransform("/world", "/Puck", ros::Time(0), m_tfTmp);
        double dt = m_tfTmp.stamp_.toSec() - m_tfPrev.stamp_.toSec();
        if (m_deltaTime != 0){
            m_listener.lookupTwist("/Puck", "/world", ros::Time(0),
                                   ros::Duration(dt), m_twist);
        }
        m_tfPrev = m_tfTmp;
    }
    catch (tf::TransformException& ex) {
        return;
    }

    m_okk[0] = m_tfTmp.getOrigin().x();
    m_okk[1] = m_tfTmp.getOrigin().y();
    m_okk[2] = m_twist.linear.x;
    m_okk[3] = m_twist.linear.y;
}

void EKF::doProcess() {
    getObservation();
    runOneKalmanIteration();
}

void EKF::init() {
    while (!hasInitObservation){
        try {
            m_listener.lookupTransform("/world", "/Puck", ros::Time(0), m_tfTmp);
            m_tfPrev = m_tfTmp;
            hasInitObservation = true;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN_ONCE("Waiting for TF Information");
            continue;
        }
    }
}
