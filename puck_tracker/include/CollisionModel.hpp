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

#ifndef PUCK_TRACKER_COLLISIONMODEL_HPP
#define PUCK_TRACKER_COLLISIONMODEL_HPP

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "SystemModel.hpp"


using namespace std;

namespace AirHockey {

    static T cross2D(Vector2 v1, Vector2 v2) {
        return v1.x() * v2.y() - v1.y() * v2.x();
    }

    class AirHockeyTable {
    public:

        //! BoundaryType of 4 lines, each row is line between <x1, y1, x2, y2>
        typedef Kalman::Matrix<T, 4, 4> BoundaryType;
    protected:
        Vector2 m_center;
        T m_z;
        T m_yaw;

        //! Table length: X-direction, width: Y-direction
        T m_length;
        T m_width;

        //! step size
        T m_dt;

        //! radius of the puck
        T m_puckRadius;
        //! restitution coefficient
        T m_e;

        BoundaryType m_boundary;

    public:
        AirHockeyTable(T e, T dt = 1 / 120.) {
            m_center.setZero();
            m_yaw = 0.;
            m_length = 1.96;
            m_width = 1.04;
            m_puckRadius = 0.03165;

            m_e = e;
            m_dt = dt;
        }

        ~AirHockeyTable() {
        }

        void setTransform(const tf::StampedTransform &transform) {
            m_center.x() = transform.getOrigin().x();
            m_center.y() = transform.getOrigin().y();
            m_z = transform.getOrigin().z();
            double cos_theta = transform.getRotation().w();
            double sin_theta = transform.getRotation().z();
            m_yaw = std::atan2(sin_theta, cos_theta);

            Kalman::Matrix<T, 2, 2> rotation;
            rotation(0, 0) = cos(m_yaw);
            rotation(0, 1) = sin(m_yaw);
            rotation(1, 0) = -sin(m_yaw);
            rotation(1, 1) = cos(m_yaw);

            Vector2 vecTmp, offsetP1, offsetP2, offsetP3, offsetP4;
            vecTmp << -(m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
            offsetP1 = m_center + rotation * vecTmp;
            vecTmp << -(m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
            offsetP2 = m_center + rotation * vecTmp;
            vecTmp << (m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
            offsetP3 = m_center + rotation * vecTmp;
            vecTmp << (m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
            offsetP4 = m_center + rotation * vecTmp;
            m_boundary.row(0) << offsetP1.x(), offsetP1.y(), offsetP3.x(), offsetP3.y();
            m_boundary.row(1) << offsetP3.x(), offsetP3.y(), offsetP4.x(), offsetP4.y();
            m_boundary.row(2) << offsetP4.x(), offsetP4.y(), offsetP2.x(), offsetP2.y();
            m_boundary.row(3) << offsetP2.x(), offsetP2.y(), offsetP1.x(), offsetP1.y();
        }

        inline Vector2 getCenter() { return m_center; }

        inline T getYaw() { return m_yaw; }

        inline BoundaryType getBoundary() { return m_boundary; }

        bool applyCollision(EKF::State &state) {
            Vector2 p, vel;
            p.x() = state.x();
            p.y() = state.y();
            vel.x() = state.dx();
            vel.y() = state.dy();
            double theta = state.theta();
            double dtheta = state.dtheta();

            for (int i = 0; i < m_boundary.rows(); ++i) {
                Vector2 p1 = m_boundary.block<1, 2>(i, 0);
                Vector2 p2 = m_boundary.block<1, 2>(i, 2);

                Vector2 u = vel * m_dt;
                Vector2 v = p2 - p1;
                Vector2 w = p1 - p;

                Eigen::Rotation2D<double> rot(M_PI_2);
                Vector2 vecT = v / v.norm();
                Vector2 vecN =  rot * vecT;

                T denominator = cross2D(v, u);
                if (abs(denominator) < 1e-6) {
                    // parallel
                    continue;
                } else {
                    T s = cross2D(v, w) / denominator;
                    T r = cross2D(u, w) / denominator;
                    if (cross2D(w, v) < 0 || s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4 && r <= 1 - 1e-4) {
                        double vtScalar = vel.dot(vecT);
                        double vnScalar = vel.dot(vecN);

                        // Velocity on next time step
                        double vtNextScalar = 2./3. * vtScalar - m_puckRadius / 5 * dtheta; // - m_puckRadius / 3 * dtheta;
                        double vnNextScalar = - m_e * vnScalar;
                        double dthetaNext = 1. / 3. * dtheta - 2. / (3. * m_puckRadius) * vnScalar;
                        Vector2 vNext = vnNextScalar * vecN + vtNextScalar * vecT;

                        // Position of intersection point
                        Vector2 pInter = p + s * u;
                        // Angular Position of intersection point
                        double thetaInter = theta + s * dtheta * m_dt;
                        // Position of next point
                        Vector2 pNext = pInter + (1 - s) * vNext * m_dt;
                        // Angular Position of next point
                        double thetaNext = thetaInter + (1 - s) * dthetaNext * m_dt;

                        state.x() = pNext.x();
                        state.y() = pNext.y();
                        state.dx() = vNext.x();
                        state.dy() = vNext.y();
                        state.theta() = thetaNext;
                        state.dtheta() = dthetaNext;
                        return true;
                    }
                }
            }
            return false;
        }
    };

    class Mallet {
    public:
        //! radius of the mallet
        T m_radiusMallet;

        //! radius of the puck
        T m_radiusPuck;

        //! time step
        T m_dt;
        //! time stamp of previous update
        double t_prev;
        //! restitution coefficient
        T m_e;

        State m_malletState, m_malletStatePredict;

        Mallet(T e, T dt) {
            m_dt = dt;
            t_prev = 0.;
            m_malletState.setZero();
            m_malletStatePredict.setZero();

            m_radiusPuck = 0.03165;
            m_radiusMallet = 0.04815;
            m_e = 0.5;
        }

        void setState(const tf::StampedTransform &tfMallet) {
            double deltaT = tfMallet.stamp_.toSec() - t_prev;

            m_malletState.dx() = (tfMallet.getOrigin().x() - m_malletState.x()) / deltaT;
            m_malletState.dy() = (tfMallet.getOrigin().y() - m_malletState.y()) / deltaT;
            m_malletState.dtheta() = (tf::getYaw(tfMallet.getRotation()) - m_malletState.theta()) / deltaT;

            m_malletState.x() = tfMallet.getOrigin().x();
            m_malletState.y() = tfMallet.getOrigin().y();
            m_malletState.theta() = tf::getYaw(tfMallet.getRotation());
            t_prev = tfMallet.stamp_.toSec();
            m_malletStatePredict = m_malletState;
        }

        void predict(){
            m_malletStatePredict.x() = m_malletStatePredict.x() + m_malletStatePredict.dx() * m_dt;
            m_malletStatePredict.y() = m_malletStatePredict.y() + m_malletStatePredict.dy() * m_dt;
            m_malletStatePredict.theta() = m_malletStatePredict.theta() + m_malletStatePredict.dtheta() * m_dt;
        }

        bool applyCollision(State &puckState) {
            Vector2 pPuck = puckState.block<2, 1>(0, 0);
            Vector2 vPuck = puckState.block<2, 1>(2, 0);
            Vector2 pPuckNext = pPuck + vPuck * m_dt;

            Vector2 pMallet = m_malletStatePredict.block<2, 1>(0, 0);
            Vector2 vMallet = m_malletStatePredict.block<2, 1>(2, 0);
            Vector2 pMalletNext = pMallet + vMallet * m_dt;

            Vector2 vRelative = m_dt * (vMallet - vPuck);
            Vector2 pRelative = pMallet - pPuck;

            double distToGo = pRelative.norm() - m_radiusPuck - m_radiusMallet;
            double t = distToGo / (vRelative.dot(pRelative) / pRelative.norm());

            if (t>0 && t<1){
                Vector2 pPuckCollide = pPuck + t * vPuck * m_dt;
                Vector2 pMalletCollide = pMallet + t * vMallet * m_dt;

                Vector2 vNorm = pPuckCollide - pMalletCollide;

                Eigen::Rotation2D<double> rot(-M_PI_2);
                Vector2 vecN = vNorm / vNorm.norm();
                Vector2 vecT = rot * vecN;
                double vtScalarPuck = vPuck.dot(vecT);
                double vnScalarPuck = vPuck.dot(vecN);
                double vtScalarMallet = vMallet.dot(vecT);
                double vnScalarMallet = vMallet.dot(vecN);

                double vtScalarPuckNext = 2. / 3. * vtScalarPuck + 1. / 3. * vtScalarMallet;
                double vnScalarPuckNext = -m_e * vnScalarPuck + (1 + m_e) * vnScalarMallet;

                Vector2 vPuckNext = vtScalarPuckNext * vecT + vnScalarPuckNext * vecN;
                //! Update the state
                puckState.block<2, 1>(0, 0) = pPuckCollide + (1 - t) * vPuckNext * m_dt;
                puckState.block<2, 1>(2, 0) = vPuckNext;
                return true;
            }
            return false;
        }
    };

    class CollisionModel {
    public:

        AirHockeyTable m_table;

        Mallet m_mallet;

        CollisionModel(tf::StampedTransform &tfTable, T &restitutionTable, T &restitutionMallet, T dt) :
            m_table(restitutionTable, dt), m_mallet(restitutionMallet, dt) {
            m_table.setTransform(tfTable);
        }

        bool applyCollision(State &puckState) {
            m_table.applyCollision(puckState);
            return (m_mallet.applyCollision(puckState));
        }

    };

}

#endif //PUCK_TRACKER_COLLISIONMODEL_HPP