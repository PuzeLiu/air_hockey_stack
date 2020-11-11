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
        //! friction coefficient
        T m_mu;

        BoundaryType m_boundary;

    public:
        AirHockeyTable(T e, T mu, T dt = 1 / 120.) {
            m_center.setZero();
            m_yaw = 0.;
            m_length = 1.96;
            m_width = 1.04;
            m_puckRadius = 0.03165;

            m_e = e;
            m_mu = mu;
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
            m_boundary.row(0) << offsetP1.x(), offsetP1.y(), offsetP2.x(), offsetP2.y();
            m_boundary.row(1) << offsetP3.x(), offsetP3.y(), offsetP4.x(), offsetP4.y();
            m_boundary.row(2) << offsetP1.x(), offsetP1.y(), offsetP3.x(), offsetP3.y();
            m_boundary.row(3) << offsetP2.x(), offsetP2.y(), offsetP4.x(), offsetP4.y();
        }

        inline Vector2 getCenter() { return m_center; }

        inline T getYaw() { return m_yaw; }

        inline BoundaryType getBoundary() { return m_boundary; }

        void applyCollision(EKF::State &state) {
            Vector2 p = state.block<2, 1>(0, 0);
            Vector2 v = state.block<2, 1>(2, 0);

            //! closest point parameter: p = p0 + t(p1 - p0)
            Vector2 pNext, vNext, pInter;
            if (isColliding(p, v, pNext, vNext, pInter)) {
                //! check if second collision happens
                Vector2 pReflect = pInter;
                Vector2 vReflect = vNext;

                if (isColliding(pReflect, vReflect, pNext, vNext, pInter)) {
//                    ROS_INFO_STREAM("Second Collision!");
                }

                //! Update the state
                state.block<2, 1>(0, 0) = pNext;
                state.block<2, 1>(2, 0) = vNext;
            }
        }

    protected:
        bool isColliding(const Vector2 &p, const Vector2 &vel, Vector2 &pNext, Vector2 &vNext, Vector2 &pInter) {

            for (int i = 0; i < m_boundary.rows(); ++i) {
                Vector2 p1 = m_boundary.block<1, 2>(i, 0);
                Vector2 p2 = m_boundary.block<1, 2>(i, 2);

                Vector2 u = vel * m_dt;
                Vector2 v = p2 - p1;
                Vector2 w = p1 - p;

                T denominator = cross2D(v, u);
                if (abs(denominator) < 1e-6) {
                    // parallel
                    continue;
                } else {
                    T s = cross2D(v, w) / denominator;
                    T r = cross2D(u, w) / denominator;
                    if (s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4 && r <= 1 - 1e-4) {
                        Vector2 vt = vel.dot(v) / std::pow(v.norm(), 2) * v;
                        Vector2 vn = vel - vt;

                        // Velocity on next time step
                        vNext = m_mu * vt + (-m_e * vn);

                        // Position of intersection point
                        pInter = p + s * u;
                        // Position of next point
                        pNext = pInter + (1 - s) * vNext * m_dt;
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

        State m_Malletstate;

        Mallet(T dt) {
            m_dt = dt;
            t_prev = 0.;
            m_Malletstate.setZero();

            m_radiusPuck = 0.03165;
            m_radiusMallet = 0.04815;
            m_e = 0.6;
        }

        void setState(const tf::StampedTransform &tfMallet) {
            double deltaT = tfMallet.stamp_.toSec() - t_prev;

            m_Malletstate.dx() = (tfMallet.getOrigin().x() - m_Malletstate.x()) / deltaT;
            m_Malletstate.dy() = (tfMallet.getOrigin().y() - m_Malletstate.y()) / deltaT;
            m_Malletstate.dtheta() = (tf::getYaw(tfMallet.getRotation()) - m_Malletstate.theta()) / deltaT;

            m_Malletstate.x() = tfMallet.getOrigin().x();
            m_Malletstate.y() = tfMallet.getOrigin().y();
            m_Malletstate.theta() = tf::getYaw(tfMallet.getRotation());
            t_prev = tfMallet.stamp_.toSec();
        }

        void applyCollision(State &puckState) {
            Vector2 pPuck = puckState.block<2, 1>(0, 0);
            Vector2 vPuck = puckState.block<2, 1>(2, 0);
            Vector2 pPuckNext = pPuck + vPuck * m_dt;

            Vector2 pMallet = m_Malletstate.block<2, 1>(0, 0);
            Vector2 vMallet = m_Malletstate.block<2, 1>(2, 0);
            Vector2 pMalletNext = pMallet + vMallet * m_dt;

            if ((pPuck - pMallet).norm() > (m_radiusMallet + m_radiusPuck) &&
                (pMalletNext - pPuckNext).norm() < (m_radiusMallet + m_radiusPuck)) {

                Vector2 vRelative = vPuck - vMallet;
                Vector2 pRelative = pMallet - pPuck;
                // Angle between velocity and relative position
                double cosAlpha = vRelative.dot(pRelative) / (vRelative.norm() * pRelative.norm());

                double dist = pRelative.norm();
                double distMin = m_radiusPuck + m_radiusMallet;

                double s = 1.;
                if (vRelative.norm() > 1e-3) {
                    s = (dist * cosAlpha - sqrt(pow(dist * cosAlpha, 2) - dist * dist + distMin * distMin))/ (vRelative.norm() * m_dt);
                }

                Vector2 pPuckCollide = pPuck + s * vPuck * m_dt;
                Vector2 pMalletCollide = pMallet + s * vMallet * m_dt;

                Vector2 vNorm = pMalletCollide - pPuckCollide;
                Vector2 vPuckNormal = vPuck.dot(vNorm) / pow(vNorm.norm(), 2) * vNorm;
                Vector2 vMalletNorm = vMallet.dot(vNorm) / pow(vNorm.norm(), 2) * vNorm;

                Vector2 vPuckTangent = vPuck - vPuckNormal;
                Vector2 vMalletTangent = vMallet - vMalletNorm;

                Vector2 vPuckNextNorm = -m_e * vPuckNormal + (1 + m_e) * vMalletNorm;
                Vector2 vPuckNextTangent = 2. / 3. * vPuckTangent + 1. / 3. * vMalletTangent;

                //! Update the state
                puckState.block<2, 1>(0, 0) = pPuckCollide + (1 - s) * (vPuckNextNorm + vPuckNextTangent) * m_dt;
                puckState.block<2, 1>(2, 0) = vPuckNextNorm + vPuckNextTangent * m_dt;
            }
        }
    };

    class CollisionModel {
    public:

        AirHockeyTable m_table;

        Mallet m_mallet;

        CollisionModel(AirHockeyTable &table, Mallet &mallet) : m_table(table), m_mallet(mallet) {
        }

        void applyCollision(State &puckState) {
            m_table.applyCollision(puckState);
            m_mallet.applyCollision(puckState);
        }

    };

}

#endif //PUCK_TRACKER_COLLISIONMODEL_HPP
