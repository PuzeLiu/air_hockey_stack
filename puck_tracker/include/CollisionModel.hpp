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
    class AirHockeyTable {
    public:
        typedef Kalman::Vector<T, 2> Vector2;
        //! BoundaryType of 4 lines, each row is line between <x1, y1, x2, y2>
        typedef Kalman::Matrix<T, 4, 4> BoundaryType;
    protected:
        Vector2 m_center;
        T m_z;
        T m_yaw;

        //! Table length: X-direction, width: Y-direction
        T m_length;
        T m_width;

        //! radius of the puck
        T m_puckRadius;

        BoundaryType m_boundary;

    public:
        AirHockeyTable() {
            m_center.setZero();
            m_yaw = 0.;
            m_length = 1.96;
            m_width = 1.04;
            m_puckRadius = 0.03165;
        }

        AirHockeyTable(tf::StampedTransform transform) : AirHockeyTable() {

            setTransform(transform);
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


    };

    class CollisionModel {
    public:
        using Vector2 = AirHockeyTable::Vector2;

        AirHockeyTable m_table;

        T m_dt;

        CollisionModel(AirHockeyTable &table) {
            m_table = table;
            m_dt = 0.01;
        }

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
                    ROS_INFO_STREAM("Second Collision!");
                }

                //! Update the state
                state.block<2, 1>(0, 0) = pNext;
                state.block<2, 1>(2, 0) = vNext;
            }
        }

    protected:
        bool isColliding(const Vector2 &p, const Vector2 &vel, Vector2 &pNext, Vector2 &vNext, Vector2 &pInter) {
            AirHockeyTable::BoundaryType tableBoundary = m_table.getBoundary();

            for (int i = 0; i < tableBoundary.rows(); ++i) {
                Vector2 p1 = tableBoundary.block<1, 2>(i, 0);
                Vector2 p2 = tableBoundary.block<1, 2>(i, 2);

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
                        Vector2 p12 = p2 - p1;
                        Vector2 vt = vel.dot(p12) / std::pow(p12.norm(), 2) * p12;
                        Vector2 vn = vel - vt;

                        // Velocity on next time step
                        vNext = vt + (-vn);

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

    private:
        static T cross2D(Vector2 v1, Vector2 v2) {
            return v1.x() * v2.y() - v1.y() * v2.x();
        }
    };
}

#endif //PUCK_TRACKER_COLLISIONMODEL_HPP
