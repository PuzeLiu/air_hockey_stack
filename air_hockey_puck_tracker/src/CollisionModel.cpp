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

#include "air_hockey_puck_tracker/CollisionModel.hpp"

using namespace std;

namespace air_hockey_baseline_agent {

    static double cross2D(Vector2 v1, Vector2 v2) {
        return v1.x() * v2.y() - v1.y() * v2.x();
    }

    AirHockeyTable::AirHockeyTable(double length, double width, double goalWidth, double puckRadius, double e,
                                   double rimFric, double dt) {
        m_length = length;
        m_width = width;
        m_puckRadius = puckRadius;
        m_goalWidth = goalWidth;

        m_e = e;
        m_mu = rimFric;
        m_dt = dt;

        Vector2 ref, offsetP1, offsetP2, offsetP3, offsetP4;
        ref << length / 2, 0;
        offsetP1 << -(m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
        offsetP2 << -(m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
        offsetP3 << (m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
        offsetP4 << (m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
        offsetP1 += ref;
        offsetP2 += ref;
        offsetP3 += ref;
        offsetP4 += ref;

        m_boundary.row(0) << offsetP1.x(), offsetP1.y(), offsetP3.x(), offsetP3.y();
        m_boundary.row(1) << offsetP3.x(), offsetP3.y(), offsetP4.x(), offsetP4.y();
        m_boundary.row(2) << offsetP4.x(), offsetP4.y(), offsetP2.x(), offsetP2.y();
        m_boundary.row(3) << offsetP2.x(), offsetP2.y(), offsetP1.x(), offsetP1.y();
    }

    AirHockeyTable::~AirHockeyTable() {

    }

    bool AirHockeyTable::hasCollision(const PuckState &state, int &collisionForm) {
        Vector2 p, vel;
        p = state.block<2, 1>(0, 0);
        vel = state.block<2, 1>(2, 0);

        if (abs(p.y()) < m_goalWidth / 2 - m_puckRadius && p.x() < m_boundary(0, 0)) {
            return false;
        } else if (abs(p.y()) < m_goalWidth / 2 - m_puckRadius && p.x() > m_boundary(0, 2)) {
            return false;
        }

        Vector2 u = vel * m_dt;
        for (int i = 0; i < m_boundary.rows(); ++i) {
            Vector2 p1 = m_boundary.block<1, 2>(i, 0);
            Vector2 p2 = m_boundary.block<1, 2>(i, 2);

            Vector2 v = p2 - p1;
            Vector2 w = p1 - p;

            double denominator = cross2D(v, u);
            double s = cross2D(v, w) / denominator;
            double r = cross2D(u, w) / denominator;
            if (abs(denominator) < 1e-6) {
                return false;
            }
            if (cross2D(w, v) < 0
                || (s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4
                    && r <= 1 - 1e-4)) {
                collisionForm = i + 1;
                return true;
            }
        }
        return false;
    }

        bool AirHockeyTable::applyCollision(PuckState &state) {
            Vector2 p, vel;
            p = state.block<2, 1>(0, 0);
            vel = state.block<2, 1>(2, 0);
            double theta = state.theta();
            double dtheta = state.dtheta();

            if (abs(p.y()) < m_goalWidth / 2 - m_puckRadius && p.x() < m_boundary(0, 0)) {
                return false;
            } else if (abs(p.y()) < m_goalWidth / 2 - m_puckRadius && p.x() > m_boundary(0, 2)) {
                return false;
            }

            for (int i = 0; i < m_boundary.rows(); ++i) {
                Vector2 p1 = m_boundary.block<1, 2>(i, 0);
                Vector2 p2 = m_boundary.block<1, 2>(i, 2);

                Vector2 u = vel * m_dt;
                Vector2 v = p2 - p1;
                Vector2 w = p1 - p;

                Eigen::Rotation2D<double> rot(M_PI_2);
                Vector2 vecT = v / v.norm();
                Vector2 vecN = rot * vecT;

                double denominator = cross2D(v, u);
                if (abs(denominator) < 1e-6) {
                    // parallel
                    continue;
                } else {
                    double s = cross2D(v, w) / denominator;
                    double r = cross2D(u, w) / denominator;
                    if (cross2D(w, v) < 0
                        || (s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4
                            && r <= 1 - 1e-4)) {

                        double vtScalar = vel.dot(vecT);
                        double vnScalar = vel.dot(vecN);
                        double dthetaNext = 0.;
                        double vtNextScalar;
                        double vnNextScalar;

                        if (abs(vtScalar + m_puckRadius * dtheta) < 3 * m_mu * (1 + m_e) * abs(vnScalar)) {
                            // Velocity on next time step without sliding
                            vtNextScalar =
                                    (2. / 3.) * vtScalar - (m_puckRadius / 3) * dtheta;
                            vnNextScalar = -m_e * vnScalar;
                            if (dtheta != 0) {
                                dthetaNext = 1. / 3. * dtheta - 2. / (3. * m_puckRadius) * vtScalar;
                            }
                        } else {
                            // Velocity on next time step with sliding
                            double sliding = vtScalar + dtheta * m_puckRadius;
                            sliding = sliding / abs(sliding);
                            vtNextScalar = vtScalar + m_mu * sliding * (1 + m_e) * vnScalar;
                            vnNextScalar = -m_e * vnScalar;
                            if (dtheta != 0) {
                                dthetaNext = dtheta + (2. * m_mu * sliding * (1 + m_e) / m_puckRadius) * vnScalar;
                            }
                        }

                        Vector2 vNext = vnNextScalar * vecN + vtNextScalar * vecT;

                        // Position of intersection point
                        Vector2 pInter = p + s * u;
                        // Angular Position of intersection point
                        double thetaInter = theta + s * dtheta * m_dt;
                        // Position of next point
                        Vector2 pNext = pInter + (1 - s) * vNext * m_dt;
                        // Angular Position of next point
                        double thetaNext = thetaInter + (1 - s) * dthetaNext * m_dt;

                        state.block<2, 1>(0, 0) = pNext;
                        state.block<2, 1>(2, 0) = vNext;
                        state.theta() = thetaNext;
                        state.dtheta() = dthetaNext;
                        return true;
                    }
                }
            }
            return false;
        }

        bool AirHockeyTable::isOutsideBoundary(Measurement &measurement) {
            if (abs(measurement.y()) > m_width / 2 - m_puckRadius + 0.01 ||
                measurement.x() < -0.01 ||
                measurement.x() > m_length - m_puckRadius + 0.01) {
                return true;
            }
            return false;
        }

        void AirHockeyTable::setME(double mE) {
            m_e = mE;
        }

        void AirHockeyTable::setMu(double mu) {
            m_mu = mu;
        }

        Mallet::Mallet(double
        puckRadius, double
        malletRadius, double
        restitution, double
        dt) {
            m_dt = dt;
            t_prev = 0.;
            m_malletState.setZero();

            m_radiusPuck = puckRadius;
            m_radiusMallet = malletRadius;
            m_e = restitution;
        }

        void Mallet::setState(const geometry_msgs::TransformStamped &tfMallet) {
            double deltaT = tfMallet.header.stamp.toSec() - t_prev;
            if (deltaT > 0) {
                m_malletState.dx() = (tfMallet.transform.translation.x - m_malletState.x())
                                     / deltaT;
                m_malletState.dy() = (tfMallet.transform.translation.y - m_malletState.y())
                                     / deltaT;
                tf2::Quaternion quat;
                quat.setX(tfMallet.transform.rotation.x);
                quat.setY(tfMallet.transform.rotation.y);
                quat.setZ(tfMallet.transform.rotation.z);
                quat.setW(tfMallet.transform.rotation.w);
                tf2::Matrix3x3 rotMat(quat);
                double roll, pitch, yaw;
                rotMat.getEulerYPR(yaw, pitch, roll);

                m_malletState.dtheta() = (yaw - m_malletState.theta()) / deltaT;

                m_malletState.x() = tfMallet.transform.translation.x;
                m_malletState.y() = tfMallet.transform.translation.y;
                m_malletState.theta() = yaw;
                t_prev = tfMallet.header.stamp.toSec();
            }
        }

        bool Mallet::applyCollision(PuckState &puckState) {
            Vector2 pPuck = puckState.block<2, 1>(0, 0);
            Vector2 vPuck = puckState.block<2, 1>(2, 0);

            Vector2 pMallet = m_malletState.block<2, 1>(0, 0);
            Vector2 vMallet = m_malletState.block<2, 1>(2, 0);

            Vector2 vRelative = m_dt * vPuck;
            Vector2 pRelative = pMallet - pPuck;

            double distToGo = pRelative.norm() - m_radiusPuck - m_radiusMallet;
            double t = distToGo / (vRelative.dot(pRelative) / pRelative.norm());

            if (t > 0 && t < 1) {
                Vector2 pPuckCollide = pPuck + t * vRelative;

                Vector2 vecN = pPuckCollide - pMallet;

                Eigen::Rotation2D<double> rot(-M_PI_2);
                vecN.normalize();
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

        void Mallet::setME(double mE) {
            m_e = mE;
        }

        CollisionModel::CollisionModel(double
        tableLength, double
        tableWidth, double
        goalWidth, double
        puckRadius, double
        malletRadius,
                double & restitutionTable, double & restitutionMallet, double & rimFriction, double
        dt) :
        m_table(tableLength, tableWidth, goalWidth, puckRadius, restitutionTable, rimFriction, dt), m_mallet(puckRadius,
                                                                                                             malletRadius,
                                                                                                             restitutionMallet,
                                                                                                             dt)
        {

        }

        bool CollisionModel::applyCollision(PuckState &puckState, const bool &checkMallet) {
            bool hasCollision;
            if (checkMallet) {
                hasCollision = m_mallet.applyCollision(puckState);
            }
            hasCollision = m_table.applyCollision(puckState) || hasCollision;
            return hasCollision;
        }

        bool CollisionModel::hasCollision(const PuckState &puckState, int &collisionForm) {
            return m_table.hasCollision(puckState, collisionForm);
        }

        void CollisionModel::setTableRestitution(const double tableRes) {
            m_table.setME(tableRes);
        }

        void CollisionModel::setMalletRestitution(const double malletRes) {
            m_mallet.setME(malletRes);
        }

        void CollisionModel::setRimFriction(const double rimFric) {
            m_table.setMu(rimFric);
        }

    }
