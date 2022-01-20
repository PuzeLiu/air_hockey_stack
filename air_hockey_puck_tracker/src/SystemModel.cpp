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

#include <rosconsole/macros_generated.h>
#include "air_hockey_puck_tracker/SystemModel.hpp"

namespace air_hockey_baseline_agent {

    SystemModel::SystemModel(double damping_, double mu_, double tableLength_, double tableWidth_, double goalWidth_, double puckRadius, double malletRadius,
                             double restitutionTable, double restitutionMallet, double rimFriction, double dt) {

        collisionModel = new CollisionModel(tableLength_, tableWidth_, goalWidth_, puckRadius, malletRadius,
                                             restitutionTable, restitutionMallet, rimFriction, dt);
	    damping = damping_;
	    mu = mu_;
	    rimFric = rimFriction;
	    resTable = restitutionTable;
	    pRadius = puckRadius;
    }

    SystemModel::~SystemModel(){
        delete collisionModel;
    }

/**
 * @brief Definition of (non-linear) state transition function
 *
 * This function defines how the system state is propagated through time,
 * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
 * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
 * the system control input \f$u\f$.
 *
 * @param [in] x The system state in current time-step
 * @param [in] u The control vector input
 * @returns The (predicted) system state in the next time-step
 */
    SystemModel::S SystemModel::f(const S &x, const C &u) const {
        //! Predicted state vector after transition
        S x_;

        x_.block<2, 1>(0, 0) = x.block<2, 1>(0, 0)
                               + u.dt() * x.block<2, 1>(2, 0);

        if (x.block<2, 1>(2, 0).norm() > 1e-6) {
            x_.block<2, 1>(2, 0) = x.block<2, 1>(2, 0)
                                   - u.dt() * (damping * x.block<2, 1>(2, 0) +
                                               mu * x.block<2, 1>(2, 0).cwiseSign());
        } else {
            x_.block<2, 1>(2, 0) = x.block<2, 1>(2, 0) - u.dt() * damping * x.block<2, 1>(2, 0);
        }
        double angle = fmod(x.theta() + u.dt() * x.dtheta(), M_PI * 2);
        if (angle > M_PI) angle -= M_PI * 2;
        else if (angle < -M_PI) angle += M_PI * 2;

        x_.theta() = angle;
        x_.dtheta() = x.dtheta();

        //apply collision if there is one
        collisionModel->applyCollision(x_,  true);

        // Return transitioned state vector
        return x_;
    }

    void SystemModel::updateJacobians(const S &x, const C &u) {
        int collisionForm;
        if(collisionModel->hasCollision(x, collisionForm)){
            printf("Collision detected boundary: %d", collisionForm);
            Eigen::Matrix<double,6, 6> A;
            Eigen::Matrix<double,6, 6> J;

            // with sliding
            /*J.setIdentity();
            J(0,2) = u.dt();
            J(1,3) = u.dt();
            J(2,3) = rimFric * s + (1 + resTable);
            J(3,3) = -resTable;
            J(4,5) = u.dt();
            J(5,3) = (2 * rimFric * s *(1 + resTable))/pRadius;*/

            //without sliding
            J.setIdentity();
            J(0,2) = u.dt();
            J(1,3) = u.dt();
            J(2,2) = 2/3;
            J(2, 5) = -pRadius / 3;
            J(3,3) = -resTable;
            J(4,5) = u.dt();
            J(5,2) = -2/3 * pRadius;

            if(collisionForm==1){
                //boundary 0
                A.setIdentity();
            }else if(collisionForm ==2){
                //boundary 1
                A.setIdentity();
                A(0,0) = 0.;
                A(1,1) = 0.;
                A(2,2) = 0.;
                A(3,3) = 0.;
                A(0,1) = 1.;
                A(1, 0) = -1.;
                A(2,3) = 1.;
                A(3,2) = -1.;
            }else if(collisionForm ==3){
                //boundary 2
                A.setIdentity();
                A(0,0) = -1.;
                A(1,1) = -1.;
                A(2,2) = -1.;
                A(3,3) = -1.;
            }else if(collisionForm == 4){
                // boundary 3
                A.setIdentity();
                A(0,0) = 0.;
                A(1,1) = 0.;
                A(2,2) = 0.;
                A(3,3) = 0.;
                A(0,1) = -1.;
                A(1, 0) = 1.;
                A(2,3) = -1.;
                A(3,2) = 1.;
            }
            this->F = A * J * A.inverse();
        }else {
            this->F.setIdentity();
            this->F(S::X, S::DX) = u.dt();
            this->F(S::Y, S::DY) = u.dt();
            this->F(S::DX, S::DX) = 1. - u.dt() * damping;
            this->F(S::DY, S::DY) = 1. - u.dt() * damping;
            this->F(S::THETA, S::DTHETA) = u.dt();
            this->F(S::DTHETA, S::DTHETA) = 1.;
            //ROS_INFO_STREAM("Update of system Jacobians" << F);
        }
    }

    void SystemModel::updateJacobiansWithCollision(const Eigen::Matrix<double,6, 6>& J, const Eigen::Matrix<double,6, 6> &A) {
        this->F = A * J * A.transpose();
    }

    void SystemModel::setDamping(double damping_) {
        SystemModel::damping = damping_;
    }

    void SystemModel::setMu(double mu_) {
        SystemModel::mu = mu_;
    }

    void SystemModel::setTableRestitution(const double tableRes) {
        collisionModel->setTableRestitution(tableRes);
    }

    void SystemModel::setMalletRestitution(const double malletRes) {
        collisionModel->setMalletRestitution(malletRes);
    }

    void SystemModel::setRimFriction(const double rimFric) {
        collisionModel->setRimFriction(rimFric);
    }

    bool SystemModel::isOutsideBoundary(Measurement &measurement) {
        return collisionModel->m_table.isOutsideBoundary(measurement);
    }


}
