/*
 * MIT License
 * Copyright (c) 2020-2021 Puze Liu, Davide Tateo
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

#ifndef SRC_COMBINATORIAL_HIT_NEW_H
#define SRC_COMBINATORIAL_HIT_NEW_H

#include <Eigen/Dense>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

namespace air_hockey_baseline_agent {
	class CombinatorialHitNew {
	public:
		CombinatorialHitNew(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double rate, double height);

		~CombinatorialHitNew();

		bool plan(const Eigen::Vector2d &xStart, const Eigen::Vector2d &vStart,
		          const Eigen::Vector2d &xEnd, const Eigen::Vector2d &vEnd,
		          trajectory_msgs::MultiDOFJointTrajectory &cartTraj);

	private:
		bool getMiddlePoint();

		bool getArcCenter();

		void fitPhase();

		void getPoint(const double t);

	private:
		Eigen::Vector2d boundLower, boundUpper;

		Eigen::Vector2d xStart, xMiddle, xEnd;      //! Start point, middle point, final point, final velocity
		Eigen::Vector2d vecDir1, vecDir2, vDir;     //! Unit vector of linear 1, 2 and velocity direction
		Eigen::Vector2d xArcCenter, xVia1, xVia2;   //! Via point of arc
		Eigen::Vector2d aAngularAcc;                   //! angular acceleration
		double stepSize;
		double height;
		double vMag;                  				//! Magnitude of hitting velocity
		double tEnd;
		double arcAngle, arcRadius, arcLength;
		double counterClockWise;         			//! Indicator of arc direction 1: clockwise, -1: counterclockwise
		double l1, l2, lHit;                        //! Length of different segments
		double tDecelerateAngular;                  //! time for smooth angular acceleration
		Eigen::Matrix<double, 5, 1> phaseCoeff;

		double z, dz_dt, dz_ddt;                    //! Phase variable (arc length, velocity, acceleration)
		Eigen::Vector2d x, dx_dt, dx_ddt;           //! 2d position variable (arc length, velocity, acceleration)
		trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint;
		bool isStartPart;
	};
}


#endif //SRC_COMBINATORIAL_HIT_NEW_H
