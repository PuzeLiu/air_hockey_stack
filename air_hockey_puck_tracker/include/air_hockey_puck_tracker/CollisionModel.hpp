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

#include <boost/algorithm/clamp.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"

namespace AirHockey {

    static double cross2D(Vector2 v1, Vector2 v2);

    class AirHockeyTable {
    public:
        //! BoundaryType of 4 lines, each row is line between <x1, y1, x2, y2>
        typedef Kalman::Matrix<double, 4, 4> BoundaryType;
    protected:
        //! Table length: X-direction, width: Y-direction
        double m_length;
        double m_width;

        //! step size
        double m_dt;

        //! radius of the puck
        double m_puckRadius;
        //! restitution coefficient
        double m_e;

        BoundaryType m_boundary;

    public:
        AirHockeyTable(double length, double width, double puckRadius, double restitution, double dt = 1 / 120.);

        ~AirHockeyTable();

        inline BoundaryType getBoundary() { return m_boundary; }

        bool applyCollision(EKF_Wrapper::State &state);

        bool isOutsideBoundary(Measurement &measurement);

    };

    class Mallet {
    public:
        //! radius of the mallet
        double m_radiusMallet;

        //! radius of the puck
        double m_radiusPuck;

        //! time step
        double m_dt;
        //! time stamp of previous update
        double t_prev;
        //! restitution coefficient
        double m_e;

        State m_malletState;

        Mallet(double puckRadius, double malletRadius, double restitution, double dt);

        void setState(const geometry_msgs::TransformStamped &tfMallet);

        bool applyCollision(State &puckState);
    };

    class CollisionModel {
    public:

        AirHockeyTable m_table;

        Mallet m_mallet;

        CollisionModel(double tableLength, double tableWidth, double puckRadius, double malletRadius,
                       double &restitutionTable, double &restitutionMallet, double dt);

        bool applyCollision(State &puckState, const bool& checkMallet);

    };

}

#endif //PUCK_TRACKER_COLLISIONMODEL_HPP
