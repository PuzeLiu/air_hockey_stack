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

#ifndef PUCK_TRACKER_SYSTEMMODEL_H
#define PUCK_TRACKER_SYSTEMMODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace AirHockey{

    typedef double T;

    typedef Kalman::Vector<T, 2> Vector2;

/**
 * @brief System state vector-type for the puck
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and (x,y)-Velocity.
 *
 * @param T Numeric scalar type
 */

class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-position
    static constexpr size_t Y = 1;
    //! X-velocity
    static constexpr size_t DX = 2;
    //! Y-velocity
    static constexpr size_t DY = 3;
    //! Yaw
    static constexpr size_t THETA = 4;
    //! Yaw-angular-velocity
    static constexpr size_t DTHETA = 5;

    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T dx()      const { return (*this)[ DX ]; }
    T dy()      const { return (*this)[ DY ]; }
    T theta()      const { return (*this)[ THETA ]; }
    T dtheta()      const { return (*this)[ DTHETA ]; }

    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& dx()      { return (*this)[ DX ]; }
    T& dy()      { return (*this)[ DY ]; }
    T& theta()      { return (*this)[ THETA ]; }
    T& dtheta()      { return (*this)[ DTHETA ]; }
};

/**
 * @brief System control-input vector-type for the puck
 *
 * This is the system control-input of the puck, here
 * is the time step dt.
 *
 * @param T Numeric scalar type
 */
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)

    //! Velocity
    static constexpr size_t DT = 0;

    T dt()       const { return (*this)[ DT ]; }

    T& dt()      { return (*this)[ DT ]; }
};

/**
 * @brief System model for the puck movement
 *
 * This is the system model defining how puck moves from one time-step to the
 * next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
class SystemModel : public Kalman::LinearizedSystemModel<State, Control>
{
public:
    //! State type shortcut definition
    typedef AirHockey::State S;

    //! Control type shortcut definition
    typedef AirHockey::Control C;

    //! Friction coefficient for air drag
    T m_c;
    //! Friction coefficient for sliding movement
    T m_d;

    /**
     * Constructor of the dynamic model
     * @param c Friction coefficient for air
     * @param d Friction coefficient for lateral movement
     */
    SystemModel(double c, double d){
        m_c = c;
        m_d = d;
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
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        x_.x() = x.x() + u.dt() * x.dx();
        x_.y() = x.y() + u.dt() * x.dy();
        x_.theta() = x.theta() + u.dt() * x.dtheta();
        if (x.dx()!=0 && x.dy()!=0) {
            x_.dx() = x.dx() - u.dt() * m_c * x.dx() - u.dt() * m_d * x.dx() / abs(x.dx());
            x_.dy() = x.dy() - u.dt() * m_c * x.dy() - u.dt() * m_d * x.dy() / abs(x.dy());
        } else{
            x_.dx() = x.dx() - u.dt() * m_c * x.dx();
            x_.dy() = x.dy() - u.dt() * m_c * x.dy();
        }

        x_.theta() = x.theta() + u.dt() * x.dtheta();
        x_.dtheta() = x.dtheta();
        // Return transitioned state vector
        return x_;
    }

protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        this->F.setIdentity();
        this->F(S::X, S::DX) = u.dt();
        this->F(S::Y, S::DY) = u.dt();
        this->F(S::DX, S::DX) = 1. - u.dt() * m_c;
        this->F(S::DY, S::DY) = 1. - u.dt() * m_c;
        this->F(S::THETA, S::DTHETA) = u.dt();
    }
};

}
#endif //PUCK_TRACKER_SYSTEMMODEL_H
