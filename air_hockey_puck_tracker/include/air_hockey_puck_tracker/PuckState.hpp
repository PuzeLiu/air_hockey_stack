//
// Created by default on 13.01.22.
//
#ifndef SRC_PUCKSTATE_HPP
#define SRC_PUCKSTATE_HPP

#include <kalman/LinearizedSystemModel.hpp>

namespace air_hockey_baseline_agent {
    /**
     * @brief System control-input vector-type for the puck
     *
     * This is the system control-input of the puck, here
     * is the time step dt.
     *
     * @param T Numeric scalar type
     */
    class Control : public Kalman::Vector<double, 1> {
    public:
        KALMAN_VECTOR(Control, double, 1)

        //! Velocity
        static constexpr size_t DT = 0;

        inline double dt() const {
            return (*this)[DT];
        }

        inline double &dt() {
            return (*this)[DT];
        }
    };

    /**
     * @brief System state vector-type for the puck
     *
     * This is a system state for a very simple planar robot that
     * is characterized by its (x,y)-Position and (x,y)-Velocity.
     *
     * @param T Numeric scalar type
     */
    class PuckState : public Kalman::Vector<double, 6> {
    public:
        KALMAN_VECTOR(PuckState, double, 6)

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

        inline double x() const {
            return (*this)[X];
        }

        inline double y() const {
            return (*this)[Y];
        }

        inline double dx() const {
            return (*this)[DX];
        }

        inline double dy() const {
            return (*this)[DY];
        }

        inline double theta() const {
            return (*this)[THETA];
        }

        inline double dtheta() const {
            return (*this)[DTHETA];
        }

        inline double &x() {
            return (*this)[X];
        }

        inline double &y() {
            return (*this)[Y];
        }

        inline double &dx() {
            return (*this)[DX];
        }

        inline double &dy() {
            return (*this)[DY];
        }

        inline double &theta() {
            return (*this)[THETA];
        }

        inline double &dtheta() {
            return (*this)[DTHETA];
        }
    };

}

#endif //SRC_PUCKSTATE_HPP
