/**
 * @file params.hpp
 * Providing common parameters for PX4 lqr offboard controller
 *
 * @author Will Wu	<willwu@wustl.edu>
 *
 * @copyright Copyright (c) System Theory Lab, 2022
 */

#pragma once

namespace LQR_PARAMS
{
    enum struct STATE_VECTOR_QUAT : int {P_X, P_Y, P_Z, V_X, V_Y, V_Z, Q_1, Q_2, Q_3, P, Q, R, DIM};
    enum struct STATE_VECTOR_QUAT_FULL {P_X, P_Y, P_Z, V_X, V_Y, V_Z, Q_0, Q_1, Q_2, Q_3, P, Q, R, DIM};
    enum struct EXIT_CODE{SUCCESS, INCORRECT_DIM = -1};

    //no need for scoped cuz format remains the same
    enum CONTROL_VECTOR{THRUST, AILERON, ELEVATOR, RUDDER, DIM};
}
