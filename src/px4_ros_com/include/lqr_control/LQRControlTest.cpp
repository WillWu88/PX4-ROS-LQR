/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
* @file LQRControlTest.cpp
*
* LQR Control Unit test using GTest
*
* @author Will Wu <willwu@wustl.edu>
* @copyright Copyright (c) System Theory Lab, 2022
*
* */

#include <gtest/gtest.h>
#include <LQRControl.hpp>
#include <mathlib/math/Functions.hpp>

TEST(LQRControlTest, QuaternionReduction)
{
    LQRControl controller;
    Eigen::Quaternionf quat_coord_normalized(1, 0, 0, 0);
    Eigen::Quaternionf quat_coord(3, 0, 0, 0);
    Eigen::Vector3f quat_reduced(0, 0, 0);
    EXPECT_TRUE(quat_reduced.isApprox(controller.reduceQuat(quat_coord_normalized)));
    EXPECT_TRUE(quat_reduced.isApprox(controller.reduceQuat(quat_coord)));
}

TEST(LQRControlTest, QuaternionSubtraction)
{
    LQRControl controller;
    Eigen::Quaternionf a(0, 1, 0, 0);
    Eigen::Quaternionf b(3, 0, 0, 0);
    Eigen::Quaternionf result(-3, 1, 0, 0);
    EXPECT_TRUE(result.isApprox(controller.quatSubtraction(a, b)));
}
TEST(LQRControlTest, LibConversion)
{
    LQRControl controller;
    Eigen::Quaternionf quat_coord(3, 0, 0, 0);
    matrix::Quatf px4_quat(3, 0, 0, 0);
    Eigen::Vector3f test_coord(9, 0, 3);
    matrix::Vector3f test_coord_px4(9, 0, 3);

    for (int i = 0; i < test_coord.size(); i++){
        EXPECT_NEAR(test_coord(i), controller.convertPX4Vec(test_coord_px4)(i), 1e-3f);
        EXPECT_NEAR(test_coord_px4(i), controller.convertEigen(test_coord)(i), 1e-3f);
    }

    Eigen::Quaternionf converted = controller.convertQuatf(px4_quat);
    quat_coord.normalize();
    EXPECT_NEAR(quat_coord.w(), converted.w(), 1e-3f);
    EXPECT_NEAR(quat_coord.x(), converted.x(), 1e-3f);
    EXPECT_NEAR(quat_coord.y(), converted.y(), 1e-3f);
    EXPECT_NEAR(quat_coord.z(), converted.z(), 1e-3f);
}

TEST(LQRControlTest, GainGeneration)
{
    LQRControl controller;
    //set point construction
    matrix::Quatf attitude_setpoint(1, 0, 0, 0);
    float yawspeed_setpoint = 0.5f;
    matrix::Vector3f rate_setpoint(0, 0, 0);

    Eigen::Matrix<float, 3, 6> gain_matrix;
    gain_matrix << Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity();

    EXPECT_EQ(3, gain_matrix.rows());
    EXPECT_EQ(6, gain_matrix.cols());

    controller.setLQRGain(gain_matrix);
    controller.setAttitudeSetpoint(attitude_setpoint, yawspeed_setpoint);
    controller.setRateSetpoint(rate_setpoint);

    //state construction
    matrix::Quatf q_state(0.0f, 0.65f, 0.0f, -0.75f);
    matrix::Vector3f rate_state(1, 0, 0);
    bool landed = false;

    Eigen::Matrix<float, 6, 1> state = controller.constructState(rate_state, q_state);
    Eigen::Matrix<float, 6, 1> state_actual;
    state_actual << -0.65f, 0.0f, 0.75f, -1.0f, 0.0f, 0.0f;
    for (int i = 0; i < state_actual.size(); i++)
    {
        EXPECT_NEAR(state_actual(i), state(i), 1e-2f);
    }

    matrix::Vector3f torque = controller.update(q_state, rate_state, landed);

    matrix::Vector3f target_torque(-1.65f, 0.0f, 0.75f);

    for (int i = 0; i < 3; i++){
        EXPECT_NEAR(target_torque(i), torque(i), 1e-2f);
    }
}
