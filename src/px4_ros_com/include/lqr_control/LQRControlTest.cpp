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
#include "LQRControl.hpp"

class LQRControlTest : public ::testing::Test {
    protected:
        void SetUp() override{
            controller.setLQRGain(Eigen::MatrixXf::Random(4,12));
            controller.updateState(Eigen::VectorXf::Random(12,1), 0, 11);
        }
        LQRControl controller;
};

TEST_F(LQRControlTest, QuaternionManip)
{
    Eigen::Quaternionf quat_coord_normalized(1, 0, 0, 0);
    Eigen::Quaternionf quat_coord(3, 0, 0, 0);
    Eigen::Vector3f quat_reduced(0, 0, 0);
    Eigen::Quaternionf quat_a(1.5, 3, 4, 6);
    EXPECT_TRUE(quat_reduced.isApprox(controller.reduceQuat(quat_coord_normalized)));
    EXPECT_TRUE(quat_reduced.isApprox(controller.reduceQuat(quat_coord)));
    EXPECT_EQ(quat_coord_normalized, controller.restoreFullQuat(quat_reduced));
    EXPECT_TRUE(quat_a.isApprox(controller.restoreFullQuat(controller.reduceQuat(quat_a))));
}
