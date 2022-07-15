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
 * @file LQRControl.cpp
 * LQR Attitude and Rate Controller
 *
 *
 * @author Will Wu(you@domain.com)
 * @date 2022-06-07
 *
 * @copyright Copyright (c) System Theory Lab, 2022
 *
 */
#include <LQRControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>


matrix::Vector3f LQRControl::convertEigen(const Eigen::Vector3f &eigen_vector)
{
	matrix::Vector3f new_vec;
	for (int i = 0; i < eigen_vector.size(); i++){
		new_vec(i) = eigen_vector(i);
	}
	return new_vec;
}

Eigen::Vector3f LQRControl::convertPX4Vec(const matrix::Vector3f &px4_vector)
{
	Eigen::Vector3f new_vec;
	for (int i = 0; i < 3; i++){
		new_vec(i) = px4_vector(i);
	}
	return new_vec;
}

Eigen::Quaternionf LQRControl::convertQuatf(const matrix::Quatf &px4_quat)
{
	Eigen::Quaternionf new_vec(px4_quat(0), px4_quat(1), px4_quat(2), px4_quat(3));
	new_vec.normalize();
	return new_vec;
}

Eigen::Vector3f LQRControl::reduceQuat(const Eigen::Quaternionf &quat_cord)
{
	Eigen::Quaternionf new_vec(quat_cord); //check copy constructor
	Eigen::Vector3f return_vec(new_vec.x(), new_vec.y(), new_vec.z());
	return return_vec;
}

Eigen::Quaternionf LQRControl::quatSubtraction(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b)
{
	Eigen::Quaternionf new_vec(a.w()-b.w(), a.x()-b.x(), a.y()-b.y(), a.z()-b.z());
	return new_vec;
}

void LQRControl::setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
{

	_attitude_setpoint = convertQuatf(qd);
	_attitude_setpoint.normalize();
	_yawspeed_setpoint = yawspeed_setpoint;
}

void LQRControl::adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
{
	Eigen::Quaternionf quat_shift = convertQuatf(q_delta);
	_attitude_setpoint *= quat_shift;
	_attitude_setpoint.normalize();
}

void LQRControl::setRateSetpoint(const matrix::Vector3f &new_rate_setpoint)
{
	_rate_setpoint = convertPX4Vec(new_rate_setpoint);
}

void LQRControl::setLQRGain(const Eigen::Matrix<float, 3, 6> &new_k)
{
	if (new_k.rows() == _num_of_output && _num_of_states == 6) {
		_lqr_gain_matrix = new_k;
	} else {
		// set gain to 0 if new input doesn't meet the dimension requirement
		_lqr_gain_matrix = Eigen::MatrixXf::Zero(_num_of_output, _num_of_states);
	}
}
Eigen::Matrix<float, 6, 1> LQRControl::constructState(const matrix::Vector3f &rate_state, const matrix::Quatf &q_state)
{
	Eigen::Matrix<float, 6, 1> state_vector;
	state_vector << reduceQuat(quatSubtraction(_attitude_setpoint, convertQuatf(q_state))), _rate_setpoint - convertPX4Vec(rate_state);
	return state_vector;
}


void LQRControl::setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				      const matrix::Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

matrix::Vector3f LQRControl::update(const matrix::Quatf &q_state, const matrix::Vector3f &rate_state, const bool landed)
{
	Eigen::Vector3f torque = _lqr_gain_matrix * constructState(rate_state, q_state);
	return convertEigen(torque);
}
