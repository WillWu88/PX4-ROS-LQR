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
#include <lqr_control/LQRControl.hpp>

LQRControl::LQRControl() :
	_num_of_output(LQR_PARAMS::CONTROL_VECTOR::DIM),
	_num_of_states(static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::DIM))
{
	_lqr_gain_matrix = Eigen::MatrixXf::Zero(_num_of_output, _num_of_states);
	_state = Eigen::VectorXf::Zero(_num_of_states);
	_setpoint = Eigen::VectorXf::Zero(_num_of_states);
}

LQRControl::LQRControl(Eigen::VectorXf state):
	_num_of_output(static_cast<int>(LQR_PARAMS::CONTROL_VECTOR::DIM)),
	_num_of_states(static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::DIM)),
	_state(state)
{
	_lqr_gain_matrix = Eigen::MatrixXf::Zero(_num_of_output, _num_of_states);
	_setpoint = Eigen::VectorXf::Zero(_num_of_states);
}

LQRControl::LQRControl(int output_num, int stat_num, Eigen::VectorXf state) :
	_num_of_output(output_num),
	_num_of_states(stat_num),
	_state(state)
{
	_lqr_gain_matrix = Eigen::MatrixXf::Zero(_num_of_output, _num_of_states);
	_setpoint = Eigen::VectorXf::Zero(_num_of_states);
}

Eigen::Vector3f LQRControl::reduceQuat(const Eigen::Quaternionf &quat_cord)
{
	Eigen::Quaternionf new_vec(quat_cord); //check copy constructor
	new_vec.normalize();
	Eigen::Vector3f return_vec(new_vec.x(), new_vec.y(), new_vec.z());
	return return_vec;
}

void LQRControl::setLQRGain(const Eigen::MatrixXf &new_k)
{
	if (new_k.rows() == _num_of_output && new_k.cols() == _num_of_states) {
		_lqr_gain_matrix = new_k;
	} else {
		// set gain to 0 if new input doesn't meet the dimension requirement
		_lqr_gain_matrix = Eigen::MatrixXf::Zero(_num_of_output, _num_of_states);
	}
}

Eigen::Quaternionf LQRControl::restoreFullQuat(const Eigen::Vector3f &quat)
{
	float q_0 = sqrt(1.0f - quat.norm() * quat.norm());
	Eigen::Quaternionf full_quat = Eigen::Quaternionf(q_0, quat[0], quat[1], quat[2]);
	return full_quat.normalized();
}

int LQRControl::updateState(const Eigen::VectorXf &new_val, int start, int end)
{
	if (start > end){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (new_val.size() != end - start + 1){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (start < 0 || start >= _num_of_states){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (end < 0 || end >= _num_of_states){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else {
		auto index = Eigen::seq(start, end);
		_state(index) = new_val;
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::SUCCESS);
	}
}

int LQRControl::updateSetpoint(const Eigen::VectorXf &new_val, int start, int end)
{
	if (start > end){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (new_val.size() != end - start + 1){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (start < 0 || start >= _num_of_states){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else if (end < 0 || end >= _num_of_states){
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::INCORRECT_DIM);
	} else {
		auto index = Eigen::seq(start, end);
		_setpoint(index) = new_val;
		return static_cast<int>(LQR_PARAMS::EXIT_CODE::SUCCESS);
	}
}

Eigen::VectorXf LQRControl::calculateError(const Eigen::VectorXf &setpoint)
{
	auto index = Eigen::seq(static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_1),
							static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_3));
	Eigen::Quaternionf quat_state = restoreFullQuat(_state(index));
	Eigen::Quaternionf quat_setpoint = restoreFullQuat(setpoint(index));
	Eigen::Vector3f quat_error_reduced = reduceQuat(quat_state.inverse() * quat_setpoint);
	Eigen::VectorXf error_vec = setpoint - _state;
	error_vec(index) = quat_error_reduced;
	return error_vec;
}


Eigen::VectorXf LQRControl::update()
{
	Eigen::VectorXf error = calculateError(_setpoint);

	return _lqr_gain_matrix * error;
}

Eigen::VectorXf LQRControl::returnState()
{
	return _state;
}

Eigen::MatrixXf LQRControl::checkGain()
{
	return _lqr_gain_matrix;
}
