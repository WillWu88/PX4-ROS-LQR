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
 * @file LQRControl.hpp
 *
 * LQR Attitude and Rate Controller
 *
 * @author Will Wu	<willwu@wustl.edu>
 *
 * @ref Matthias Grob <AttitudeControl.hpp> <AttitudeControl.cpp>
 * @ref <RateControl.cpp> <RateControl.hpp>
 * @copyright Copyright (c) System Theory Lab, 2022
 */

#pragma once

#include <Eigen/Eigen>

class LQRControl
{
public:
	LQRControl() = default;
	~LQRControl() = default;

	/**
	 * convert eigen lib vectors in px4 vectors
	 * @param eigen_vector vector to convert
	 * @return px4 3d vector
	 * */
	matrix::Vector3f convertEigen(const Eigen::Vector3f &eigen_vector);

	/**
	 * convert px4 vectors into eigen vectors
	 * @param eigen_vector vector to convert
	 * @return px4 3d vector
	 * */
	Eigen::Vector3f convertPX4Vec(const matrix::Vector3f &px4_vector);

	/**
	 * convert px4 quaternion into eigen quaternions
	 * @param px4_quat
	 * @return eigen quaternion
	 * */
	Eigen::Quaternionf convertQuatf(const matrix::Quatf &px4_quat);

	/**
	 * Normalize then reduce quaternion by eliminating the scalar
	 * @param quat_cord ref to the quaternion to change
	 * @return a 3x1 vector containing the i, j, k value
	 */
	Eigen::Vector3f reduceQuat(const Eigen::Quaternionf &quat_cord);

	/**
	 * Quaternion subtraction for Eigen
	 * @param quaternion a
	 * @param quaternion b
	 */
	Eigen::Quaternionf quatSubtraction(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b);

	/**
	 * Set new lqr gain matrix (3x6)
	 * @param new_k new gain matrix
	 */
	void setLQRGain(const Eigen::Matrix<float, 3 , 6> &new_k);

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint);

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta);

 	/**
  	* Update Rate Setpoint
  	* @param new_rate_setpoint
  	*/
	void setRateSetpoint(const matrix::Vector3f &new_rate_setpoint);

	/**
	 * Construct error state for gain multiplication
	 *
	 * @param rate_state current angular rate from estimator
	 * @param q_state current rotation from estimator
	 * @return error vector for gain multiplication
	 */
	Eigen::Matrix<float, 6, 1> constructState(const matrix::Vector3f &rate_state,
												   const matrix::Quatf &q_state);

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	int returnOutputWidth() {return _num_of_output;}

	int returnStateWidth() {return _num_of_states;}

	/**
	 * Run the controller once
	 *
	 * @param q_state current rotation from the estimator
	 * @param rate_state current rate from the estimator
	 * @param landed vehicle landing status
	 * @warning need to implement landing logic
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Quatf &q_state, const matrix::Vector3f &rate_state,
				const bool landed);

private:
	static constexpr const int _num_of_output = 3;
	static constexpr const int _num_of_states = 6;
	Eigen::Quaternionf _attitude_setpoint; //storing in the order of x, y, z, w
	Eigen::Matrix<float, _num_of_output, _num_of_states> _lqr_gain_matrix;
	Eigen::Vector3f _rate_setpoint;
	float _yawspeed_setpoint{0.f};


	//Optional Params
	matrix::Vector3f _rate_limit;

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;

};
