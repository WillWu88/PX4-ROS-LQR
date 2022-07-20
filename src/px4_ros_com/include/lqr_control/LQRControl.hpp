/**
 * @file LQRControl.hpp
 *
 * LQR Controller class, providing lqr helper functions
 *
 * @author Will Wu	<willwu@wustl.edu>
 *
 * @copyright Copyright (c) System Theory Lab, 2022
 */

#pragma once

#include "params.hpp"

class LQRControl
{
public:
	LQRControl();
	LQRControl(Eigen::VectorXf state);
	LQRControl(int output_num, int stat_num, Eigen::VectorXf state);
	~LQRControl() = default;

	/**
	 * Normalize then reduce quaternion by eliminating the scalar
	 * @param quat_cord ref to the quaternion to change
	 * @return a 3x1 vector containing the i, j, k value
	 */
	Eigen::Vector3f reduceQuat(const Eigen::Quaternionf &quat_cord);

	/**
	 * Set new lqr gain matrix (3x6), assume externally calculated gain
	 * @param new_k new gain matrix
	 */
	void setLQRGain(const Eigen::MatrixXf &new_k);

	/**
	 * Restore unit quaternion based on q1, q2, q3 (assume that order)
	 * @param quat: vector that contains q1, q2 q3 (in that order))
	 */
	Eigen::Quaternionf restoreFullQuat(const Eigen::Vector3f &quat);

	int returnOutputWidth() {return _num_of_output;}
	int returnStateWidth() {return _num_of_states;}


	/**
	 * Update state variable, can be partial given the index
	 * @param new_val: new vector to override the old one
	 * @param start: starting index of subvector
	 * @param end: ending index of subvector
	 * @return integer that denotes the state of execution */
	int updateState(const Eigen::VectorXf &new_val, int start, int end);

	/**
	 * Update setpoint variable, can be partial given the index
	 * @param new_val: new vector to override the old one
	 * @param start: starting index of subvector
	 * @param end: ending index of subvector
	 * @return integer that denotes the state of execution */
	int updateSetpoint(const Eigen::VectorXf &new_val, int start, int end);

	int updateEQInput(float new_val, int index);
	int updateEQInput(const Eigen::VectorXf &new_val, int start, int end);

	/**
	 * Calculate error vector, notice quaternion way of calculating error
	 * @param setpoint vector */
	Eigen::VectorXf calculateError (const Eigen::VectorXf &setpoint);
	/**
	 * Run the controller once
	 *
	 * @param q_state current rotation from the estimator
	 * @param rate_state current rate from the estimator
	 * @param landed vehicle landing status
	 * @warning need to implement landing logic
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	Eigen::VectorXf update();

	Eigen::VectorXf returnState();

	Eigen::MatrixXf checkGain();



private:
	const int _num_of_output;
	const int _num_of_states;
	Eigen::MatrixXf _lqr_gain_matrix;
	Eigen::VectorXf _state;
	Eigen::VectorXf _setpoint;
	Eigen::VectorXf _eq_u;

};
