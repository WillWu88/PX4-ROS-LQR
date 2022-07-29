/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/actuator_controls0.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <eigen3/Eigen/Eigen>
#include <lqr_control/LQRControl.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardLQR: public rclcpp::Node {
public:
	explicit OffboardLQR() : Node("offboard_lqr_controller") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);
		thrust_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("fmu/vehicle_thrust_setpoint/in", 10);
		torque_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("fmu/vehicle_torque_setpoint/in", 10);
		actuator_controls_publisher_ =
			this->create_publisher<px4_msgs::msg::ActuatorControls0>("fmu/actuator_controls_0/in", 10);
		actuator_outputs_publisher_ =
			this->create_publisher<px4_msgs::msg::ActuatorOutputs>("fmu/actuator_outputs/in", 10);
		// update vehicle state once an update becomes available
		vehicle_pos_vel_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"fmu/vehicle_local_position/out",
			10,
			[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
				if (msg == nullptr) {
					RCLCPP_INFO(this->get_logger(), "Empty Messsage Received");
				} else {
					_position_velocity[0] = msg->x;
					_position_velocity[1] = msg->y;
					_position_velocity[2] = msg->z;
					_position_velocity[3] = msg->vx;
					_position_velocity[4] = msg->vy;
					_position_velocity[5] = msg->vz;
					update_state();
				}
			});
		vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"fmu/vehicle_attitude/out",
			10,
			[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
				if (msg == nullptr){
					RCLCPP_INFO(this->get_logger(), "Empty Messsage Received");
				} else {
					_attitude.w() = msg->q[0];
					_attitude.x() = msg->q[1];
					_attitude.y() = msg->q[2];
					_attitude.z() = msg->q[3];
					update_state();
				}
			});
		vehicle_angular_v_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
			"fmu/vehicle_angular_velocity/out",
			10,
			[this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
				if (msg == nullptr){
					RCLCPP_INFO(this->get_logger(), "Empty Messsage Received");
				} else {
					_rate[0] = msg->xyz[0];
					_rate[1] = msg->xyz[1];
					_rate[2] = msg->xyz[2];
					update_state();
				}
			});
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in");
		vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in");
		thrust_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("fmu/vehicle_thrust_setpoint/in");
		torque_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("fmu/vehicle_torque_setpoint/in");
		actuator_controls_publisher_ =
			this->create_publisher<px4_msgs::msg::ActuatorControls0>("fmu/actuator_controls_0/in");
		actuator_outputs_publisher_ =
			this->create_publisher<px4_msgs::msg::ActuatorOutputs>("fmu/actuator_outputs/in");
#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 500) {
				// Change to Offboard mode after 1000 ms
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            // offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_outputs_test();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 501) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(2ms, timer_callback); //update control at 500Hz
	}

	void arm() const;
	void disarm() const;

private:
	LQRControl controller_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorControls0>::SharedPtr actuator_controls_publisher_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_publisher_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_pos_vel_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr vehicle_angular_v_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vehicle_trajectory_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void publish_control_setpoint();
	void publish_control_setpoint_test();
	void publish_outputs_test();
	// void message_received();
	Eigen::Vector<float, 6> _position_velocity;
	Eigen::Quaternionf _attitude;
	Eigen::Vector3f _rate;

	int update_state();

};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardLQR::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send, sending control signals");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardLQR::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardLQR::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = true;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish Commanded Wrench Calculated by Controller
 */
void OffboardLQR::publish_control_setpoint() {
	Eigen::VectorXf control_val = controller_.update();
	px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
	px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};

	thrust_msg.timestamp = timestamp_.load();
	torque_msg.timestamp = timestamp_.load();

	thrust_msg.xyz[0] = 0;
	thrust_msg.xyz[1] = 0;
	thrust_msg.xyz[2] = control_val[LQR_PARAMS::CONTROL_VECTOR::THRUST];

	torque_msg.xyz[0] = control_val[LQR_PARAMS::CONTROL_VECTOR::AILERON];
	torque_msg.xyz[1] = control_val[LQR_PARAMS::CONTROL_VECTOR::ELEVATOR];
	torque_msg.xyz[2] = control_val[LQR_PARAMS::CONTROL_VECTOR::RUDDER];

	thrust_command_publisher_->publish(thrust_msg);
	torque_command_publisher_->publish(torque_msg);

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardLQR::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardLQR::publish_control_setpoint_test()
{
	//float norm_scale = 1;
	px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
	px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
	px4_msgs::msg::ActuatorControls0 msg{};

	thrust_msg.timestamp = timestamp_.load();
	torque_msg.timestamp = timestamp_.load();

	msg.timestamp = timestamp_.load();
	msg.control[msg.INDEX_ROLL] = 0;
	msg.control[msg.INDEX_PITCH] = 0;
	msg.control[msg.INDEX_YAW] = 0;
	msg.control[msg.INDEX_THROTTLE] = 0.8;
	thrust_msg.xyz[0] = 0;
	thrust_msg.xyz[1] = 0;
	thrust_msg.xyz[2] = -0.8;

	torque_msg.xyz[0] = 0;
	torque_msg.xyz[1] = 0;
	torque_msg.xyz[2] = 0;

	//thrust_command_publisher_->publish(thrust_msg);
	//torque_command_publisher_->publish(torque_msg);
	actuator_controls_publisher_->publish(msg);
}

void OffboardLQR::publish_outputs_test()
{
	px4_msgs::msg::ActuatorOutputs pwm_message{};

	pwm_message.timestamp = timestamp_.load();

	pwm_message.noutputs = 6;
	pwm_message.output[0] = 1750;
	pwm_message.output[1] = 1750;
	pwm_message.output[2] = 1750;
	pwm_message.output[3] = 1750;
	pwm_message.output[4] = 1500;
	pwm_message.output[5] = 1500;
	pwm_message.output[8] = 8;

	actuator_outputs_publisher_->publish(pwm_message);
}

int OffboardLQR::update_state()
{
	int status = controller_.updateState(_rate,
							static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P),
							static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::R));
	if (status != LQR_PARAMS::EXIT_CODE::SUCCESS)
	{
		RCLCPP_INFO(this->get_logger(), "State update error");
		return status;
	}
	status = controller_.updateState(_position_velocity,
										 static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P_X),
										 static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::V_Z));
	if (status != LQR_PARAMS::EXIT_CODE::SUCCESS)
	{
		RCLCPP_INFO(this->get_logger(), "State update error");
		return status;
	}
	controller_.updateState(controller_.reduceQuat(_attitude),
							static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_1),
							static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_3));
	if (status != LQR_PARAMS::EXIT_CODE::SUCCESS)
	{
		RCLCPP_INFO(this->get_logger(), "State update error");
		return status;
	}
	return LQR_PARAMS::EXIT_CODE::SUCCESS;
}
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::cout << "publishing test controls" << std::endl;
	rclcpp::spin(std::make_shared<OffboardLQR>());

	rclcpp::shutdown();
	return 0;
}
