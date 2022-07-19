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
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/actuator_controls.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <eigen3/Eigen/Eigen>
#include <lqr_control/LQRControl.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardLQR: public rclcpp::Node {
public:
	OffboardLQR() : Node("offboard_lqr_controller") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		actuator_command_publisher_ =
			this->create_publisher<ActuatorControls>("fmu/actuator_controls_0/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		actuator_command_publisher_ =
			this->create_publisher<ActuatorControls0>("fmu/actuator_controls_0/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					times_tamp_.store(msg->timestamp);
				});

		// update vehicle state once an update becomes available
		vehicle_pos_vel_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"fmu/vehicle_local_position/out",
			10,
			[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
				Eigen::VectorXf new_state;
				new_state << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz;
				controller_.updateState(new_state,
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P_X),
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::V_Z));
			});
		vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"fmu/vehicle_attitude/out",
			10,
			[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
				Eigen::Quaternionf new_state(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
				controller_.updateState(controller_.reduceQuat(new_state),
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_1),
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::Q_3));
			});
		vehicle_angular_v_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
			"fmu/vehicle_angular_velocity/out",
			10,
			[this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
				Eigen::VectorXf new_state(msg->xyz.size());
				new_state << msg->xyz[0], msg->xyz[1], msg->xyz[2];
				controller_.updateState(new_state,
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P),
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::R));
			});

        // update vehicle position setpoint, from qgroundcontrol
        vehicle_trajectory_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
			"fmu/trajectory_setpoint/out",
			10,
			[this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg) {
				Eigen::VectorXf new_state(msg->position.size());
				new_state << msg->position[0], msg->position[1], msg->position[2];
				controller_.updateSetpoint(new_state,
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P_X),
									   static_cast<int>(LQR_PARAMS::STATE_VECTOR_QUAT::P_Z));
			});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_control_setpoint();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(5ms, timer_callback); //update control at 200Hz
	}

	void arm() const;
	void disarm() const;

private:
	LQRControl controller_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ActuatorControls>::SharedPtr actuator_command_publisher_;

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
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardLQR::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardLQR::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardLQR::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish Commanded Wrench Calculated by Controller
 */
void OffboardLQR::publish_control_setpoint() {
	Eigen::VectorXf control_val = controller_.update();
	ActuatorControls msg{}; //needs review
	msg.timestamp = timestamp_.load();
	msg.control[msg.INDEX_ROLL] = control_val(LQR_PARAMS::CONTROL_VECTOR::AILERON);
	msg.control[msg.INDEX_PITCH] = control_val(LQR_PARAMS::CONTROL_VECTOR::ELEVATOR);
	msg.control[msg.INDEX_YAW] = control_val(LQR_PARAMS::CONTROL_VECTOR::RUDDER);
	msg.control[msg.INDEX_THROTTLE] = control_val(LQR_PARAMS::CONTROL_VECTOR::THRUST);

	actuator_command_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardLQR::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
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

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardLQR>());

	rclcpp::shutdown();
	return 0;
}
