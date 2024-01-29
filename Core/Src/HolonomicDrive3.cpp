/*
 * HolonomicDrive3.cpp
 *
 *  Created on: Jan 1, 2024
 *      Author: arusso
 */

#include "HolonomicDrive3.h"

#define PI 3.14159265359
#define SQRT_2_OVER_2 0.70710678118
#define SQRT_3_OVER_2 0.86602540378

#define MAX_AXEL_WHEEL 0.05 // a wheel can gain <value> rotation per update

Vel sub(Vel vel1, Vel vel2) {
	return {vel1.x - vel2.x, vel1.y - vel2.y, vel1.theta - vel2.theta};
}

void sub(float* arr1, float* arr2, float* ret) {
	for(int i=0; i<3; i++) {
		ret[i] = arr1[i] - arr2[i];
	}
}

void abs(float* arr, float* ret) {
	for(int i=0; i<3; i++) {
		if(arr[i]>=0) {
			ret[i] = arr[i];
		}
		else {
			ret[i] = -arr[i];
		}
	}
}

int get_index_max(float* arr) {
	if(arr[0] > arr[1] && arr[0] > arr[2]) {
		return 0;
	}
	else if(arr[1] > arr[0] && arr[1] > arr[2]) {
		return 1;
	}
	else {
		return 2;
	}
}

HolonomicDrive3::HolonomicDrive3(Stepper stepper0, Stepper stepper1, Stepper stepper2, float wheel_radius, float wheel_distance) {
	this->steppers[0] = stepper0;
	this->steppers[1] = stepper1;
	this->steppers[2] = stepper2;
	this->wheel_circumference = wheel_radius * 2.0 * PI;
	this->wheel_distance = wheel_distance;
	this->current_wheels_speeds_rps[0] = 0;
	this->current_wheels_speeds_rps[1] = 0;
	this->current_wheels_speeds_rps[2] = 0;

}

void HolonomicDrive3::set_cmd_vel(Vel cmd) {
	this->cmd_vel = cmd;
}

void HolonomicDrive3::compute_wheels_speeds(Vel cmd, float *ret_speeds_rps) {
    float wheel0_mps = 0.5 * this->cmd_vel.x - SQRT_3_OVER_2 * this->cmd_vel.y - this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    float wheel1_mps = 0.5 * this->cmd_vel.x + SQRT_3_OVER_2 * this->cmd_vel.y - this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    float wheel2_mps = - this->cmd_vel.x + this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    // wheel mps -> wheel rps
    ret_speeds_rps[0] = wheel0_mps / this->wheel_circumference;
    ret_speeds_rps[1] = wheel1_mps / this->wheel_circumference;
    ret_speeds_rps[2] = wheel2_mps / this->wheel_circumference;
}

void HolonomicDrive3::write_wheels_speeds(float *speeds_rps) {
	for(int i=0; i<3; i++) {
		this->steppers[i].set_speed_rps(speeds_rps[i]);
		this->current_wheels_speeds_rps[i] = speeds_rps[i];
	}
}

void HolonomicDrive3::spin_once_motors_control() {

	// compare current_vel and cmd_vel wheels speeds to check the required acceleration to transition directly from current to command
	float cmd_wheels_speeds_rps[3];
	this->compute_wheels_speeds(cmd_vel, cmd_wheels_speeds_rps);

	float cmd_sub_current_speeds[3];
	sub(cmd_wheels_speeds_rps, this->current_wheels_speeds_rps, cmd_sub_current_speeds);

	float speeds_diff[3];
	abs(cmd_sub_current_speeds, speeds_diff);
	if(speeds_diff[0] < MAX_AXEL_WHEEL && speeds_diff[1] < MAX_AXEL_WHEEL && speeds_diff[2] < MAX_AXEL_WHEEL) {
		// acceleration requested is ok, no need to accelerate gradually.
		this->write_wheels_speeds(cmd_wheels_speeds_rps);
	}
	else {
		// find max accel needed
		int i_max = get_index_max(speeds_diff);

		float speed_ratio = MAX_AXEL_WHEEL / speeds_diff[i_max]; // speed ratio of each original speed to add

		float new_speeds_cmds[3];
		for(int i=0; i<3; i++) {
			new_speeds_cmds[i] = current_wheels_speeds_rps[i] + speed_ratio * cmd_sub_current_speeds[i];;
		}

		// set speed
		this->write_wheels_speeds(new_speeds_cmds);

		// todo documenter les equations


	}

}


HolonomicDrive3::~HolonomicDrive3() {
	// TODO Auto-generated destructor stub
}

