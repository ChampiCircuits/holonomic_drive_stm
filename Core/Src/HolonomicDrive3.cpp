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

HolonomicDrive3::HolonomicDrive3(Stepper stepper0, Stepper stepper1, Stepper stepper2, float wheel_radius, float wheel_distance) {
	this->steppers[0] = stepper0;
	this->steppers[1] = stepper1;
	this->steppers[2] = stepper2;
	this->wheel_circumference = wheel_radius * 2.0 * PI;
	this->wheel_distance = wheel_distance;
}

void HolonomicDrive3::set_cmd_vel(CmdVel cmd) {
	this->cmd_vel = cmd;
}

void HolonomicDrive3::spin_once_motors_control() {
    float wheel0_mps = 0.5 * this->cmd_vel.x - SQRT_3_OVER_2 * this->cmd_vel.y - this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    float wheel1_mps = 0.5 * this->cmd_vel.x + SQRT_3_OVER_2 * this->cmd_vel.y - this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    float wheel2_mps = - this->cmd_vel.x + this->wheel_distance * this->cmd_vel.theta * PI / 180.0;
    // wheel mps -> wheel rps
    float wheel0_rps = wheel0_mps / this->wheel_circumference;
    float wheel1_rps = wheel1_mps / this->wheel_circumference;
    float wheel2_rps = wheel2_mps / this->wheel_circumference;

    this->steppers[0].set_speed_rps(wheel0_rps);
    this->steppers[1].set_speed_rps(wheel1_rps);
    this->steppers[2].set_speed_rps(wheel2_rps);
}

HolonomicDrive3::~HolonomicDrive3() {
	// TODO Auto-generated destructor stub
}

