/*
 * HolonomicDrive3.h
 *
 *  Created on: Jan 1, 2024
 *      Author: arusso
 */

#ifndef HOLONOMICDRIVE3_H_
#define HOLONOMICDRIVE3_H_


#include "Stepper.h"

struct Vel {
	float x = 0;
	float y = 0;
	float theta = 0;
};

class HolonomicDrive3 {
public:
	HolonomicDrive3();
	HolonomicDrive3(Stepper stepper0, Stepper stepper1, Stepper stepper2, float wheel_radius, float wheel_distance);
	void set_cmd_vel(Vel cmd);
	void write_wheels_speeds(float* speeds_rps);
	void compute_wheels_speeds(Vel cmd, float* ret_speeds_rps);
	void spin_once_motors_control();
	virtual ~HolonomicDrive3();
private:
	Stepper steppers[3];
	float wheel_circumference;
	float wheel_distance;
	Vel cmd_vel;
	float current_wheels_speeds_rps[3];

};

#endif /* HOLONOMICDRIVE3_H_ */
