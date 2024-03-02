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
	HolonomicDrive3(const Stepper& stepper0, const Stepper& stepper1, const Stepper& stepper2, double wheel_radius, float wheel_distance);
	void set_cmd_vel(Vel cmd);
	void write_wheels_speeds(float* speeds_rps);
	void compute_wheels_speeds(Vel cmd, float* ret_speeds_rps);
	void spin_once_motors_control();
	Vel get_current_vel();
	void update_current_vel(const float* speeds_rps);
	virtual ~HolonomicDrive3();
private:
	Stepper steppers[3];
	float wheel_circumference;
	float wheel_distance;
	Vel cmd_vel;
	float current_wheels_speeds_rps[3];
	Vel current_vel;

};

#endif /* HOLONOMICDRIVE3_H_ */
