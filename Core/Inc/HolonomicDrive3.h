/*
 * HolonomicDrive3.h
 *
 *  Created on: Jan 1, 2024
 *      Author: arusso
 */

#ifndef HOLONOMICDRIVE3_H_
#define HOLONOMICDRIVE3_H_


#include "Stepper.h"

struct CmdVel {
	float x = 0;
	float y = 0;
	float theta = 0;
};

class HolonomicDrive3 {
public:
	HolonomicDrive3(Stepper stepper0, Stepper stepper1, Stepper stepper2, float wheel_radius, float wheel_distance);
	void set_cmd_vel(CmdVel cmd);
	void spin_once_motors_control();
	virtual ~HolonomicDrive3();
private:
	Stepper steppers[3];
	float wheel_circumference;
	float wheel_distance;
	CmdVel cmd_vel;
};

#endif /* HOLONOMICDRIVE3_H_ */
