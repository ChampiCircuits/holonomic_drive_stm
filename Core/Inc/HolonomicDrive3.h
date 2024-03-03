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
	double x = 0;
    double y = 0;
    double theta = 0;
};

class HolonomicDrive3 {
public:
	HolonomicDrive3();
	HolonomicDrive3(const Stepper& stepper0, const Stepper& stepper1, const Stepper& stepper2);
    void set_config(double max_accel, double wheel_radius, double base_radius);
    bool is_configured();
	void set_cmd_vel(Vel cmd);
	void write_wheels_speeds(double *speeds_rps);
	void compute_wheels_speeds(Vel cmd, double *ret_speeds_rps);
	void spin_once_motors_control();
	Vel get_current_vel();
	void update_current_vel(const double *speeds_rps);
	virtual ~HolonomicDrive3();
private:
	Stepper steppers[3];
	double wheel_circumference{};
    double wheel_distance{};
	Vel cmd_vel;
    double current_wheels_speeds_rps[3]{};
	Vel current_vel;
    /*
     * max_accel_per_cycle, en rotation par seconde par cycle, est la vitesse maximale autorisée
     * ajoutable à la vitesse actuelle d'une roue à chaque cycle. Soit : Combien peut-on ajouter de vitesse
     * à une roue à chaque cycle de contrôle ?
     */
    double max_accel_per_cycle{};
    bool has_config{};


};

#endif /* HOLONOMICDRIVE3_H_ */
