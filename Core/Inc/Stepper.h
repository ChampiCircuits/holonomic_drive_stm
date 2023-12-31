/*
 * Stepper.h
 *
 *  Created on: Dec 31, 2023
 *      Author: arusso
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include "stm32g4xx_hal.h"

class Stepper {
public:
	Stepper(TIM_HandleTypeDef tim_handle);
	void set_speed_step_freq(int hz);
	virtual ~Stepper();

private:
	TIM_HandleTypeDef tim_handle;
	int current_freq;
	int current_dir;
};

#endif /* SRC_STEPPER_H_ */