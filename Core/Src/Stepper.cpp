/*
 * Stepper.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: arusso
 */

#include "Stepper.h"


void PWM_set_high_duration(TIM_TypeDef *timx, int us) {
	int ccr = 16./25. * us;
	timx->CCR1 = ccr;
}

void PWM_set_freq(TIM_TypeDef *timx, int hz) {
	int arr = 16000000./(25*(float)hz);
	timx->ARR = arr;
}

Stepper::Stepper(TIM_HandleTypeDef tim_handle) {
	// TODO Auto-generated constructor stub
	this->tim_handle = tim_handle;
	PWM_set_high_duration(this->tim_handle.Instance, 10);

	this->current_dir = 0;
	this->current_freq = 0;
}

void Stepper::set_speed_step_freq(int hz) {
	if(hz < 15){// todo calculer freq min automatiquement
		HAL_TIM_PWM_Stop(&this->tim_handle, TIM_CHANNEL_1);
		return;
	}
	if(!this->current_freq) {
		// stepper stopped, start pwm
		HAL_TIM_PWM_Start(&this->tim_handle, TIM_CHANNEL_1);
	}
	PWM_set_freq(this->tim_handle.Instance, hz);
}

Stepper::~Stepper() {
	// TODO Auto-generated destructor stub
}

