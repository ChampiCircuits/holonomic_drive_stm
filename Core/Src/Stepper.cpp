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
	timx->CNT = 0;
	timx->ARR = arr;
}

Stepper::Stepper() {}

Stepper::Stepper(TIM_HandleTypeDef tim_handle_step, GPIO_TypeDef *GPIOx_dir, uint16_t GPIO_Pin_dir) {
	// TODO Auto-generated constructor stub
	this->tim_handle = tim_handle_step;
	this->GPIOx_dir = GPIOx_dir;
	this->GPIO_Pin_dir = GPIO_Pin_dir;
	PWM_set_high_duration(this->tim_handle.Instance, 10);
	HAL_TIM_PWM_Stop(&this->tim_handle, TIM_CHANNEL_1);

	this->current_dir = 0;
	this->current_freq = 0;
}

void Stepper::set_speed_step_freq(int hz, int dir) {
	if(hz < 15){// todo calculer freq min automatiquement
		HAL_TIM_PWM_Stop(&this->tim_handle, TIM_CHANNEL_1);
		return;
	}
	if(!this->current_freq) {
		// stepper stopped, start pwm
		HAL_TIM_PWM_Start(&this->tim_handle, TIM_CHANNEL_1);
	}
	PWM_set_freq(this->tim_handle.Instance, hz);

	if(dir==1) {
		HAL_GPIO_WritePin(this->GPIOx_dir, this->GPIO_Pin_dir, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(this->GPIOx_dir, this->GPIO_Pin_dir, GPIO_PIN_RESET);
	}
}

void Stepper::set_speed_rps(float rps) {
	// 3200 steps per revolution
	int hz = rps * 3200.0;
	if(hz>=0) {
		this->set_speed_step_freq(hz, 1);
	}
	else {
		this->set_speed_step_freq(-hz, 0);
	}
}

Stepper::~Stepper() {
	// TODO Auto-generated destructor stub
}

