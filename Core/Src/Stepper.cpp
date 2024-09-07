/*
 * Stepper.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: arusso
 */

#include "Stepper.h"

#define SYS_CORE_CLOCK_HZ 170000000.
#define SYS_CORE_CLOCK_MHZ 170.


void PWM_set_high_duration(TIM_TypeDef *timx, int us) {
	timx->CCR1 = SYS_CORE_CLOCK_MHZ/(timx->PSC+1) * us;
}

void PWM_set_freq(TIM_TypeDef *timx, int hz) {
	int arr = SYS_CORE_CLOCK_HZ/((timx->PSC+1)*hz);
	timx->CNT = 0;
	timx->ARR = arr;
}

Stepper::Stepper() {}

// SPEED CONTROLLED STEPPER
Stepper::Stepper(TIM_HandleTypeDef tim_handle_step, uint32_t tim_channel_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir) 
{
	this->tim_handle = tim_handle_step;
	this->gpio_port_dir = gpio_port_dir;
	this->gpio_pin_dir = gpio_pin_dir;
	this->tim_channel = tim_channel_step;
	HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&this->tim_handle, this->tim_channel);

	this->current_dir = 0;
	this->state.state = State::STOPPED;
}
	
// STEP CONTROLLED STEPPER
Stepper::Stepper(unsigned long (*get_time_us)(), GPIO_TypeDef *gpio_port_step, uint16_t gpio_pin_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir)
{
	this->gpio_port_step = gpio_port_step;
    this->gpio_pin_step = gpio_pin_step;
    this->gpio_port_dir = gpio_port_dir;
    this->gpio_pin_dir = gpio_pin_dir;

    this->get_time_us = get_time_us;

    HAL_GPIO_WritePin(this->gpio_port_step, this->gpio_pin_step, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_RESET);

}

void Stepper::set_step_freq_speed(int hz, int dir) {
	if(hz < 15){// todo calculer freq min automatiquement
		if(!this->state.state == State::STOPPED) {
			PWM_set_high_duration(this->tim_handle.Instance, 0);
			this->state.state = State::STOPPED;
		}
		return;
	}
	if(this->state.state == State::STOPPED) {
		// stepper stopped, start pwm
		PWM_set_high_duration(this->tim_handle.Instance, 10);
		this->state.state = State::HIGH;
	}
	PWM_set_freq(this->tim_handle.Instance, hz);

	if(dir==1) {
		HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_RESET);
	}
}

void Stepper::set_speed_rot_per_s(float rps) {
	// 3200 steps per revolution
	int hz = rps * 3200.0;
	if(hz>=0) {
		this->set_step_freq_speed(hz, 1);
	}
	else {
		this->set_step_freq_speed(-hz, 0);
	}
}

Stepper::~Stepper() {
	// TODO Auto-generated destructor stub
}

void Stepper::set_speed_steps_per_s(unsigned long goal_speed_sps) {
	this->goal_speed_sps = goal_speed_sps;
	this->time_step = 10000000 / goal_speed_sps; 
}

void Stepper::set_goal(int goal, bool keep_previous_speed)
{
    this->goal = goal;
    this->state.state = State::HIGH;
    this->state.direction = goal > this->state.pos ? 1 : -1;
    time_start_step = get_time_us();
    time_start_high = this->time_start_step;
    HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, state.direction == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (keep_previous_speed) {
      this->current_speed = this->goal_speed_sps;
    } else {
      this->current_speed = 0;
    }
}

int Stepper::get_pos()
{
    return this->state.pos;
}

void Stepper::set_pos(int pos)
{
	this->state.pos = pos;
}

void Stepper::spin_once()
{
    if (this->state.state == State::STOPPED) {
      return;
    }

    compute_time_step();

    if (this->state.state == State::HIGH) {
      if (get_time_us() - this->time_start_high > this->time_high) {
        // Set low
        HAL_GPIO_WritePin(this->gpio_port_step, this->gpio_pin_step, GPIO_PIN_RESET);
        this->state.state = State::LOW;
      }
    } else if (this->state.state == State::LOW) {
      if (get_time_us() - this->time_start_step > this->time_step) {
        // Set high
        HAL_GPIO_WritePin(this->gpio_port_step, this->gpio_pin_step, GPIO_PIN_SET);
        this->state.state = State::HIGH;
        this->time_start_high = get_time_us();
        this->time_start_step = get_time_us();
        this->state.pos += state.direction;
      }
    }

    if (this->state.pos == goal) {
      this->state.state = State::STOPPED;
      this->speed_when_stopped = this->current_speed;
      this->current_speed = 0;
    }
}

bool Stepper::is_stopped()
{
    return this->state.state == State::STOPPED;
}


void Stepper::compute_time_step() {

	// We compute the new speed every 5ms (return if we are not there yet)
	static unsigned long last_time = 0;
	if (get_time_us() - last_time < 5000 && last_time != 0) {
		return;
	}
	last_time = get_time_us();


	if (this->current_speed == this->goal_speed_sps) {
		return;
	}

	if (this->current_speed < this->goal_speed_sps) {
		this->current_speed += (long) (((double) this->max_acceleration) * 0.005); // 5ms (0.005s
		if (this->current_speed > this->goal_speed_sps) {
		this->current_speed = this->goal_speed_sps;
		}
	} else {
		this->current_speed -= this->max_acceleration;
		if (this->current_speed < this->goal_speed_sps) {
		this->current_speed = this->goal_speed_sps;
		}
	}

	this->time_step = 10000000 / this->current_speed;
}
