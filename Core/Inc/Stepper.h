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
	Stepper();
	/**
	 * Constructor used for speed controlled stepper
	 */
	Stepper(TIM_HandleTypeDef tim_handle_step, uint32_t tim_channel_step, GPIO_TypeDef *GPIOx_dir, uint16_t GPIO_Pin_dir);

	void set_step_freq_speed(int hz, int dir);
	void set_speed_rot_per_s(float rps);
	virtual ~Stepper();


	/**
	 * Constructor used for step controlled stepper
	 */
    Stepper(unsigned long (*get_time_us)(), GPIO_TypeDef *gpio_port_step, uint16_t gpio_pin_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir);



	/**
	 * @brief Set a new goal to reach.
	 * 
	 * @param goal Set a new goal to reach in steps. 
	 * @param keep_previous_speed Choose if current speed should be kept or we start from 0.
	 */
	void set_goal(int goal, bool keep_previous_speed = false);

	/**
	 * @brief Returns current position.
	 * 
	 * @return int the current position in steps.
	 */
	int get_pos();

	/**
	 * @brief Define the current position of the stepper in steps.
	 * 
	 * @param pos the position in steps.
	 */
  	void set_pos(int pos);

	void spin_once();

	bool is_stopped();

	void set_speed_steps_per_s(unsigned long goal_speed_sps);


private:
	TIM_HandleTypeDef tim_handle;
	GPIO_TypeDef *gpio_port_dir;
	uint16_t gpio_pin_dir;
	GPIO_TypeDef *gpio_port_step;
    uint16_t gpio_pin_step;

	uint32_t tim_channel;

	int current_dir;

    struct State {
      int pos;
      int direction; // 1 or -1
      enum {STOPPED, HIGH, LOW} state;
    } state = {0, 1, State::STOPPED};

    int goal = 0; // steps

    unsigned long goal_speed_sps = 10000; // the goal speed to reach during a movement in step/s
    unsigned long time_step; //us
    unsigned long time_high = 10; // us


    unsigned long time_start_step = 0;
    unsigned long time_start_high = 0;

    long current_speed = 0;
    long speed_when_stopped = 0;
    long max_acceleration = 500; // step/s^2

    unsigned long (*get_time_us)();
	void compute_time_step();

};

#endif /* SRC_STEPPER_H_ */
