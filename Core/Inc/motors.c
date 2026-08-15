/*
 * motors.c
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#include "motors.h"
#include "calib.h"
#include "tim.h"

/* AT8236 dual-input H-bridge: PWM one input, hold the other at 0.
   Motor C = TIM1 CH1/CH2 (PE9/PE11)   -> left
   Motor D = TIM1 CH3/CH4 (PE13/PE14)  -> right                  */

static void set_pair(uint32_t ch_a, uint32_t ch_b, int32_t duty)
{
    if (duty >  PWM_MAX) duty =  PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, ch_a, (uint32_t)duty);
        __HAL_TIM_SET_COMPARE(&htim1, ch_b, 0u);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, ch_a, 0u);
        __HAL_TIM_SET_COMPARE(&htim1, ch_b, (uint32_t)(-duty));
    }
}

void motor_left(int32_t duty)
{
#if INVERT_LEFT
    duty = -duty;
#endif
    set_pair(TIM_CHANNEL_1, TIM_CHANNEL_2, duty);
}

void motor_right(int32_t duty)
{
#if INVERT_RIGHT
    duty = -duty;
#endif
    set_pair(TIM_CHANNEL_3, TIM_CHANNEL_4, duty);
}

void motors_coast(void)
{
    motor_left(0);
    motor_right(0);
}

void motors_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    motors_coast();
}
