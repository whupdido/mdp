/*
 * servo.c
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#include "servo.h"
#include "calib.h"
#include "tim.h"

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    servo_us(SERVO_CENTRE);
}

void servo_us(uint16_t us)
{
    /* Hard clamp. A servo held against its mechanical stop will
       buzz, overheat and strip its gears.                        */
    if (us < 900u)  us = 900u;
    if (us > 2100u) us = 2100u;
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, us);
}
