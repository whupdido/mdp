#include "servo.h"
#include "calib.h"
#include "tim.h"

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    servo_us(SERVO_CENTRE);
}

void servo_us(uint16_t us)
{
    if (us < 900u)  us = 900u;
    if (us > 2100u) us = 2100u;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}
