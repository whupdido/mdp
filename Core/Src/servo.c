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
    /* Clamp to the verified mechanical range. Commanding outside this does
       not steer any further -- it just stalls the servo against its stop
       for the whole duration of the move. Keep these in step with
       SERVO_LEFT / SERVO_RIGHT in calib.h. */
    if (us < 1000u) us = 1000u;
    if (us > 2100u) us = 2100u;

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}
