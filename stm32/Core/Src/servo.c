#include "servo.h"
#include "calib.h"
#include "tim.h"
#include "flash_storage.h"

void servo_init(void)
{
	/* 1. Pre-load 1500 us into CCR2 before the pin is energized */
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, SERVO_CENTRE);

	/* 2. Force an update event (UG bit) so CCR2 loads into the active shadow register immediately */
	HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);

	/* 3. Start the PWM output directly at 1500 us */
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
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
