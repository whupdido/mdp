#include "motors.h"
#include "calib.h"
#include "tim.h"

/* ---------------------------------------------------------------------------
 * Motor A (left)  = PB8 TIM10_CH1 (AF3)  +  PB9 TIM11_CH1 (AF3)
 * Motor B (right) = PE5 TIM9_CH1  (AF3)  +  PE6 TIM9_CH2  (AF3)
 *
 * Note that motor A's two bridge inputs live on two SEPARATE timers. That is
 * fine -- they are never both PWM'd at once, so they do not need to be
 * synchronised.
 *
 * AT8236 dual-input H-bridge truth table:
 *
 *      IN1  IN2   OUT1 OUT2   function
 *       0    0     Z    Z     coast (outputs high-Z, wheel free)
 *       1    0     H    L     forward
 *       0    1     L    H     reverse
 *       1    1     L    L     brake
 *
 *
 * ------------------------------------------------------------------------- */

static void set_pair(TIM_HandleTypeDef *ha, uint32_t ch_a,
                     TIM_HandleTypeDef *hb, uint32_t ch_b, int32_t duty)
{
    if (duty >  PWM_MAX) duty =  PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(ha, ch_a, (uint32_t)PWM_MAX);
        __HAL_TIM_SET_COMPARE(hb, ch_b, (uint32_t)(PWM_MAX - duty));
    } else {
        __HAL_TIM_SET_COMPARE(hb, ch_b, (uint32_t)PWM_MAX);
        __HAL_TIM_SET_COMPARE(ha, ch_a, (uint32_t)(PWM_MAX + duty));
    }
}

void motor_left(int32_t duty)
{
#if INVERT_LEFT
    duty = -duty;
#endif
    set_pair(&htim10, TIM_CHANNEL_1, &htim11, TIM_CHANNEL_1, duty);
}

void motor_right(int32_t duty)
{
#if INVERT_RIGHT
    duty = -duty;
#endif
    set_pair(&htim9, TIM_CHANNEL_1, &htim9, TIM_CHANNEL_2, duty);
}

void motors_brake(void)
{
    motor_left(0);
    motor_right(0);
}

void motors_coast(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_2, 0u);
}

void motors_init(void)
{
    /* Park the bridges in COAST before the outputs go live, so nothing
       twitches between HAL_TIM_PWM_Start() and the first real command. */
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_2, 0u);

    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    motors_coast();
}
