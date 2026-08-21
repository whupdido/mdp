#include "encoders.h"
#include "tim.h"

volatile int32_t enc_left_total  = 0;
volatile int32_t enc_right_total = 0;
volatile int32_t enc_left_delta  = 0;
volatile int32_t enc_right_delta = 0;

static uint32_t last_l = 0;    /* TIM2 is 32-bit */
static uint16_t last_r = 0;    /* TIM3 is 16-bit */

void encoders_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    encoders_reset();
}

void encoders_reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    last_l = 0;
    last_r = 0;
    enc_left_total = enc_right_total = 0;
    enc_left_delta = enc_right_delta = 0;
}

void encoders_sample(void)
{
    uint32_t now_l =           __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t now_r = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* Unsigned subtract then cast to signed handles wraparound. */
    enc_left_delta  = (int32_t)(now_l - last_l);
    enc_right_delta = -(int32_t)(int16_t)(now_r - last_r);  /* verified: right encoder counts backwards */

    last_l = now_l;
    last_r = now_r;

    enc_left_total  += enc_left_delta;
    enc_right_total += enc_right_delta;
}
