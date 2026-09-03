#include "main.h"
#include "sensors.h"
#include "oled.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim8;

/* Variables for Input Capture */
volatile uint8_t  capture_state = 0;
volatile uint32_t echo_val1 = 0;
volatile uint32_t echo_val2 = 0;
volatile float    ultrasonic_distance_cm = 0.0f;
volatile uint8_t image_found = 0;

/**
 * @brief Sends the 10us trigger pulse and force-resets the state machine.
 */
void trigger_ultrasonic(void)
{
    /* 1. Force hardware reset (Prevents permanent lockups if an echo is lost) */
    HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_2);
    capture_state = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

    /* 2. Send the TRIG pulse */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    /* 'volatile' guarantees the compiler won't delete this delay loop */
    volatile uint32_t delay = 2000;
    while(delay--) {
        __NOP(); /* No-Operation command to kill clock cycles */
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    /* 3. Start listening for the returning sound wave */
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
}

/**
 * @brief Hardware Interrupt Callback. Runs automatically when ECHO changes state.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (capture_state == 0)
        {
            /* 1st interrupt: Rising Edge */
            echo_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            capture_state = 1;

            /* BUG FIX: Stop, swap polarity, clear phantom flags, restart */
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2); /* Clears the false edge glitch! */
            HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
        }
        else if (capture_state == 1)
        {
            /* 2nd interrupt: Falling Edge */
            echo_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            uint16_t diff = (uint16_t)echo_val2 - (uint16_t)echo_val1;
            ultrasonic_distance_cm = (float)diff / 58.0f;

            /* Reset hardware for the next trigger */
            capture_state = 0;
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
        }
    }
}

uint8_t check_rear_collision(void)
{
    /* 1. Fire a new pulse for the NEXT check */
    trigger_ultrasonic();

    /* 2. Read the global variable updated by the interrupt */
    if (ultrasonic_distance_cm > 0.0f && ultrasonic_distance_cm < 10.0f) {
    	OLED_Clear();
    	char buf[32];
    	snprintf(buf, sizeof(buf), "Object %fcm away!", ultrasonic_distance_cm);
    	OLED_ShowString(0,0,(const uint8_t* ) buf);
    	OLED_Refresh_Gram();
        return 1; /* Collision imminent */
    }

    return 0; /* Path clear */
}

/**
 * @brief Infinite loop to test the HC-SR04 and display on the OLED.
 *        WARNING: This is a blocking loop. Call this in main() for testing only.
 */
void test_ultrasonic_oled(void)
{
    char buf[32];

    /* Setup the static part of the OLED screen */
    OLED_Clear();
    OLED_ShowString(0, 0, (const uint8_t *)"HC-SR04 Test");
    OLED_Refresh_Gram();

    while (1)
    {
        /* 1. Send the trigger pulse */
        trigger_ultrasonic();

        /* 2. Wait for sound to travel and interrupt to fire.
         * The HC-SR04 requires a minimum of 50ms between triggers so
         * returning echoes from the previous ping don't overlap the new one. */
        HAL_Delay(60);

        /* 3. Format the distance string
         * We add a few trailing spaces "   " to overwrite any leftover characters
         * if the string length shrinks (e.g., going from 100.5 to 9.5). */
        snprintf(buf, sizeof(buf), "Dist: %.1f cm   ", ultrasonic_distance_cm);

        /* 4. Display the live reading on the next line (Y = 20) */
        OLED_ShowString(0, 20, (const uint8_t *)buf);
        OLED_Refresh_Gram();

        /* Optional: Print to PuTTY as well */
        /*
        command_send(buf);
        command_send("\r\n");
        */
    }
}

/**
 * @brief Checks if the forward path is blocked (Using dual IRs)
 */
//uint8_t check_front_collision(void)
//{
//    /* Read analog values and convert to CM (requires your ADC conversion code) */
//    float left_dist_cm = get_sharp_ir_left_cm();
//    float right_dist_cm = get_sharp_ir_right_cm();
//
//    /* If EITHER sensor sees an obstacle closer than 12 cm, abort! */
//    /* (We use 12cm so it stops before entering the 10cm IR blind spot) */
//    if (left_dist_cm < 12.0f || right_dist_cm < 12.0f) {
//        return 1;
//    }
//    return 0;
//}
