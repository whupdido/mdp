#include "main.h"
#include "sensors.h"
#include "calib.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim8;

/* Variables for Input Capture */
volatile uint8_t  capture_state = 0;
volatile uint32_t echo_val1 = 0;
volatile uint32_t echo_val2 = 0;
volatile float    ultrasonic_distance_cm = 0.0f;
volatile uint8_t image_found = 0;

/*
 * The DMA will automatically dump the 12-bit ADC values here.
 * index 0 = Rank 1 (PA2 / Right IR)
 * index 1 = Rank 2 (PA3 / Left  IR)
 * IR_RIGHT and IR_LEFT below ARE those indices, so the buffer, the fit arrays
 * and the getters all use one numbering instead of swapping it per function.
 */
volatile uint16_t ir_adc_buffer[2] = {0, 0};

#define IR_RIGHT   0u
#define IR_LEFT    1u

/* Make sure you have access to your ADC handle (defined in adc.c) */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/**
 * @brief Start the ADC free-running into ir_adc_buffer, with the DMA interrupt
 *        kept masked.
 *
 * HAL_ADC_Start_DMA enables the DMA half- and full-transfer interrupts, and at
 * these conversion rates that is ruinous. Two channels free-run into a two
 * entry circular buffer, so at the stock 84 cycle sampling time a conversion
 * lands every ~4.6 us and the CPU takes a DMA interrupt that often, spending a
 * large slice of every second inside HAL_DMA_IRQHandler for nothing.
 *
 * At the 3 cycle sampling time the source test uses it stops being merely
 * wasteful: an interrupt is requested every ~0.7 us, which is faster than the
 * handler can run. The CPU never leaves the ISR, and since DMA2_Stream0 sits at
 * NVIC priority 0 while SysTick is at 15, SysTick stops being serviced, uwTick
 * freezes, and HAL_Delay never returns. That is precisely what left the source
 * test stuck on "sampling...".
 *
 * Nothing here uses the DMA transfer callbacks, the buffer is polled, so the
 * interrupt has no job. Masked, the DMA keeps refilling the buffer purely in
 * hardware with zero CPU cost, which also gives the rest of the firmware back
 * the cycles it was quietly losing.
 */
static uint8_t ir_dma_start(void) {
    uint8_t ok = (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ir_adc_buffer, 2) == HAL_OK);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    return ok;
}

/* Live model coefficients, one pair per sensor, seeded from calib.h.
   Fixed at the calib.h values; the on-car calibration UI that used to rewrite
   them at run time has been removed. */
static float ir_m[2] = { IR_RIGHT_M, IR_LEFT_M };
static float ir_b[2] = { IR_RIGHT_B, IR_LEFT_B };
static float ir_k[2] = { IR_RIGHT_K, IR_LEFT_K };

/* Set by ir_sensors_init(). A failed DMA start leaves the buffer at zero
   forever, which is indistinguishable from a dead sensor unless you check. */
static uint8_t ir_dma_started = 0;

/**
 * @brief Call this ONCE before your main loop starts to kick off the background DMA
 */
void ir_sensors_init(void) {
    ir_dma_started = ir_dma_start();
}

/**
 * @brief Re-arm the ADC/DMA after a failed or aborted start.
 */
uint8_t ir_sensors_restart(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    ir_dma_started = ir_dma_start();
    return ir_dma_started;
}

/**
 * @brief Helper to convert raw 12-bit ADC (0-4095) to Voltage (0.0 - 3.3V)
 */
static float adc_to_voltage(uint16_t raw_adc) {
    return ((float)raw_adc / 4095.0f) * 3.3f;
}

/**
 * @brief Instantaneous voltage on one IR channel.
 */
static float ir_volts(uint8_t ch) {
    return adc_to_voltage(ir_adc_buffer[ch]);
}

/**
 * @brief The transfer function: d = m/(V - b), clamped to the trusted band.
 */
static float ir_volts_to_cm(uint8_t ch, float voltage) {
    float denom = voltage - ir_b[ch];

    /* Far away, or nothing plugged in: the model blows up here, so bail out
       to the far limit instead of returning a huge or negative distance. */
    if (denom < 0.01f) return IR_MAX_CM;

    float distance_cm = ir_m[ch] / denom - ir_k[ch];

    if (distance_cm > IR_MAX_CM) distance_cm = IR_MAX_CM;
    if (distance_cm < IR_MIN_CM) distance_cm = IR_MIN_CM;
    return distance_cm;
}

/* --- Public Getters for your Control Loop --- */

float get_sharp_ir_left_cm(void) {
    return ir_volts_to_cm(IR_LEFT, ir_volts(IR_LEFT));
}

float get_sharp_ir_right_cm(void) {
    return ir_volts_to_cm(IR_RIGHT, ir_volts(IR_RIGHT));
}

/**
 * @brief Formats the voltages into strings for your OLED screen.
 */
void display_ir_voltages_oled(void) {
    float left_v   = ir_volts(IR_LEFT);
    float right_v  = ir_volts(IR_RIGHT);
    float left_d   = ir_volts_to_cm(IR_LEFT,  left_v);
    float right_d  = ir_volts_to_cm(IR_RIGHT, right_v);

    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];

    sprintf(line1, "Left : %.2fV", left_v);
    sprintf(line3, "Distance: %.2f", left_d);
    sprintf(line2, "Right: %.2fV", right_v);
    sprintf(line4, "Distance: %.2f", right_d);

    OLED_Clear();
    OLED_ShowString(0,  0, (const uint8_t *)line1);
    OLED_ShowString(0, 10, (const uint8_t *)line3);
    OLED_ShowString(0, 20, (const uint8_t *)line2);
    OLED_ShowString(0, 30, (const uint8_t *)line4);
    OLED_Refresh_Gram();
}

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

/**
 * @brief Checks if the forward path is blocked using the HC-SR04.
 * @return 1 if collision is imminent, 0 if path is clear.
 */
uint8_t check_front_collision(void)
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
