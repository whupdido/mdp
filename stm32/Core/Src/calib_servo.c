/*
 * calib_servo.c -- safe automated steering & turn calibration
 * NOT USED ANYMORE
 */

#include "calib_servo.h"
#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "command.h"
#include "calib.h"
#include "oled.h"
#include "icm20948.h"
#include "flash_storage.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void safe_servo_us(uint16_t us)
{
    if (us < SERVO_SAFE_MIN_US) us = SERVO_SAFE_MIN_US;
    if (us > SERVO_SAFE_MAX_US) us = SERVO_SAFE_MAX_US;
    servo_us(us);
}

/**
  * @brief Drives forward a fixed distance in open loop and measures wheel delta.
  */
static int32_t run_straight_sample(uint16_t servo_pwm, int32_t target_ticks)
{
    safe_servo_us(servo_pwm);
    HAL_Delay(250);

    encoders_reset();

    /* Gentle ramp-up to prevent wheel slip */
    for (int32_t d = 0; d <= 3200; d += 200) {
        motor_left(d);
        motor_right(d);
        HAL_Delay(10);
    }

    /* Drive until target distance is reached */
    while (((abs(enc_left_total) + abs(enc_right_total)) / 2) < target_ticks) {
        HAL_Delay(10);
    }

    /* Active brake to stop */
    motor_left(0);
    motor_right(0);
    HAL_Delay(350);

    return (enc_right_total - enc_left_total);
}

uint16_t auto_calib_servo_centre(void)
{
    uint16_t pwm = 1500;
    /* 1. Reduce forward track length to 250 mm */
    int32_t target_ticks = (int32_t)(250.0f / MM_PER_COUNT);
    char buf[96];

    command_send("\r\n--- Step 1: Straight-Line Centre Calibration (Compact) ---\r\n");

    for (int iter = 0; iter < 4; iter++) {
        /* Run forward test drive */
        int32_t diff = run_straight_sample(pwm, target_ticks);

        snprintf(buf, sizeof(buf), "[Iter %d] PWM: %u us | Delta(R-L): %ld ticks\r\n",
                 iter + 1, pwm, (long)diff);
        command_send(buf);

        /* Error threshold: within +/- 6 counts over 250mm */
        if (abs(diff) <= 6) {
            command_send(">> Centre locked within tolerance.\r\n");
            break;
        }

        /* Proportional adjustment */
        int16_t adjustment = (int16_t)(diff / 3);
        if (adjustment > 20)  adjustment = 20;
        if (adjustment < -20) adjustment = -20;
        pwm += adjustment;

        /* Enforce safe search bounds */
        if (pwm < SERVO_CENTRE_SEARCH_MIN) pwm = SERVO_CENTRE_SEARCH_MIN;
        if (pwm > SERVO_CENTRE_SEARCH_MAX) pwm = SERVO_CENTRE_SEARCH_MAX;

        /* --- 2. Closed-Loop Reverse Back to Start Line --- */
        safe_servo_us(1500);
        HAL_Delay(150);

        encoders_reset();
        motor_left(-3000);
        motor_right(-3000);

        /* Reverse until the robot covers the exact distance it drove forward */
        while (((abs(enc_left_total) + abs(enc_right_total)) / 2) < target_ticks) {
            HAL_Delay(10);
        }

        /* Brake at the starting line */
        motor_left(0);
        motor_right(0);
        HAL_Delay(400);
    }

    return pwm;
}

uint32_t auto_calib_turn_90_deg(uint16_t steer_pwm, int8_t direction)
{
    char buf[96];
    safe_servo_us(steer_pwm);
    HAL_Delay(300);

    encoders_reset();

    /* Drive turning arc at low speed */
    motor_left(2200);
    motor_right(2200);

    /* Run sample arc */
    HAL_Delay(1600);

    motor_left(0);
    motor_right(0);
    HAL_Delay(300);

    uint32_t turn_ticks = (abs(enc_left_total) + abs(enc_right_total)) / 2;
    snprintf(buf, sizeof(buf), ">> %s 90-deg Arc Measured Ticks: %lu\r\n",
             (direction < 0) ? "LEFT" : "RIGHT", (unsigned long)turn_ticks);
    command_send(buf);

    return turn_ticks;
}

void display_calib_results_oled(const ServoCalibResult_t *res)
{
    char line[22];

    OLED_Clear();

    /* Line 0: Header */
    OLED_ShowString(0, 0, (const uint8_t *)" CALIB RESULTS ");

    /* Line 1: Steering Center & Limits (e.g., "C:1500 L:1150 R:1850") */
    snprintf(line, sizeof(line), "C:%u L:%u", res->centre_us, res->left_steer_us);
    OLED_ShowString(0, 2, (const uint8_t *)line);

    snprintf(line, sizeof(line), "R:%u", res->right_steer_us);
    OLED_ShowString(0, 4, (const uint8_t *)line);

    /* Line 3: 90-Degree Turn Counts (e.g., "FL:3350 FR:4150") */
//    snprintf(line, sizeof(line), "FL:%lu FR:%lu",
//             (unsigned long)res->turn_counts_fl,
//             (unsigned long)res->turn_counts_fr);
    OLED_ShowString(0, 6, (const uint8_t *)line);
    OLED_Refresh_Gram();
}

/**
  * @brief Drives in an arc and stops automatically when the integrated
  *        yaw heading reaches exactly 90.0 degrees.
  * @retval Measured encoder counts required to complete 90 degrees.
  */
uint32_t auto_calib_turn_90_imu(uint16_t steer_pwm, int8_t direction)
{
    char buf[96];
    float current_yaw = 0.0f;
    uint32_t start_time = HAL_GetTick();
    uint32_t last_time = start_time;

    safe_servo_us(steer_pwm);
    HAL_Delay(300);

    encoders_reset();

    /* Ramp up drive motors */
    for (int32_t d = 0; d <= 3000; d += 200) {
        motor_left(d);
        motor_right(d);
        HAL_Delay(10);
    }

    /* Turn until 90 degrees reached OR safety limits hit */
    while (fabsf(current_yaw) < 90.0f) {
        uint32_t now = HAL_GetTick();
        float dt = (float)(now - last_time) / 1000.0f;
        last_time = now;

        float gz = icm20948_read_gyro_z(); /* deg/sec */

        /* Account for gyro sign: turning left yields positive or negative gz */
        if (direction < 0) {
            current_yaw += fabsf(gz) * dt; /* Left */
        } else {
            current_yaw += fabsf(gz) * dt; /* Right */
        }

        /* --- HARD SAFETY CUTOFFS --- */
        /* 1. Max run time ceiling (5.0 seconds) */
        if ((now - start_time) > 5000u) {
            command_send("WARNING: Turn timeout reached (5s) -- stopping motors!\r\n");
            break;
        }

        /* 2. Max encoder count ceiling (approx 1.5 full turns) */
        uint32_t avg_ticks = (abs(enc_left_total) + abs(enc_right_total)) / 2;
        if (avg_ticks > 6000u) {
            command_send("WARNING: Encoder tick ceiling reached -- stopping motors!\r\n");
            break;
        }

        HAL_Delay(5); /* 200 Hz integration */
    }

    /* Active brake to stop */
    motor_left(0);
    motor_right(0);
    safe_servo_us(1500);
    HAL_Delay(300);

    uint32_t turn_ticks = (abs(enc_left_total) + abs(enc_right_total)) / 2;

    snprintf(buf, sizeof(buf), ">> Turn Result: Final Yaw = %.1f deg, Ticks = %lu\r\n",
             current_yaw, (unsigned long)turn_ticks);
    command_send(buf);

    return turn_ticks;
}

void run_servo_autocalibration(void)
{
    ServoCalibResult_t res;

    /* 1. Calibrate Gyro stationary bias */
    command_send("\r\n[IMU] Calibrating Gyro Zero Bias (keep robot stationary)...\r\n");
    OLED_ShowString(0,0, (const uint8_t* ) "Calibrating Gyro...");
    OLED_Refresh_Gram();
    icm20948_calib_gyro_bias();
    command_send("[IMU] Gyro bias locked.\r\n");

    /* 2. Centre Trim */
    OLED_ShowString(0,5, (const uint8_t* ) "Calibrating Centre...");
	OLED_Refresh_Gram();
    res.centre_us = auto_calib_servo_centre();
    res.left_steer_us  = SERVO_LEFT;
    res.right_steer_us = SERVO_RIGHT;

//    /* 3. Automatic Left 90-Deg Turn */
//    command_send("\r\n[Step 2/3] Auto-measuring LEFT 90-deg turn arc...\r\n");
//    OLED_ShowString(0,10, (const uint8_t* ) "Calibrating Left...");
//	OLED_Refresh_Gram();
//    HAL_Delay(1000);
//    res.turn_counts_fl = auto_calib_turn_90_imu(res.left_steer_us, -1);
//
//    /* 4. Automatic Right 90-Deg Turn */
//    command_send("\r\n[Step 3/3] Auto-measuring RIGHT 90-deg turn arc...\r\n");
//    OLED_ShowString(0,15, (const uint8_t* ) "Calibrating Right...");
//	OLED_Refresh_Gram();
//    HAL_Delay(1000);
//    res.turn_counts_fr = auto_calib_turn_90_imu(res.right_steer_us, 1);

    FlashCalibData_t new_data = {
        .magic          = CALIB_MAGIC_KEY,
        .centre_us      = res.centre_us,
        .left_us        = res.left_steer_us,
        .right_us       = res.right_steer_us
        //.turn_counts_fl = res.turn_counts_fl,
        //.turn_counts_fr = res.turn_counts_fr
    };

    if (flash_calib_save(&new_data)) {
        command_send(">> Calibration successfully saved to Flash Sector 11!\r\n");
    } else {
        command_send(">> ERROR: Flash write failed!\r\n");
    }

    /* Return straight */
    safe_servo_us(res.centre_us);

    /* 5. Display Summary */
    display_calib_results_oled(&res);
}
