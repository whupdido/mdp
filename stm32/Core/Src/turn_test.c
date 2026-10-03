/*
 * turn_test.c -- SW1-stepped turning radius measurement.
 *
 * Every SW1 tap runs ONE 90 degree turn and stops:
 *     taps  1- 5 : forward-right  (FR)
 *     taps  6-10 : forward-left   (FL)
 *     taps 11-15 : reverse-right  (BR)
 *     taps 16-20 : reverse-left   (BL)
 *     tap  21    : summary, radius page
 *     tap  22    : summary, angle page
 *     tap  23    : start over
 *
 * An aborted turn (STALL / TIMEOUT / ABORT / BLOCKED) is not counted and the
 * same run is repeated on the next tap.
 *
 * Method, same as the archived tools/turn_test:
 *     R = arc / theta
 *   arc   = mean of the two rear-wheel encoder paths (rear-axle centre)
 *   theta = gyro heading change, INCLUDING the coast after the controller
 *           stops steering BRAKING_LEAD_DEG short of target.
 *
 * Difference from the archived version: the heading comes from the yaw the
 * 100 Hz control ISR already integrates (motion_yaw_deg()), not from reading
 * the IMU here as well. Reading the IMU from main while TIM6 reads it over the
 * same I2C bus can collide mid-transfer and silently return 0 deg/s.
 *
 * Every result is also sent on USART3 as a "[TT]" line so a serial terminal
 * can log it. Anything parsing replies already skips unknown lines.
 */

#include "main.h"
#include "turn_test.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"
#include "command.h"
#include "calib.h"
#include "oled.h"
#include "icm20948.h"
#include <stdio.h>
#include <math.h>

#define TT_REPEATS        5u
#define TT_ANGLE          90
#define TT_START_DELAY_MS 800u   /* after the tap, so your hand is off the car */
#define TT_SETTLE_MS      700u   /* keep integrating after the move: the coast */
#define TT_NCASES         4u

extern int calibrated;           /* main.c: set once the gyro bias is locked */

typedef struct {
    const char *name;
    int8_t left;      /* 1 = left, 0 = right   */
    int8_t forward;   /* 1 = forward, 0 = back */
} tt_case_t;

/* The order you asked for: FR, FL, BR, BL. */
static const tt_case_t TT_CASES[TT_NCASES] = {
    { "FR", 0, 1 },
    { "FL", 1, 1 },
    { "BR", 0, 0 },
    { "BL", 1, 0 },
};

/* TT_SUMMARY_R / _A mean "the next tap shows that page". */
typedef enum { TT_RUNNING = 0, TT_SUMMARY_R, TT_SUMMARY_A, TT_DONE } tt_phase_t;

static tt_phase_t tt_phase = TT_RUNNING;
static uint8_t    tt_case  = 0;
static uint8_t    tt_rep   = 0;
static float      tt_radius[TT_NCASES][TT_REPEATS];
static float      tt_angle [TT_NCASES][TT_REPEATS];

static void tt_line(uint8_t y, const char *s) {
    OLED_ShowString(0, y, (const uint8_t *)s);
}

static const char *tt_result_name(move_result_t r) {
    switch (r) {
        case MOVE_DONE:    return "DONE";
        case MOVE_STALL:   return "STALL";
        case MOVE_TIMEOUT: return "TIMEOUT";
        case MOVE_ABORT:   return "ABORT";
        case MOVE_BLOCKED: return "BLOCKED";
        default:           return "?";
    }
}

static void tt_calibrate(void) {
    OLED_Clear();
    tt_line(0,  "Gyro bias");
    tt_line(10, "HOLD CAR STILL");
    OLED_Refresh_Gram();
    HAL_Delay(1500);
    icm20948_calib_gyro_bias();
    calibrated = 1;
    tt_line(30, "locked");
    OLED_Refresh_Gram();
    HAL_Delay(600);
}

/**
 * @brief Run one turn and measure it.
 * @return the move result; radius/angle are only meaningful on MOVE_DONE
 */
static move_result_t tt_run_one(const tt_case_t *c, float *radius_mm,
                                float *angle_deg, int32_t *dl, int32_t *dr) {
    int32_t l0   = enc_left_total;
    int32_t r0   = enc_right_total;
    float   yaw0 = motion_yaw_deg();

    move_turn_deg(c->left, c->forward, TT_ANGLE);
    move_result_t res = motion_result();
    motors_coast();

    /* The ISR keeps integrating yaw and encoders while idle, so waiting here
       captures the coast in both the angle and the arc. */
    HAL_Delay(TT_SETTLE_MS);

    *dl = enc_left_total  - l0;
    *dr = enc_right_total - r0;
    float yaw = motion_yaw_deg() - yaw0;

    /* Sign the angle the way the controller does, so a good turn reads +90
       whichever of the four it is. */
    float s = (c->left ? 1.0f : -1.0f) * (c->forward ? 1.0f : -1.0f);
    *angle_deg = yaw * s;

    float arc_mm = fabsf(((float)*dl + (float)*dr) * 0.5f) * MM_PER_COUNT;
    float theta  = fabsf(yaw) * 3.14159265f / 180.0f;
    *radius_mm   = (theta > 0.01f) ? (arc_mm / theta) : 0.0f;
    return res;
}

static void tt_stats(const float *x, uint8_t n, float *mean, float *half_range) {
    float sum = 0.0f, lo = 1e9f, hi = -1e9f;
    for (uint8_t i = 0; i < n; i++) {
        sum += x[i];
        if (x[i] < lo) lo = x[i];
        if (x[i] > hi) hi = x[i];
    }
    *mean       = sum / (float)n;
    *half_range = (hi - lo) * 0.5f;
}

static void tt_show_summary(uint8_t angle_page) {
    char b[24];
    char u[64];

    OLED_Clear();
    tt_line(0, angle_page ? "ANGLE deg  mean" : "RADIUS mm  mean");
    for (uint8_t ci = 0; ci < TT_NCASES; ci++) {
        float mean, hr;
        tt_stats(angle_page ? tt_angle[ci] : tt_radius[ci], TT_REPEATS, &mean, &hr);
        if (angle_page) {
            snprintf(b, sizeof b, "%s %5.1f +-%.1f", TT_CASES[ci].name,
                     (double)mean, (double)hr);
        } else {
            snprintf(b, sizeof b, "%s %4.0f +-%3.0f", TT_CASES[ci].name,
                     (double)mean, (double)hr);
        }
        tt_line((uint8_t)(10u + ci * 10u), b);

        snprintf(u, sizeof u, "[TT] SUMMARY %s %s mean=%.1f +-%.1f\r\n",
                 TT_CASES[ci].name, angle_page ? "angle_deg" : "radius_mm",
                 (double)mean, (double)hr);
        command_send(u);
    }
    tt_line(50, angle_page ? "SW1 = restart" : "SW1 = angles");
    OLED_Refresh_Gram();
}

void turn_test_show_idle(void) {
    char b[24];

    OLED_Clear();
    tt_line(0, "TURN TEST");
    snprintf(b, sizeof b, "next: %s %u/%u", TT_CASES[tt_case].name,
             (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
    tt_line(10, b);
    tt_line(30, "SW1 = turn 90");
    if (!calibrated) tt_line(40, "(gyro cal 1st)");
    tt_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

void turn_test_step(void) {
    char b[24];
    char u[96];

    /* --- summary pages --- */
    if (tt_phase == TT_SUMMARY_R) {
        tt_show_summary(0);
        tt_phase = TT_SUMMARY_A;
        return;
    }
    if (tt_phase == TT_SUMMARY_A) {
        tt_show_summary(1);
        tt_phase = TT_DONE;
        return;
    }
    if (tt_phase == TT_DONE) {
        tt_phase = TT_RUNNING;
        tt_case  = 0;
        tt_rep   = 0;
        command_send("[TT] restart\r\n");
        turn_test_show_idle();
        return;
    }

    /* --- one turn --- */
    /* Every angle is a gyro integral, so an uncalibrated bias becomes an
       angle error that grows with how long the turn takes. */
    if (!calibrated) tt_calibrate();

    const tt_case_t *c = &TT_CASES[tt_case];

    OLED_Clear();
    snprintf(b, sizeof b, "%s run %u/%u", c->name,
             (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
    tt_line(0, b);
    tt_line(20, "running...");
    OLED_Refresh_Gram();
    HAL_Delay(TT_START_DELAY_MS);

    float rad, ang;
    int32_t dl, dr;
    move_result_t res = tt_run_one(c, &rad, &ang, &dl, &dr);

    snprintf(u, sizeof u,
             "[TT] %s run=%u result=%s radius_mm=%.1f angle_deg=%.2f encL=%ld encR=%ld\r\n",
             c->name, (unsigned)(tt_rep + 1u), tt_result_name(res),
             (double)rad, (double)ang, (long)dl, (long)dr);
    command_send(u);

    OLED_Clear();
    snprintf(b, sizeof b, "%s run %u/%u", c->name,
             (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
    tt_line(0, b);
    snprintf(b, sizeof b, "R   %.0f mm", (double)rad);
    tt_line(10, b);
    snprintf(b, sizeof b, "ang %.1f deg", (double)ang);
    tt_line(20, b);

    if (res != MOVE_DONE) {
        /* Not counted: position is unknown, so the numbers mean nothing. */
        snprintf(b, sizeof b, "!! %s", tt_result_name(res));
        tt_line(30, b);
        tt_line(50, "SW1 = retry");
        OLED_Refresh_Gram();
        return;
    }

    tt_radius[tt_case][tt_rep] = rad;
    tt_angle [tt_case][tt_rep] = ang;

    /* advance */
    tt_rep++;
    if (tt_rep >= TT_REPEATS) {
        tt_rep = 0;
        tt_case++;
    }

    if (tt_case >= TT_NCASES) {
        tt_line(50, "SW1 = summary");
        tt_phase = TT_SUMMARY_R;
        /* summary is drawn on the next tap */
    } else {
        snprintf(b, sizeof b, "next: %s %u/%u", TT_CASES[tt_case].name,
                 (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
        tt_line(40, b);
        tt_line(50, "SW1 = go");
    }
    OLED_Refresh_Gram();
}
