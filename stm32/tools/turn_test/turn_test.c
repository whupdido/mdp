#include "main.h"
#include "turn_test.h"
#include "ui.h"
#include "oled.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"
#include "calib.h"
#include "icm20948.h"
#include <stdio.h>
#include <math.h>

/* How many times each of the four cases is run. One sample tells you a number;
   it does not tell you whether the number is repeatable, which is the actual
   question when you are asking whether deceleration changed anything. */
#define TT_REPEATS      5u
#define TT_SETTLE_MS    700u    /* gyro integration after the move, for coast */
#define TT_ANGLE        90

typedef struct {
    const char *name;
    int8_t left;      /* 1 = left, 0 = right   */
    int8_t forward;   /* 1 = forward, 0 = back */
} tt_case_t;

static const tt_case_t TT_CASES[4] = {
    { "FL", 1, 1 },
    { "FR", 0, 1 },
    { "BL", 1, 0 },
    { "BR", 0, 0 },
};

/**
 * @brief Integrate gyro Z for a fixed window, signed the way the turn is.
 *
 * The controller stops steering BRAKING_LEAD_DEG short of the target and lets
 * the chassis coast into the remainder, so the heading it reports at the end of
 * the move is not the heading the car finishes at. Measuring the coast is the
 * difference between reporting what the controller believed and what the car
 * actually did.
 */
static float tt_integrate_coast(int8_t left, uint32_t ms) {
    float acc = 0.0f;
    uint32_t steps = ms / 10u;

    for (uint32_t i = 0; i < steps; i++) {
        float gz = icm20948_read_gyro_z();      /* deg/s, bias removed */
        acc += gz * 0.01f;
        HAL_Delay(10);
    }
    return left ? acc : -acc;
}

/**
 * @brief Run one turn and measure it.
 * @param radius_mm  out: radius of the arc the car's centre traced
 * @param angle_deg  out: total heading change including the coast
 * @return 1 if the move completed, 0 if it was aborted
 */
static uint8_t tt_run_one(const tt_case_t *c, float *radius_mm, float *angle_deg) {
    encoders_reset();

    uint8_t ok = move_turn_deg(c->left, c->forward, TT_ANGLE);

    /* Capture the coast before reading anything else. */
    float coast = tt_integrate_coast(c->left, TT_SETTLE_MS);

    int32_t l = enc_left_total;
    int32_t r = enc_right_total;

    /* The centre of the car travels the mean of the two wheel paths. Both
       encoders read positive going forward, so this is a plain average. */
    float centre_counts = ((float)l + (float)r) * 0.5f;
    float arc_mm = fabsf(centre_counts) * MM_PER_COUNT;

    float total_deg = motion_last_turn_deg() + coast;
    float theta = fabsf(total_deg) * 3.14159265f / 180.0f;

    *angle_deg = total_deg;
    *radius_mm = (theta > 0.01f) ? (arc_mm / theta) : 0.0f;
    return ok;
}

void turn_test_run(void) {
    float radius[4][TT_REPEATS];
    float angle[4][TT_REPEATS];
    uint8_t runs[4] = {0, 0, 0, 0};
    char b[24];

    /* --- warning --- */
    OLED_Clear();
    OLED_ShowString(0,  0, (const uint8_t *)"TURN TEST");
    OLED_ShowString(0, 10, (const uint8_t *)"CAR WILL MOVE");
    OLED_ShowString(0, 20, (const uint8_t *)"clear the floor");
    OLED_ShowString(0, 40, (const uint8_t *)"Tap  = start");
    OLED_ShowString(0, 50, (const uint8_t *)"Hold = quit");
    OLED_Refresh_Gram();
    if (ui_btn_wait() == UI_BTN_HOLD) { OLED_Clear(); OLED_Refresh_Gram(); return; }

    /* --- gyro bias, with the car still ---
       Every angle below is a gyro integral, so an uncalibrated bias turns
       straight into an angle error that grows with how long the turn takes.
       Skipping this makes every later number quietly wrong. */
    OLED_Clear();
    OLED_ShowString(0,  0, (const uint8_t *)"Gyro bias");
    OLED_ShowString(0, 10, (const uint8_t *)"HOLD CAR STILL");
    OLED_Refresh_Gram();
    HAL_Delay(1500);
    icm20948_calib_gyro_bias();
    OLED_ShowString(0, 30, (const uint8_t *)"locked");
    OLED_Refresh_Gram();
    HAL_Delay(600);

    /* --- the sweep --- */
    for (uint8_t ci = 0; ci < 4u; ci++) {
        const tt_case_t *c = &TT_CASES[ci];

        for (uint8_t rep = 0; rep < TT_REPEATS; rep++) {
            OLED_Clear();
            snprintf(b, sizeof b, "%s  run %u/%u", c->name,
                     (unsigned)(rep + 1u), (unsigned)TT_REPEATS);
            OLED_ShowString(0,  0, (const uint8_t *)b);
            OLED_ShowString(0, 10, (const uint8_t *)"place car, then");
            OLED_ShowString(0, 30, (const uint8_t *)"Tap  = run");
            OLED_ShowString(0, 40, (const uint8_t *)"Hold = skip case");
            OLED_Refresh_Gram();

            if (ui_btn_wait() == UI_BTN_HOLD) break;

            OLED_Clear();
            OLED_ShowString(0, 20, (const uint8_t *)"running...");
            OLED_Refresh_Gram();

            float rad, ang;
            uint8_t ok = tt_run_one(c, &rad, &ang);
            motors_coast();

            OLED_Clear();
            snprintf(b, sizeof b, "%s run %u", c->name, (unsigned)(rep + 1u));
            OLED_ShowString(0,  0, (const uint8_t *)b);
            snprintf(b, sizeof b, "ang %.1f deg", (double)ang);
            OLED_ShowString(0, 10, (const uint8_t *)b);
            snprintf(b, sizeof b, "R   %.0f mm", (double)rad);
            OLED_ShowString(0, 20, (const uint8_t *)b);
            if (!ok) OLED_ShowString(0, 30, (const uint8_t *)"!! ABORTED");
            OLED_ShowString(0, 50, (const uint8_t *)"Tap to go on");
            OLED_Refresh_Gram();
            ui_btn_wait();

            if (ok) {
                radius[ci][runs[ci]] = rad;
                angle[ci][runs[ci]]  = ang;
                runs[ci]++;
            }
        }
    }

    /* --- summary: radius, then angle --- */
    for (uint8_t page = 0; page < 2u; page++) {
        OLED_Clear();
        OLED_ShowString(0, 0, (const uint8_t *)(page == 0u ? "RADIUS mm  mean"
                                                           : "ANGLE deg  mean"));
        for (uint8_t ci = 0; ci < 4u; ci++) {
            if (runs[ci] == 0u) {
                snprintf(b, sizeof b, "%s  skipped", TT_CASES[ci].name);
            } else {
                float sum = 0.0f, lo = 1e9f, hi = -1e9f;
                for (uint8_t i = 0; i < runs[ci]; i++) {
                    float x = (page == 0u) ? radius[ci][i] : angle[ci][i];
                    sum += x;
                    if (x < lo) lo = x;
                    if (x > hi) hi = x;
                }
                float mean = sum / (float)runs[ci];
                /* spread is the honest part: a mean from 3 runs that disagree
                   by 40 mm is not a number worth putting in calib.h */
                if (page == 0u) {
                    snprintf(b, sizeof b, "%s %4.0f +-%3.0f", TT_CASES[ci].name,
                             (double)mean, (double)((hi - lo) * 0.5f));
                } else {
                    snprintf(b, sizeof b, "%s %5.1f +-%.1f", TT_CASES[ci].name,
                             (double)mean, (double)((hi - lo) * 0.5f));
                }
            }
            OLED_ShowString(0, (uint8_t)(10u + ci * 10u), (const uint8_t *)b);
        }
        OLED_ShowString(0, 50, (const uint8_t *)(page == 0u ? "Tap = angles"
                                                            : "Tap = exit"));
        OLED_Refresh_Gram();
        ui_btn_wait();
    }

    OLED_Clear();
    OLED_Refresh_Gram();
}
