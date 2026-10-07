/*
 * turn_test.c -- SW1-stepped turning radius measurement.
 *
 * Every SW1 tap runs ONE turn of TT_ANGLE degrees and stops. TT_ANGLE is 30,
 * the turn the planner drives (FL030, FR030, BL030, BR030); set it to 90 for
 * the original 90 degree test.
 *     taps  1- 5 : forward-right  (FR)
 *     taps  6-10 : forward-left   (FL)
 *     taps 11-15 : reverse-right  (BR)
 *     taps 16-20 : reverse-left   (BL)
 *     tap  21    : summary, radius from arc
 *     tap  22    : summary, radius from chord
 *     tap  23    : summary, angle
 *     tap  24    : summary, braking-lead change
 *     tap  25    : start over
 * With one kind selected (CAL_TURN_FR .. CAL_TURN_BL in main.c), taps 1-5 run
 * that turn, tap 6 shows all four numbers for it on one page, tap 7 starts
 * over. Handy for re-checking one turn after changing its braking lead.
 *
 * An aborted turn (STALL / TIMEOUT / ABORT / BLOCKED) is not counted and the
 * same run is repeated on the next tap.
 *
 * Two radii per turn, both about the rear-axle centre (the point the planner
 * and calib.h use):
 *   arc   R = arc / theta
 *         arc   = mean of the two rear-wheel encoder paths
 *         theta = gyro heading change, INCLUDING the coast after the
 *                 controller stops steering short of target.
 *   chord R = chord / (2 sin(theta / 2))
 *         chord = straight-line distance the rear-axle centre moved, from
 *                 the encoder + gyro odometry in control.c.
 * On a perfect circular arc the two agree. They part when the curvature is
 * not constant, which on a 30 degree turn mostly means the coast: the brake
 * goes on and the wheels centre while the car is still moving. The chord
 * radius is the one that says where the car ends up, so it is the one to
 * compare with TURN_RADIUS_*_MM in calib.h, which the planner reads.
 *
 * The lead page is how far each turn type overshoots (+) or undershoots (-)
 * TT_ANGLE on average, coast included. Add it to that turn's braking lead in
 * control.c (MODE_TURN_DEG, the target_deg_total <= 45 block for 30 degree
 * turns; the 2.5 above it for 90): a turn that ends 1.2 deg past 30 needs its
 * lead 1.2 deg bigger. Settle the leads first, then measure the radii.
 *
 * Tape check: mark the floor under the rear-axle centre before and after a
 * turn and measure the chord c. Then R = c / (2 sin(angle / 2)), with the
 * angle off the OLED; for 30 degrees that is R = c / 0.518.
 *
 * The heading comes from the yaw the 100 Hz control ISR already integrates
 * (motion_yaw_deg()), not from reading the IMU here as well. Reading the IMU
 * from main while TIM6 reads it over the same I2C bus can collide
 * mid-transfer and silently return 0 deg/s.
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
/* Degrees per test turn. 30 is the planner's turn; 90 gives the old test. */
#define TT_ANGLE          30
#define TT_START_DELAY_MS 800u   /* after the tap, so your hand is off the car */
#define TT_SETTLE_MS      700u   /* keep integrating after the move: the coast */
#define TT_NCASES         4u
#define TT_DEG2RAD        (3.14159265f / 180.0f)

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

/* One turn's measurements. */
typedef struct {
    float   angle_deg;        /* signed so a good turn reads +TT_ANGLE        */
    float   radius_arc_mm;    /* arc / theta                                  */
    float   radius_chord_mm;  /* chord / (2 sin(theta / 2))                    */
    float   chord_mm;         /* straight-line move of the rear-axle centre   */
    float   fwd_mm;           /* along the start heading (negative reversing) */
    float   lat_mm;           /* to the left of the start heading             */
    int32_t dl, dr;           /* encoder counts, left and right               */
} tt_result_t;

/* TT_SUMMARY_* mean "the next tap shows that page". */
typedef enum { TT_RUNNING = 0, TT_SUMMARY_R, TT_SUMMARY_C, TT_SUMMARY_A, TT_SUMMARY_L,
               TT_SUMMARY_ONE, TT_DONE } tt_phase_t;
typedef enum { TT_PAGE_ARC = 0, TT_PAGE_CHORD, TT_PAGE_ANGLE, TT_PAGE_LEAD } tt_page_t;

static tt_phase_t tt_phase = TT_RUNNING;
static uint8_t    tt_only  = TT_ALL;   /* TT_ALL, or the one kind being run */
static uint8_t    tt_case  = 0;
static uint8_t    tt_rep   = 0;
static float      tt_radius      [TT_NCASES][TT_REPEATS];
static float      tt_radius_chord[TT_NCASES][TT_REPEATS];
static float      tt_angle       [TT_NCASES][TT_REPEATS];

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
 * @return the move result; the measurements are only meaningful on MOVE_DONE
 */
static move_result_t tt_run_one(const tt_case_t *c, tt_result_t *m) {
    int32_t l0   = enc_left_total;
    int32_t r0   = enc_right_total;
    float   yaw0 = motion_yaw_deg();
    float   x0, y0;
    motion_odometry_mm(&x0, &y0);

    move_turn_deg(c->left, c->forward, TT_ANGLE);
    move_result_t res = motion_result();

    /* The brake stays on, as after a real FL/FR/BL/BR. This used to free-wheel
       the motors here, which let the car roll while the steering swung back
       to centre: movement a real run does not have.
       The ISR keeps integrating yaw, encoders and odometry while idle, so
       waiting here captures the coast in the angle, the arc and the chord. */
    HAL_Delay(TT_SETTLE_MS);

    float x1, y1;
    motion_odometry_mm(&x1, &y1);
    m->dl = enc_left_total  - l0;
    m->dr = enc_right_total - r0;
    float yaw = motion_yaw_deg() - yaw0;

    /* Sign the angle the way the controller does, so a good turn reads
       +TT_ANGLE whichever of the four it is. */
    float s = (c->left ? 1.0f : -1.0f) * (c->forward ? 1.0f : -1.0f);
    m->angle_deg = yaw * s;

    float theta  = fabsf(yaw) * TT_DEG2RAD;
    float arc_mm = fabsf(((float)m->dl + (float)m->dr) * 0.5f) * MM_PER_COUNT;
    m->radius_arc_mm = (theta > 0.01f) ? (arc_mm / theta) : 0.0f;

    /* Displacement of the rear-axle centre, in the frame the turn started in. */
    float dx = x1 - x0;
    float dy = y1 - y0;
    float h0 = yaw0 * TT_DEG2RAD;
    m->chord_mm = sqrtf(dx * dx + dy * dy);
    m->fwd_mm   =  dx * cosf(h0) + dy * sinf(h0);
    m->lat_mm   = -dx * sinf(h0) + dy * cosf(h0);
    m->radius_chord_mm = (theta > 0.01f) ? (m->chord_mm / (2.0f * sinf(0.5f * theta))) : 0.0f;
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

static void tt_show_summary(tt_page_t page) {
    static const char *const TITLE[] = { "R arc mm   mean", "R chord mm mean", "ANGLE deg  mean", "LEAD change deg" };
    static const char *const FIELD[] = { "radius_mm", "radius_chord_mm", "angle_deg", "lead_change_deg" };
    static const char *const NEXT[]  = { "SW1 = chord R", "SW1 = angles", "SW1 = lead", "SW1 = restart" };
    char b[24];
    char u[80];

    OLED_Clear();
    tt_line(0, TITLE[page]);
    for (uint8_t ci = 0; ci < TT_NCASES; ci++) {
        const float *x = (page == TT_PAGE_ARC)   ? tt_radius[ci]
                       : (page == TT_PAGE_CHORD) ? tt_radius_chord[ci]
                       :                           tt_angle[ci];
        float mean, hr;
        tt_stats(x, TT_REPEATS, &mean, &hr);
        if (page == TT_PAGE_LEAD) {
            /* Overshoot past TT_ANGLE: add this to the turn's braking lead. */
            mean -= (float)TT_ANGLE;
            snprintf(b, sizeof b, "%s %+5.1f", TT_CASES[ci].name, (double)mean);
        } else if (page == TT_PAGE_ANGLE) {
            snprintf(b, sizeof b, "%s %5.1f +-%.1f", TT_CASES[ci].name,
                     (double)mean, (double)hr);
        } else {
            snprintf(b, sizeof b, "%s %4.0f +-%3.0f", TT_CASES[ci].name,
                     (double)mean, (double)hr);
        }
        tt_line((uint8_t)(10u + ci * 10u), b);

        snprintf(u, sizeof u, "[TT] SUMMARY %s %s mean=%.1f +-%.1f target_deg=%d\r\n",
                 TT_CASES[ci].name, FIELD[page], (double)mean, (double)hr, TT_ANGLE);
        command_send(u);
    }
    tt_line(50, NEXT[page]);
    OLED_Refresh_Gram();
}

/* Take the caller's choice of turns. A change (the first call) starts over. */
static void tt_select(uint8_t only) {
    if (only != TT_ALL && only >= TT_NCASES) only = TT_ALL;
    if (only == tt_only) return;
    tt_only  = only;
    tt_phase = TT_RUNNING;
    tt_rep   = 0;
    tt_case  = (only == TT_ALL) ? 0u : only;
}

static float tt_clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* One kind: everything about it on one page. */
static void tt_show_one_summary(uint8_t ci) {
    char b[17];   /* 16 characters is a full OLED line */
    char u[96];
    float ra, ra_hr, rc, rc_hr, an, an_hr;
    tt_stats(tt_radius[ci],       TT_REPEATS, &ra, &ra_hr);
    tt_stats(tt_radius_chord[ci], TT_REPEATS, &rc, &rc_hr);
    tt_stats(tt_angle[ci],        TT_REPEATS, &an, &an_hr);
    float lead = an - (float)TT_ANGLE;

    OLED_Clear();
    snprintf(b, sizeof b, "%s%03d summary", TT_CASES[ci].name, TT_ANGLE);
    tt_line(0, b);
    snprintf(b, sizeof b, "R arc  %4.0f +-%2.0f", (double)tt_clamp(ra, 0.0f, 9999.0f),
             (double)tt_clamp(ra_hr, 0.0f, 99.0f));
    tt_line(10, b);
    snprintf(b, sizeof b, "Rchord %4.0f +-%2.0f", (double)tt_clamp(rc, 0.0f, 9999.0f),
             (double)tt_clamp(rc_hr, 0.0f, 99.0f));
    tt_line(20, b);
    snprintf(b, sizeof b, "ang %5.1f +-%3.1f", (double)tt_clamp(an, -99.9f, 999.9f),
             (double)tt_clamp(an_hr, 0.0f, 9.9f));
    tt_line(30, b);
    snprintf(b, sizeof b, "lead %+4.1f deg", (double)tt_clamp(lead, -99.9f, 99.9f));
    tt_line(40, b);
    tt_line(50, "SW1 = restart");
    OLED_Refresh_Gram();

    snprintf(u, sizeof u, "[TT] SUMMARY %s radius_mm mean=%.1f +-%.1f target_deg=%d\r\n",
             TT_CASES[ci].name, (double)ra, (double)ra_hr, TT_ANGLE);
    command_send(u);
    snprintf(u, sizeof u, "[TT] SUMMARY %s radius_chord_mm mean=%.1f +-%.1f target_deg=%d\r\n",
             TT_CASES[ci].name, (double)rc, (double)rc_hr, TT_ANGLE);
    command_send(u);
    snprintf(u, sizeof u, "[TT] SUMMARY %s angle_deg mean=%.2f +-%.2f target_deg=%d\r\n",
             TT_CASES[ci].name, (double)an, (double)an_hr, TT_ANGLE);
    command_send(u);
    snprintf(u, sizeof u, "[TT] SUMMARY %s lead_change_deg mean=%+.2f target_deg=%d\r\n",
             TT_CASES[ci].name, (double)lead, TT_ANGLE);
    command_send(u);
}

void turn_test_show_idle(uint8_t only) {
    char b[24];

    tt_select(only);

    OLED_Clear();
    snprintf(b, sizeof b, "TURN TEST %d deg", TT_ANGLE);
    tt_line(0, b);
    snprintf(b, sizeof b, "next: %s %u/%u", TT_CASES[tt_case].name,
             (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
    tt_line(10, b);
    snprintf(b, sizeof b, "SW1 = turn %d", TT_ANGLE);
    tt_line(30, b);
    if (!calibrated) tt_line(40, "(gyro cal 1st)");
    tt_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

void turn_test_step(uint8_t only) {
    char b[24];
    char u[200];

    tt_select(only);

    /* --- summary pages --- */
    if (tt_phase == TT_SUMMARY_ONE) {
        tt_show_one_summary(tt_case);
        tt_phase = TT_DONE;
        return;
    }
    if (tt_phase == TT_SUMMARY_R) {
        tt_show_summary(TT_PAGE_ARC);
        tt_phase = TT_SUMMARY_C;
        return;
    }
    if (tt_phase == TT_SUMMARY_C) {
        tt_show_summary(TT_PAGE_CHORD);
        tt_phase = TT_SUMMARY_A;
        return;
    }
    if (tt_phase == TT_SUMMARY_A) {
        tt_show_summary(TT_PAGE_ANGLE);
        tt_phase = TT_SUMMARY_L;
        return;
    }
    if (tt_phase == TT_SUMMARY_L) {
        tt_show_summary(TT_PAGE_LEAD);
        tt_phase = TT_DONE;
        return;
    }
    if (tt_phase == TT_DONE) {
        tt_phase = TT_RUNNING;
        tt_case  = (tt_only == TT_ALL) ? 0u : tt_only;
        tt_rep   = 0;
        command_send("[TT] restart\r\n");
        turn_test_show_idle(tt_only);
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

    tt_result_t m;
    move_result_t res = tt_run_one(c, &m);

    snprintf(u, sizeof u,
             "[TT] %s run=%u result=%s radius_mm=%.1f radius_chord_mm=%.1f angle_deg=%.2f"
             " chord_mm=%.1f fwd_mm=%.1f lat_mm=%.1f encL=%ld encR=%ld target_deg=%d\r\n",
             c->name, (unsigned)(tt_rep + 1u), tt_result_name(res),
             (double)m.radius_arc_mm, (double)m.radius_chord_mm, (double)m.angle_deg,
             (double)m.chord_mm, (double)m.fwd_mm, (double)m.lat_mm,
             (long)m.dl, (long)m.dr, TT_ANGLE);
    command_send(u);

    OLED_Clear();
    snprintf(b, sizeof b, "%s run %u/%u", c->name,
             (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
    tt_line(0, b);
    snprintf(b, sizeof b, "R arc   %.0f mm", (double)m.radius_arc_mm);
    tt_line(10, b);
    snprintf(b, sizeof b, "R chord %.0f mm", (double)m.radius_chord_mm);
    tt_line(20, b);
    snprintf(b, sizeof b, "ang %.1f deg", (double)m.angle_deg);
    tt_line(30, b);

    if (res != MOVE_DONE) {
        /* Not counted: position is unknown, so the numbers mean nothing. */
        snprintf(b, sizeof b, "!! %s", tt_result_name(res));
        tt_line(40, b);
        tt_line(50, "SW1 = retry");
        OLED_Refresh_Gram();
        return;
    }

    tt_radius      [tt_case][tt_rep] = m.radius_arc_mm;
    tt_radius_chord[tt_case][tt_rep] = m.radius_chord_mm;
    tt_angle       [tt_case][tt_rep] = m.angle_deg;

    /* advance */
    uint8_t finished = 0u;
    tt_rep++;
    if (tt_rep >= TT_REPEATS) {
        tt_rep = 0;
        if (tt_only == TT_ALL) {
            tt_case++;
            finished = (tt_case >= TT_NCASES);
        } else {
            finished = 1u;     /* one kind: tt_case stays on it for the summary */
        }
    }

    if (finished) {
        tt_line(50, "SW1 = summary");
        tt_phase = (tt_only == TT_ALL) ? TT_SUMMARY_R : TT_SUMMARY_ONE;
        /* summary is drawn on the next tap */
    } else {
        snprintf(b, sizeof b, "next: %s %u/%u", TT_CASES[tt_case].name,
                 (unsigned)(tt_rep + 1u), (unsigned)TT_REPEATS);
        tt_line(40, b);
        tt_line(50, "SW1 = go");
    }
    OLED_Refresh_Gram();
}
