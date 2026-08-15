/*
 * control.c
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#include "control.h"
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "calib.h"
#include <stdlib.h>

/* --- gains: tune these, see Part 9.5 --- */
#define KFF        120.0f   /* feed-forward: duty per (count/tick) */
#define KP_SPEED     4.0f
#define KI_SPEED     0.4f
#define KS_SYNC      6.0f   /* straight-line wheel sync */
#define I_LIMIT   2000.0f

typedef enum { M_IDLE = 0, M_SETTLE, M_RUN } mstate_t;

static volatile mstate_t state = M_IDLE;
static int32_t  target_counts;
static int32_t  target_delta;
static float    i_l, i_r;
static uint8_t  use_sync;
static uint16_t settle;

void control_init(void) { state = M_IDLE; }

uint8_t motion_busy(void) { return (state != M_IDLE); }

void motion_stop(void)
{
    state = M_IDLE;
    motors_coast();
    servo_us(SERVO_CENTRE);
}

static void begin(int32_t counts, int32_t delta, uint8_t sync, uint16_t settle_ticks)
{
    encoders_reset();
    i_l = i_r = 0.0f;
    target_counts = labs(counts);
    target_delta  = delta;
    use_sync      = sync;
    settle        = settle_ticks;
    state         = M_SETTLE;
}

void move_straight_mm(int32_t mm)
{
    servo_us(SERVO_CENTRE);
    int32_t counts = (int32_t)((float)mm / MM_PER_COUNT);
    /* 20 ticks = 200 ms for the servo to reach centre before moving */
    begin(counts, (mm >= 0) ? SPEED_STRAIGHT : -SPEED_STRAIGHT, 1u, 20u);
}

void move_turn(int8_t left, int8_t forward, int32_t counts)
{
    servo_us(left ? SERVO_LEFT : SERVO_RIGHT);
    /* 30 ticks = 300 ms: full lock takes longer than centring     */
    begin(counts, forward ? SPEED_TURN : -SPEED_TURN, 0u, 30u);
}

void control_tick(void)
{
    encoders_sample();

    if (state == M_IDLE) return;

    if (state == M_SETTLE) {
        motors_coast();
        if (settle-- == 0u) {
            encoders_reset();      /* discard anything from settling */
            state = M_RUN;
        }
        return;
    }

    /* --- distance check --- */
    int32_t travelled = (enc_left_total + enc_right_total) / 2;
    if (labs(travelled) >= target_counts) {
        motion_stop();
        return;
    }

    /* --- per-wheel PI on speed --- */
    float e_l = (float)(target_delta - enc_left_delta);
    float e_r = (float)(target_delta - enc_right_delta);

    i_l += e_l;  i_r += e_r;
    if (i_l >  I_LIMIT) i_l =  I_LIMIT;
    if (i_l < -I_LIMIT) i_l = -I_LIMIT;
    if (i_r >  I_LIMIT) i_r =  I_LIMIT;
    if (i_r < -I_LIMIT) i_r = -I_LIMIT;

    float u_l = KFF * (float)target_delta + KP_SPEED * e_l + KI_SPEED * i_l;
    float u_r = KFF * (float)target_delta + KP_SPEED * e_r + KI_SPEED * i_r;

    /* --- sync term: only when going straight ---
       On an arc the wheels SHOULD travel different distances, so
       enabling this during a turn makes the car fight itself.     */
    if (use_sync) {
        float drift = (float)(enc_left_total - enc_right_total);
        u_l -= KS_SYNC * drift;
        u_r += KS_SYNC * drift;
    }

    motor_left((int32_t)u_l);
    motor_right((int32_t)u_r);
}
