/*
 * control.c
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#include "main.h"          /* for __disable_irq / __get_PRIMASK */
#include "control.h"
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "calib.h"
#include <stdlib.h>


#define KFF            480.0f   /* feed-forward: duty per (count/tick)      */
#define KP_SPEED        16.0f
#define KI_SPEED         1.6f
#define KS_SYNC         24.0f   /* straight-line wheel sync                 */
#define I_DUTY_LIMIT  3200.0f   /* cap on the integral term, in duty units  */

/* Max duty change per 10 ms tick. Ramping instead of stepping is what keeps
   the AT8236 out of current limit when a wheel starts from rest. 600/tick
   reaches full duty in ~280 ms. */
#define SLEW_PER_TICK   600

typedef enum { M_IDLE = 0, M_SETTLE, M_RUN } mstate_t;

static volatile mstate_t state = M_IDLE;
static int32_t  target_counts;
static int32_t  target_delta;
static float    i_l, i_r;
static int32_t  out_l, out_r;      /* last duty actually sent, for slewing */
static uint8_t  use_sync;
static uint16_t settle;
static uint16_t stall;
static uint16_t elapsed;

static int32_t slew(int32_t now, int32_t want)
{
    int32_t d = want - now;
    if (d >  SLEW_PER_TICK) d =  SLEW_PER_TICK;
    if (d < -SLEW_PER_TICK) d = -SLEW_PER_TICK;
    return now + d;
}

void control_init(void)
{
    state = M_IDLE;
    out_l = out_r = 0;
}

uint8_t motion_busy(void) { return (state != M_IDLE); }

void motion_stop(void)
{
    /* Called from both main context (STOP command) and ISR context
       (move complete), so the state change must be atomic. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    state = M_IDLE;
    out_l = out_r = 0;

    __set_PRIMASK(primask);

    motors_brake();
    servo_us(SERVO_CENTRE);
}

static void begin(int32_t counts, int32_t delta, uint8_t sync, uint16_t settle_ticks)
{
    /* Called from main context while control_tick() runs in the TIM6 ISR.
       encoders_reset() and the state variables must not be touched
       half-way through a tick. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    encoders_reset();
    i_l = i_r = 0.0f;
    out_l = out_r = 0;
    target_counts = labs(counts);
    target_delta  = delta;
    use_sync      = sync;
    settle        = settle_ticks;
    stall         = 0u;
    elapsed       = 0u;
    /* A zero-length move must not arm the state machine, or the distance
       check below would never see a reason to stop. */
    state = (target_counts > 0) ? M_SETTLE : M_IDLE;

    __set_PRIMASK(primask);
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
        motors_brake();
        if (settle == 0u) {
            encoders_reset();      /* discard anything from settling */
            state = M_RUN;
        } else {
            settle--;
        }
        return;
    }

    /* --- hard safety limits ------------------------------------------
       Without these, a jammed wheel leaves the PI loop winding up and
       holding full duty into a stalled motor indefinitely.            */
    if (++elapsed > MOVE_TIMEOUT_TICKS) {
        motion_stop();
        return;
    }

    if (labs(enc_left_delta)  < STALL_MIN_COUNTS &&
        labs(enc_right_delta) < STALL_MIN_COUNTS) {
        if (++stall > STALL_TICKS) {
            motion_stop();
            return;
        }
    } else {
        stall = 0u;
    }

    /* --- distance check --- */
    int32_t travelled = (enc_left_total + enc_right_total) / 2;
    if (labs(travelled) >= target_counts) {
        motion_stop();
        return;
    }

    /* --- per-wheel PI on speed ---
       The integral is accumulated already scaled by KI_SPEED, so the
       clamp below is directly in duty units and is easy to reason about. */
    float e_l = (float)(target_delta - enc_left_delta);
    float e_r = (float)(target_delta - enc_right_delta);

    i_l += KI_SPEED * e_l;
    i_r += KI_SPEED * e_r;
    if (i_l >  I_DUTY_LIMIT) i_l =  I_DUTY_LIMIT;
    if (i_l < -I_DUTY_LIMIT) i_l = -I_DUTY_LIMIT;
    if (i_r >  I_DUTY_LIMIT) i_r =  I_DUTY_LIMIT;
    if (i_r < -I_DUTY_LIMIT) i_r = -I_DUTY_LIMIT;

    float u_l = KFF * (float)target_delta + KP_SPEED * e_l + i_l;
    float u_r = KFF * (float)target_delta + KP_SPEED * e_r + i_r;

    /* --- sync term: only when going straight ---
       On an arc the wheels SHOULD travel different distances, so
       enabling this during a turn makes the car fight itself.     */
    if (use_sync) {
        float drift = (float)(enc_left_total - enc_right_total);
        u_l -= KS_SYNC * drift;
        u_r += KS_SYNC * drift;
    }

    /* --- slew limit: never step the bridge from rest to full duty --- */
    out_l = slew(out_l, (int32_t)u_l);
    out_r = slew(out_r, (int32_t)u_r);

    motor_left(out_l);
    motor_right(out_r);
}
