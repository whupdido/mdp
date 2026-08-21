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

/* --- gains ----------------------------------------------------------------
 * Scaled for PWM_MAX = 16799. The previous values were tuned against a 4199
 * scale, so every duty-producing gain here is 4x what it used to be. If you
 * change PWM_MAX again, scale KFF, KP_SPEED, KI_SPEED, KS_SYNC, I_DUTY_LIMIT
 * and SLEW_PER_TICK by the same factor.
 *
 * Sanity check: KFF * SPEED_STRAIGHT = 480 * 20 = 9600, which is 57 % of
 * PWM_MAX -- the same operating point as before.
 * ------------------------------------------------------------------------- */
#define KFF            480.0f   /* feed-forward: duty per (count/tick)      */
#define KP_SPEED        16.0f
#define KI_SPEED         1.6f
#define KS_SYNC         24.0f   /* straight-line wheel sync                 */
#define I_DUTY_LIMIT  3200.0f   /* cap on the integral term, in duty units  */

/* Max duty change per 10 ms tick. Ramping instead of stepping is what keeps
   the AT8236 out of current limit when a wheel starts from rest. 600/tick
   reaches full duty in ~280 ms. */
#define SLEW_PER_TICK   600

/* Deceleration profile. Without a taper the loop holds cruise speed right up
   to the final tick and then coasts through the remainder -- that was the
   ~21 % overshoot measured during distance calibration. */
#define RAMP_DOWN_COUNTS  600   /* ~80 mm of taper before the target   */
#define MIN_CRAWL           4   /* counts/tick floor, or it stalls out */

/* Differential for turns. Commanding both rear wheels the same speed on an
   arc is a locked axle: the tyres scrub and the car understeers no matter
   how much steering lock you add. Measured symptom -- turn rate FELL as
   lock increased (40 deg at +-400 us, 31 deg at +-500 us). */
#define TURN_BIAS       0.35f   /* inner wheel runs this much slower */

typedef enum { M_IDLE = 0, M_SETTLE, M_RUN } mstate_t;

static volatile mstate_t state = M_IDLE;
static volatile move_result_t last_result = MOVE_NONE;
static int32_t  target_counts;
static int32_t  target_delta;
static float    i_l, i_r;
static int32_t  out_l, out_r;      /* last duty actually sent, for slewing */
static uint8_t  use_sync;
static int8_t   turn_left;      /* 1 = left turn -> LEFT wheel is inner */
static uint16_t settle;
static uint16_t stall;
static uint16_t elapsed;

static int32_t slew(int32_t now, int32_t want)
{
    /* Asymmetric on purpose. The rate limit exists to keep inrush current
       out of the AT8236 when duty RISES; applying it on the way down just
       makes the car coast past its target (measured: ~300 counts of
       run-out). So a reduction in magnitude within the same sign is
       allowed to take effect immediately, while growth and any sign
       reversal -- which would plug a spinning motor -- stay limited. */
    if ((now >= 0 && want >= 0 && want <= now) ||
        (now <= 0 && want <= 0 && want >= now)) {
        return want;
    }

    int32_t d = want - now;
    if (d >  SLEW_PER_TICK) d =  SLEW_PER_TICK;
    if (d < -SLEW_PER_TICK) d = -SLEW_PER_TICK;
    return now + d;
}

void control_init(void)
{
    state = M_IDLE;
    last_result = MOVE_NONE;
    out_l = out_r = 0;
}

uint8_t motion_busy(void) { return (state != M_IDLE); }

move_result_t motion_result(void) { return last_result; }

/* Single exit point for every way a move can end, so the reason is always
   recorded. Called from ISR context (normal completion, stall, timeout) and
   from main context (STOP command), hence the critical section. */
static void finish(move_result_t reason)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    state = M_IDLE;
    out_l = out_r = 0;
    last_result = reason;

    __set_PRIMASK(primask);

    motors_brake();
    servo_us(SERVO_CENTRE);
}

void motion_stop(void) { finish(MOVE_ABORT); }

static void begin(int32_t counts, int32_t delta, uint8_t sync,
                  uint16_t settle_ticks, int8_t left)
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
    turn_left     = left;
    settle        = settle_ticks;
    stall         = 0u;
    elapsed       = 0u;
    /* A zero-length move must not arm the state machine, or the distance
       check below would never see a reason to stop. */
    state = (target_counts > 0) ? M_SETTLE : M_IDLE;
    if (state == M_IDLE) last_result = MOVE_DONE;   /* zero-length move */

    __set_PRIMASK(primask);
}

void move_straight_mm(int32_t mm)
{
    servo_us(SERVO_CENTRE);
    int32_t counts = (int32_t)((float)mm / MM_PER_COUNT);
    /* 20 ticks = 200 ms for the servo to reach centre before moving */
    begin(counts, (mm >= 0) ? SPEED_STRAIGHT : -SPEED_STRAIGHT, 1u, 20u, 0);
}

void move_turn(int8_t left, int8_t forward, int32_t counts)
{
    servo_us(left ? SERVO_LEFT : SERVO_RIGHT);
    /* 30 ticks = 300 ms: full lock takes longer than centring     */
    begin(counts, forward ? SPEED_TURN : -SPEED_TURN, 0u, 30u, left);
}

/* Scale the calibrated 90 deg constants linearly. Checklist A.4 asks for a
   supervisor-specified angle between 90 and 360 degrees, so 90 alone is not
   enough. Linear scaling assumes the radius is constant through the arc,
   which holds because the steering lock does not change mid-move -- but
   verify a 180 and a 360 on the floor before trusting it. */
void move_turn_deg(int8_t left, int8_t forward, int32_t degrees)
{
    if (degrees <= 0)  degrees = 90;    /* legacy FL000 etc. means 90 */
    if (degrees > 360) degrees = 360;

    int32_t base;
    if (forward) base = left ? TURN_COUNTS_FL : TURN_COUNTS_FR;
    else         base = left ? TURN_COUNTS_BL : TURN_COUNTS_BR;

    move_turn(left, forward, (base * degrees) / 90);
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
        finish(MOVE_TIMEOUT);
        return;
    }

    if (labs(enc_left_delta)  < STALL_MIN_COUNTS &&
        labs(enc_right_delta) < STALL_MIN_COUNTS) {
        if (++stall > STALL_TICKS) {
            finish(MOVE_STALL);
            return;
        }
    } else {
        stall = 0u;
    }

    /* --- distance check --- */
    int32_t travelled = (enc_left_total + enc_right_total) / 2;
    if (labs(travelled) >= target_counts) {
        finish(MOVE_DONE);
        return;
    }

    /* --- decelerate over the last stretch so we stop ON target --- */
    int32_t remaining = target_counts - labs(travelled);
    int32_t delta = target_delta;
    if (remaining < RAMP_DOWN_COUNTS) {
        int32_t scaled  = (target_delta * remaining) / RAMP_DOWN_COUNTS;
        int32_t floor_v = (target_delta >= 0) ? MIN_CRAWL : -MIN_CRAWL;
        /* Never let the commanded speed reach zero before the distance
           does, or the stall guard fires instead of the distance check. */
        if (labs(scaled) < MIN_CRAWL) scaled = floor_v;
        delta = scaled;
    }

    /* --- differential: the inner wheel travels a shorter arc ---
       use_sync is 0 exactly when turning, so it doubles as the flag. */
    int32_t delta_l = delta, delta_r = delta;
    if (!use_sync) {
        int32_t inner = (int32_t)((float)delta * (1.0f - TURN_BIAS));
        int32_t outer = (int32_t)((float)delta * (1.0f + TURN_BIAS));
        if (turn_left) { delta_l = inner; delta_r = outer; }
        else           { delta_l = outer; delta_r = inner; }
    }

    /* --- per-wheel PI on speed ---
       The integral is accumulated already scaled by KI_SPEED, so the
       clamp below is directly in duty units and is easy to reason about. */
    float e_l = (float)(delta_l - enc_left_delta);
    float e_r = (float)(delta_r - enc_right_delta);

    i_l += KI_SPEED * e_l;
    i_r += KI_SPEED * e_r;
    if (i_l >  I_DUTY_LIMIT) i_l =  I_DUTY_LIMIT;
    if (i_l < -I_DUTY_LIMIT) i_l = -I_DUTY_LIMIT;
    if (i_r >  I_DUTY_LIMIT) i_r =  I_DUTY_LIMIT;
    if (i_r < -I_DUTY_LIMIT) i_r = -I_DUTY_LIMIT;

    float u_l = KFF * (float)delta_l + KP_SPEED * e_l + i_l;
    float u_r = KFF * (float)delta_r + KP_SPEED * e_r + i_r;

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
