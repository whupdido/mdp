/*
 * control.c -- closed-loop motion controller with IMU heading & distance feedback
 */

#include "control.h"
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "sensors.h"
#include "calib.h"
#include "icm20948.h"
#include "command.h"
#include <math.h>
#include <stdlib.h>

/* --- Motion Mode State --- */
typedef enum {
    MODE_IDLE = 0,
    MODE_STRAIGHT,
    MODE_TURN_DEG,
    MODE_TURN_RAW,
	MODE_PIVOT_DEG
} control_mode_t;

static volatile control_mode_t current_mode = MODE_IDLE;
static volatile move_result_t  last_result  = MOVE_NONE;
static volatile uint8_t        busy_flag    = 0;
static volatile uint8_t        abort_latched = 0;
static float current_speed_ramp = 0.0f;
/* Kush: set once a straight move starts slowing for its target; see
   MODE_STRAIGHT in control_tick(). */
static volatile uint8_t straight_braking = 0;
/* Kush: rear-axle odometry for the turn test; see motion_odometry_mm(). */
static volatile float odo_x_mm = 0.0f;
static volatile float odo_y_mm = 0.0f;
static float pivot_speed_ramp = 0.0f;
/* Track cumulative signed ticks during in-place pivots */
static volatile int32_t pivot_enc_l_accum = 0;
static volatile int32_t pivot_enc_r_accum = 0;

/* Motion Targets and Accumulators */
static volatile int32_t  target_counts_total = 0;
static volatile int32_t  accum_counts        = 0;
static volatile float    target_deg_total    = 0.0f;
static volatile float    accum_deg           = 0.0f;
static volatile int8_t   dir_forward         = 1;
static volatile int8_t   turn_left           = 0;
/* Relative encoder accumulators for straight driving */
static volatile int32_t enc_left_straight_accum  = 0;
static volatile int32_t enc_right_straight_accum = 0;

/* Active Straight-Line Heading Lock Reference */
static volatile float    locked_heading_deg  = 0.0f;
static volatile float    global_yaw_deg      = 0.0f;

/* Safety & Diagnostics */
static volatile uint32_t move_ticks          = 0;
static volatile uint32_t stall_ticks_count   = 0;

/* Speed PID States (tracks counts per 10ms tick) */
static float left_pid_integral  = 0.0f;
static float right_pid_integral = 0.0f;

/* PID Gains for speed control (MG513 motors) */
#define SPEED_KP    120.0f
#define SPEED_KI    15.0f

/* Kush (08-Oct): straight-line steering.
 *   1 = damping from the gyro's yaw rate, and both gains scaled by
 *       STEER_REF_SPEED / measured wheel speed, so the steering corrects
 *       equally hard at any speed.
 *   0 = the code before 08-Oct: P on heading, D on the rear wheel speed
 *       difference (which the speed loops hold at zero, so it damped almost
 *       nothing: two swings after a push), all scaled down with the
 *       commanded speed (a quarter in the crawl, so any heading error left
 *       when the car started slowing was still there when it stopped).
 * The damping uses a smoothed gyro rate (STEER_V2_RATE_SMOOTH): the gyro's
 * own filter is at its ~200 Hz reset default, so each reading carries motor
 * and gear vibration, and fed straight in it kept the servo twitching. Only
 * the steering's copy is smoothed; the heading and the turns read the gyro
 * exactly as before. KP is eased from 130 to win back the smoothing's delay.
 * Tune on the floor: push the car sideways mid-move.
 *   Swings back past the line more than once: raise STEER_V2_KD a little or
 *   lower STEER_V2_KP.  Servo still twitchy on a plain straight: lower
 *   STEER_V2_RATE_SMOOTH a little (more smoothing) and STEER_V2_KP with it.
 * Check with CAL_TRACE (hdg per phase) and CAL_STRAIGHT. */
#define STRAIGHT_STEER_V2   1
#define STEER_V2_KP         110.0f  /* us per deg of heading error (was 130)         */
#define STEER_V2_KD         10.0f   /* us per deg/s of yaw rate, from the gyro       */
#define STEER_V2_RATE_SMOOTH 0.35f  /* share of each new gyro reading in that rate   */
#define STEER_REF_SPEED     27.0f   /* counts/tick the gains hold at: the cruise
                                       speed CAL_TRACE measured on 08-Oct            */
#define STEER_SCALE_MIN     0.4f    /* gains never cut below this (above ~67/tick)  */
#define STEER_SCALE_MAX     2.0f    /* or KP boosted above this (below ~13/tick);
                                       the damping is never boosted, only cut       */
static float steer_speed_f = 0.0f;  /* smoothed wheel speed for that, counts/tick   */
#if STRAIGHT_STEER_V2
static float gz_smooth     = 0.0f;  /* smoothed gyro rate for the damping, deg/s    */
#endif

/* Encoder counts of crawl kept before a straight move's target, so the
   wheels have settled at crawl speed when the brake goes on (~14 mm). */
#define BRAKE_MARGIN_COUNTS 100
float steer_integral = 0.0f;

/* Accelerates/decelerates the commanded speed by this much per tick. */
#define STRAIGHT_RAMP_STEP  1.2f

/* Kush (09-Oct): where a straight move starts slowing, worked out from the
 * speed the wheels are really doing (profile.brake_measured = 1).
 *
 * The old formula, (cmd^2 - crawl^2) / (2 * RAMP_STEP), is the distance the
 * COMMAND covers while it ramps down. The wheels only do ~45 % of the command
 * (27 counts/tick measured at a command of 60), so they cover about 45 % of
 * that, and the car then crawled the rest: ~10 cm, 0.6-2.4 s, at the end of
 * every move. From a cruise of 80 it was worse (~20 cm, up to ~3.8 s), which
 * is why 0c37bfe raised the crawl to 50 -- but that makes the car stop from
 * ~20 counts/tick instead of ~6, ~5-10 mm long.
 *
 * Here the ramp-down distance comes from the speed the wheels really do and
 * how fast they really follow the command down (ramp_down_counts), plus their
 * lag behind the command and a safety factor. The crawl speed, the crawl
 * margin and BRAKE_LEAD_COUNTS are untouched, so the car still reaches the
 * same crawl and stops the same way. This file's control_tick() was run on
 * the host against a simulated car (motor fitted to 27/tick at 60), for time
 * constants of 50-150 ms and deadbands of 600-1500 duty, with and without the
 * Task 2 extra feedforward: the speed at the brake stayed inside Task 1's own
 * range (3-11 counts/tick) and the stop error within a few mm of it, with
 * the crawl cut from ~10 cm to a few cm.
 * To check on the floor: CAL_TRACE page 2 (crawl length) and CAL_STRAIGHT,
 * with CAL_USE_TASK2_PROFILE = 1 in main.c. Crawl too short or stops long:
 * raise BRAKE_SAFETY (1.2-1.4) or BRAKE_LAG_TICKS.                          */
#ifndef BRAKE_SAFETY
#define BRAKE_SAFETY        1.1f   /* x the ideal ramp-down distance          */
#endif
#ifndef BRAKE_LAG_TICKS
#define BRAKE_LAG_TICKS     6.0f   /* wheels lag the command by about this    */
#endif
#define BRAKE_TO_ACTUAL     0.25f  /* wheel speed at the command being ramped
                                      to, as a share of it (~0.25-0.4 at the
                                      crawl); low is the safe side           */
/* Motor duty per count/tick of wheel speed: full PWM (16799) gives ~82
   counts/tick on the floor (testMaxSpeed note in calib.h). */
#define MOTOR_DUTY_PER_COUNT 193.0f

/* --- Speed profiles (control.h) --- */
const motion_profile_t MOTION_PROFILE_TASK1 = {
    .straight_cruise   = (float)SPEED_STRAIGHT,
    .straight_crawl    = (float)SPEED_CRAWL,
    .brake_measured    = 0u,
    .straight_ff_extra = 0.0f,
};
/* Kush (11-Oct, speed step 5): brake_measured 0 -> 1 with the cruise raised to
   80 (calib.h): from 80 the commanded-speed brake point would leave ~20 cm of
   crawl at the end of every move. Same crawl (15) and brake lead as before. */
const motion_profile_t MOTION_PROFILE_TASK2 = {
    .straight_cruise   = (float)T2_SPEED_STRAIGHT,
    .straight_crawl    = (float)T2_SPEED_CRAWL,
    .brake_measured    = 1u,
    .straight_ff_extra = (float)T2_STRAIGHT_FF_EXTRA,
};
static const motion_profile_t *volatile profile = &MOTION_PROFILE_TASK1;

/* Smoothed measured wheel speed during a straight move, counts/tick. */
static float straight_v_meas = 0.0f;

/* --- Sensor watch for move_straight_watch() (control.h) --- */
typedef struct {
    uint8_t  active;
    uint8_t  kind;
    uint8_t  side;
    uint8_t  log_face;
    uint8_t  armed;
    uint8_t  triggered;
    uint8_t  seen;
    uint8_t  seen_ticks;
    uint8_t  cond_ticks;
    uint8_t  slowed;        /* cruise has dropped to sense_speed (latched) */
    float    thresh_cm;
    float    sense_speed;
    float    ff_fade_from;  /* command when it slowed: the Task 2 extra
                               feedforward fades out from here            */
    int32_t  arm_counts;    /* in accum_counts units, like the move's target   */
    int32_t  after_counts;
    int32_t  l0, r0;        /* encoder totals at the start of the move         */
    int32_t  seen_mm;       /* positions reported back are encoder distance:   */
    int32_t  trigger_mm;    /*  accum_counts halves (L+R) in integers and runs */
    float    trigger_speed; /*  ~1 % short, driven_mm does not                  */
} watch_rt_t;
static volatile watch_rt_t watch_rt;

/* Face log for watch_face_fit(): one sample per FACE_LOG_STEP_MM driven. */
#define FACE_LOG_MAX        256u
#define FACE_LOG_STEP_MM    4
static volatile int16_t  face_log_mm[FACE_LOG_MAX];
static volatile float    face_log_cm[FACE_LOG_MAX];
static volatile uint16_t face_log_n = 0u;
static volatile int32_t  face_log_last_mm = 0;

/* Kush: see control.h. Only CAL_TRACE sets it, and only for one move. */
void (*volatile straight_tick_hook)(const straight_tick_t *t) = 0;
void (*volatile control_tick_hook)(void) = 0;

/* ------------------------------------------------------------------------- */
/* Helper Functions                                                          */
/* ------------------------------------------------------------------------- */

static void reset_speed_pid(void)
{
    left_pid_integral  = 0.0f;
    right_pid_integral = 0.0f;
    current_speed_ramp = 0.0f;
}

static void stop_hardware(move_result_t result)
{
    motor_left(0);
    motor_right(0);
    servo_us(SERVO_CENTRE);

    current_mode = MODE_IDLE;
    last_result  = result;
    busy_flag    = 0;
    reset_speed_pid();
}

/* Kush (09-Oct): starts the move the caller has set up, unless a stop came in
 * since the caller's own check at the top. Every move checks for an abort,
 * then waits (servo to full lock, or to centre) before it starts. A
 * motion_stop() from an interrupt inside that wait ran stop_hardware() while
 * no move was running, which centred the wheels, and the move then started
 * anyway. A STOP from the Pi got away with it: command_poll() handles it a
 * second time from inside the move's loop and stops the move there. An SW1
 * stop (CAL_TASK2, control_tick_hook) has no second chance: a turn then drove
 * straight on centred wheels, never reached its angle, and ran on to the
 * 20 s timeout (simulated: about one tap in six). Interrupts are off between
 * the check and the start, so a stop lands either before (no move, verdict
 * ABORT) or after (stopped as usual). Returns 1 if the move started. */
static uint8_t start_move(control_mode_t mode)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t go = abort_latched ? 0u : 1u;
    if (go) {
        busy_flag    = 1;
        current_mode = mode;
    }
    __set_PRIMASK(primask);
    if (!go) stop_hardware(MOVE_ABORT);    /* wheels back to centre */
    return go;
}

/* 1 while the Task 2 extra feedforward is fully in (see MODE_STRAIGHT). */
static uint8_t straight_ff_extra_on(void)
{
    return (profile->straight_ff_extra > 0.0f && dir_forward == 1 &&
            !(watch_rt.active && watch_rt.slowed)) ? 1u : 0u;
}

/* Counts a straight move covers while its command ramps from `from` down to
   `to` (both commanded counts/tick), given the wheels are really doing
   v_meas. See BRAKE_SAFETY at the top.
   The wheel speed settles on a straight line against the command,
       v = v_to + b * (cmd - to),
   whose slope follows from the controller's own numbers: duty is
   (20 + KP) * cmd + extra * (cmd - crawl) - KP * v, and the motors need about
   MOTOR_DUTY_PER_COUNT per count/tick, so b = (20 + KP + extra) / (193 + KP):
   0.45 on the plain feedforward (matches 27 counts/tick at a command of 60)
   and 1.25 with Task 2's extra. While the command falls at STRAIGHT_RAMP_STEP
   per tick the wheels slow at b times that. The speed used is the larger of
   the measured one and the one the wheels are heading for at this command, so
   a short move that starts braking while the wheels are still catching up
   allows for the speed they are about to reach. */
static int32_t ramp_down_counts(float from, float to, float v_meas)
{
    const float extra = straight_ff_extra_on() ? profile->straight_ff_extra : 0.0f;
    const float b     = (20.0f + SPEED_KP + extra) / (MOTOR_DUTY_PER_COUNT + SPEED_KP);
    const float v_to  = BRAKE_TO_ACTUAL * to;
    float v = v_to + b * (from - to);          /* where the wheels are heading */
    if (v_meas > v) v = v_meas;
    const float lag = BRAKE_LAG_TICKS * v;
    if (from <= to + 1.0f) return (int32_t)lag;
    float dv2 = v * v - v_to * v_to;
    if (dv2 < 0.0f) dv2 = 0.0f;
    return (int32_t)(BRAKE_SAFETY * dv2 / (2.0f * b * STRAIGHT_RAMP_STEP) + lag);
}

/* Counts a straight move needs, from now, to be down to a watch's sense speed
   by its arm point (the slow-down latch in MODE_STRAIGHT). Without the Task 2
   extra feedforward this is ramp_down_counts(), as before. With it, the car
   can still be accelerating when the latch is due (a watch that arms after a
   short run), with the wheels already well above the speed they will do at
   the sense command, because the extra fades to nothing there. So both speeds
   are worked out from the crawl up, on the same straight lines as above:
   heading for v_c + b * (cmd - crawl) now, slowing to v_c + b0 * (sense -
   crawl), b0 being the slope without the extra. */
static int32_t sense_slow_counts(float from, float sense, float v_meas)
{
    if (!straight_ff_extra_on()) return ramp_down_counts(from, sense, v_meas);
    const float crawl = profile->straight_crawl;
    const float b0    = (20.0f + SPEED_KP) / (MOTOR_DUTY_PER_COUNT + SPEED_KP);
    const float b     = (20.0f + SPEED_KP + profile->straight_ff_extra) /
                        (MOTOR_DUTY_PER_COUNT + SPEED_KP);
    const float v_c   = BRAKE_TO_ACTUAL * crawl;
    const float v_to  = v_c + b0 * ((sense > crawl) ? (sense - crawl) : 0.0f);
    float v = v_c + b * ((from > crawl) ? (from - crawl) : 0.0f);
    if (v_meas > v) v = v_meas;
    const float lag = BRAKE_LAG_TICKS * v;
    if (v <= v_to) return (int32_t)lag;
    return (int32_t)(BRAKE_SAFETY * (v * v - v_to * v_to) / (2.0f * b * STRAIGHT_RAMP_STEP) + lag);
}

/* Counts a straight move needs, from now, to come down to its crawl and stop
   the normal way. The Task 1 profile keeps the original formula. */
static int32_t straight_stop_counts(void)
{
    const float sp    = fabsf(current_speed_ramp);
    /* Kush (09-Oct): a move_straight_watch() keeps the crawl (SPEED_CRAWL) and
       the measured brake point its stops were tuned with, whatever the
       profile. Every other straight follows the profile. */
    if (watch_rt.active) {
        return ramp_down_counts(sp, (float)SPEED_CRAWL, straight_v_meas) + BRAKE_MARGIN_COUNTS;
    }
    const float crawl = profile->straight_crawl;
    if (profile->brake_measured) {
        return ramp_down_counts(sp, crawl, straight_v_meas) + BRAKE_MARGIN_COUNTS;
    }
    return (int32_t)((sp * sp - crawl * crawl) / (2.0f * STRAIGHT_RAMP_STEP)) + BRAKE_MARGIN_COUNTS;
}

/* One tick of a move_straight_watch() watch, from MODE_STRAIGHT once
   accum_counts is up to date. May pull target_counts_total in. */
static void straight_watch_tick(void)
{
    if (!watch_rt.active || watch_rt.triggered) return;

    const int32_t pos = accum_counts;
    if (!watch_rt.armed) {
        if (pos < watch_rt.arm_counts) return;
        watch_rt.armed = 1u;
    }

    float v;
    if (watch_rt.kind == WATCH_US_BELOW) {
        if (ultrasonic_age_ms() > 100u) {      /* no fresh echo: no decision */
            watch_rt.cond_ticks = 0u;
            return;
        }
        v = ultrasonic_distance_cm;
    } else {
        v = ir_filtered_cm(watch_rt.side);
    }

    /* Distance driven so far by the encoders (what driven_mm is measured in). */
    const int32_t mm = (int32_t)(0.5f * (float)((enc_left_total - watch_rt.l0) +
                                                (enc_right_total - watch_rt.r0)) * MM_PER_COUNT);

    /* Face log: while the IR sees something, one sample per FACE_LOG_STEP_MM. */
    if (watch_rt.log_face && watch_rt.kind != WATCH_US_BELOW && v < watch_rt.thresh_cm) {
        if (face_log_n < FACE_LOG_MAX &&
            (face_log_n == 0u || (mm - face_log_last_mm) >= FACE_LOG_STEP_MM)) {
            face_log_mm[face_log_n] = (int16_t)mm;
            face_log_cm[face_log_n] = v;
            face_log_n++;
            face_log_last_mm = mm;
        }
    }

    uint8_t cond = 0u;
    switch (watch_rt.kind) {
        case WATCH_IR_ABOVE: cond = (v >= watch_rt.thresh_cm); break;
        case WATCH_IR_BELOW:
        case WATCH_US_BELOW: cond = (v <= watch_rt.thresh_cm); break;
        case WATCH_IR_PASS:
            if (!watch_rt.seen) {
                if (v <= watch_rt.thresh_cm) {
                    if (++watch_rt.seen_ticks >= WATCH_CONFIRM_TICKS) {
                        watch_rt.seen    = 1u;
                        watch_rt.seen_mm = mm;
                    }
                } else {
                    watch_rt.seen_ticks = 0u;
                }
            } else {
                cond = (v >= watch_rt.thresh_cm);
            }
            break;
        default:
            break;
    }

    if (!cond) { watch_rt.cond_ticks = 0u; return; }
    if (++watch_rt.cond_ticks < WATCH_CONFIRM_TICKS) return;

    /* Confirmed. Stop after_counts from here -- or further, if the car needs
       more room than that to slow to its crawl, so it always stops the
       normal way instead of braking hard from speed. */
    watch_rt.triggered      = 1u;
    watch_rt.trigger_mm     = mm;
    watch_rt.trigger_speed  = straight_v_meas;
    int32_t room = straight_stop_counts();
    if (room < watch_rt.after_counts) room = watch_rt.after_counts;
    if (pos + room < target_counts_total) target_counts_total = pos + room;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void motion_set_profile(const motion_profile_t *p)
{
    if (p != 0) profile = p;
}

const motion_profile_t *motion_get_profile(void)
{
    return profile;
}

void control_init(void)
{
    current_mode = MODE_IDLE;
    last_result  = MOVE_NONE;
    busy_flag    = 0;
    abort_latched = 0u;
    global_yaw_deg = 0.0f;
    /* Ensure steering is centered while idle at startup */
	servo_us(SERVO_CENTRE);
	motor_left(0);
	motor_right(0);
    //stop_hardware(MOVE_NONE);
}

uint8_t motion_busy(void)
{
    return busy_flag;
}

void motion_stop(void)
{
    abort_latched = 1u;
    stop_hardware(MOVE_ABORT);
}

uint8_t motion_abort_requested(void)
{
    return abort_latched;
}

void motion_abort_clear(void)
{
    abort_latched = 0u;
}

move_result_t motion_result(void)
{
    return last_result;
}

/* Heading the 100 Hz ISR integrates continuously, in degrees, CCW positive.
   Used by turn_test.c so it does not have to read the IMU itself (which would
   share the I2C bus with the ISR). A 32-bit float read is atomic on the M4. */
float motion_yaw_deg(void)
{
    return global_yaw_deg;
}

void motion_odometry_mm(float *x_mm, float *y_mm)
{
    /* Both coordinates from the same tick: the ISR must not run in between. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *x_mm = odo_x_mm;
    *y_mm = odo_y_mm;
    __set_PRIMASK(primask);
}

/* ------------------------------------------------------------------------- */
/* High-Level Motion Commands (Blocking)                                    */
/* ------------------------------------------------------------------------- */

uint8_t move_straight_mm(int32_t mm)
{
    if (motion_abort_requested()) return 0;
    if (mm == 0) { last_result = MOVE_DONE; return 1; } /* Zhenxi: see report_result() in command.c */

    /* Lock wheels to calibrated center at launch (plus the reverse trim when
       backing up, so the wheels are already there as the car starts) */
	servo_us((uint16_t)(SERVO_CENTRE + ((mm < 0) ? SERVO_REVERSE_TRIM_US : 0)));
	HAL_Delay(150);

    target_counts_total       = (int32_t)(fabsf((float)mm) / MM_PER_COUNT);
    dir_forward               = (mm > 0) ? 1 : -1;
    accum_counts              = 0;
    enc_left_straight_accum   = 0;
    enc_right_straight_accum  = 0;
    steer_integral            = 0.0f;
    steer_speed_f             = 0.0f;
    left_pid_integral         = 0.0f;
	right_pid_integral        = 0.0f;
	current_speed_ramp        = 0.0f;
	straight_braking          = 0u;
	straight_v_meas           = 0.0f;
    move_ticks                = 0;
    stall_ticks_count         = 0;
    locked_heading_deg        = global_yaw_deg;
    last_result               = MOVE_NONE; /* Zhenxi: this move has no verdict yet */

    reset_speed_pid();
    if (!start_move(MODE_STRAIGHT)) return 0;   /* stopped during the wait above */

    /* Monitor sensors while moving */
	while (busy_flag) {
		/* Zhenxi: keep reading the UART while the move runs.
		 *
		 * This loop blocks dispatch(), and dispatch() is the only place a
		 * STOP is acted on -- so a STOP from the tablet sat in the UART
		 * buffer until the move finished by itself. obstacle_nav.c already
		 * polls inside its own wait loops for exactly this reason; the two
		 * moves the tablet can drive need the same. A STOP now lands in
		 * motion_stop() within one loop pass, which drops busy_flag and
		 * ends this loop. Any other command arriving mid-move gets BUSY
		 * from dispatch(), as before.                                    */
		command_poll();

		/* Only check for front collisions if we are driving forward */
		if (dir_forward == 1 && check_front_collision()) {
			/* Zhenxi: was MOVE_DONE, which made a 15 cm move that stopped at
			 * 3 cm indistinguishable from one that completed. The [WARN]
			 * line below still goes out; BLOCKED is what the Pi and tablet
			 * key off.                                                    */
			stop_hardware(MOVE_BLOCKED);
			stop_hardware(MOVE_DONE);
			busy_flag = 0;

			/* GYRO FIX: Wait for chassis mechanical vibrations to stop
			 * before returning control, preventing phantom IMU spikes! */
			HAL_Delay(300);

			command_send("\r\n[WARN] COLLISION AVOIDED! Stopping early.\r\n");
			return 0; /* Return 0 = Aborted */
		}
		HAL_Delay(5);
	}

    /* Zhenxi: only claim DONE if nothing else already claimed the result.
     *
     * Every way out of the loop above has already been through
     * stop_hardware(): the ISR on DONE / STALL / TIMEOUT, motion_stop() on
     * a STOP. Calling stop_hardware(MOVE_DONE) unconditionally here then
     * overwrote that verdict -- a stall became DONE before command.c could
     * report it, and the tablet's "position no longer trustworthy" warning
     * could never fire. The hardware is already stopped; the settle delay
     * is all that is still needed.                                       */
    if (last_result == MOVE_NONE) stop_hardware(MOVE_DONE);
    /* Final settle. Kush (12-Oct, STRAIGHT_SETTLE_FAST in calib.h): 50 ms
       instead of 100 after a straight; it stops from its crawl, so there is
       next to no coast left. Turns keep their 100 ms (move_turn_deg). */
    HAL_Delay(STRAIGHT_SETTLE_FAST ? 50u : 100u);
	return 1;
}

/* Signed distance the rear wheels have covered since (l0, r0), mm. */
static int32_t wheels_mm_since(int32_t l0, int32_t r0)
{
    return (int32_t)(0.5f * (float)((enc_left_total - l0) + (enc_right_total - r0)) * MM_PER_COUNT);
}

int32_t move_straight_measured(int32_t mm)
{
    const int32_t l0 = enc_left_total, r0 = enc_right_total;
    move_straight_mm(mm);
    return wheels_mm_since(l0, r0);   /* after move_straight_mm's settle: coast included */
}

uint8_t move_straight_watch(int32_t max_mm, watch_t *w)
{
    w->triggered = 0u;
    w->seen      = 0u;
    w->seen_mm   = 0;
    w->trigger_mm = 0;
    w->trigger_speed_mm_tick = 0.0f;
    w->driven_mm = 0;
    if (max_mm <= 0 || motion_abort_requested()) return 0;

    const int32_t l0 = enc_left_total, r0 = enc_right_total;

    /* The ISR only looks at the watch while a straight move is running, and
       move_straight_mm() does not start one until it has reset everything. */
    watch_rt.active        = 0u;
    watch_rt.kind          = (uint8_t)w->kind;
    watch_rt.side          = w->ir_side;
    watch_rt.log_face      = w->log_face;
    watch_rt.thresh_cm     = w->thresh_cm;
    watch_rt.sense_speed   = w->sense_speed;
    watch_rt.arm_counts    = (w->arm_mm > 0) ? (int32_t)((float)w->arm_mm / MM_PER_COUNT) : 0;
    watch_rt.after_counts  = (w->after_mm > 0) ? (int32_t)((float)w->after_mm / MM_PER_COUNT) : 0;
    watch_rt.armed         = (watch_rt.arm_counts == 0) ? 1u : 0u;
    watch_rt.triggered     = 0u;
    watch_rt.seen          = 0u;
    watch_rt.seen_ticks    = 0u;
    watch_rt.cond_ticks    = 0u;
    watch_rt.slowed        = 0u;
    watch_rt.ff_fade_from  = 0.0f;
    watch_rt.l0            = l0;
    watch_rt.r0            = r0;
    watch_rt.seen_mm       = 0;
    watch_rt.trigger_mm    = 0;
    watch_rt.trigger_speed = 0.0f;
    face_log_n             = 0u;
    face_log_last_mm       = 0;
    watch_rt.active        = 1u;

    uint8_t ok = move_straight_mm(max_mm);

    watch_rt.active = 0u;

    w->triggered  = watch_rt.triggered;
    w->seen       = watch_rt.seen;
    w->seen_mm    = watch_rt.seen_mm;
    w->trigger_mm = watch_rt.trigger_mm;
    w->trigger_speed_mm_tick = watch_rt.trigger_speed * MM_PER_COUNT;
    w->driven_mm  = wheels_mm_since(l0, r0);
    return ok;
}

uint8_t watch_face_fit(float trim, uint16_t min_n, float min_span_mm,
                       float *slope_cm_per_mm, uint16_t *n_used, float *span_mm)
{
    const uint16_t n = face_log_n;
    const uint16_t cut = (uint16_t)((float)n * trim);
    const uint16_t lo = cut;
    const uint16_t hi = (n > cut) ? (uint16_t)(n - cut) : 0u;   /* exclusive */

    *slope_cm_per_mm = 0.0f;
    *n_used = (hi > lo) ? (uint16_t)(hi - lo) : 0u;
    *span_mm = (hi > lo) ? (float)(face_log_mm[hi - 1u] - face_log_mm[lo]) : 0.0f;
    if (*n_used < min_n || *span_mm < min_span_mm) return 0u;

    /* Least squares, x relative to the first sample to keep the sums small. */
    const float x0 = (float)face_log_mm[lo];
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint16_t i = lo; i < hi; i++) {
        const float x = (float)face_log_mm[i] - x0;
        const float y = face_log_cm[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const float m   = (float)(*n_used);
    const float den = m * sxx - sx * sx;
    if (den <= 1e-3f) return 0u;
    *slope_cm_per_mm = (m * sxy - sx * sy) / den;
    return 1u;
}

uint8_t move_turn_deg(int8_t left, int8_t forward, int32_t degrees)
{
    if (motion_abort_requested()) return 0;
    if (degrees <= 0) { last_result = MOVE_DONE; return 1; } /* Zhenxi: see report_result() in command.c */

    turn_left           = left;
    dir_forward         = forward ? 1 : -1;
    target_deg_total    = (float)degrees;
    accum_deg           = 0.0f;
    move_ticks          = 0;
    stall_ticks_count   = 0;
    left_pid_integral   = 0.0f;
	right_pid_integral  = 0.0f;
    last_result         = MOVE_NONE; /* Zhenxi: this move has no verdict yet */

    /* Set Ackermann steering angle */
    if (left) servo_us(SERVO_LEFT);
    else      servo_us(SERVO_RIGHT);

    HAL_Delay(SERVO_SETTLE_MS); /* Allow servo to reach mechanical position (calib.h) */

    reset_speed_pid();
    if (!start_move(MODE_TURN_DEG)) return 0;   /* stopped during the wait above */

    /* Monitor sensors while turning */
	while (busy_flag) {
		/* Zhenxi: same as move_straight_mm -- let a STOP in mid-turn. */
		command_poll();

		/* Only check for front collisions if driving FORWARD in the turn */
		if (dir_forward == 1 && check_front_collision()) {
			stop_hardware(MOVE_DONE);

			/* GYRO FIX: Let the physical crash shockwave dissipate so
			 * the gyro returns to absolute 0 before calculating remaining angle! */
			HAL_Delay(400);

			float remaining_deg = target_deg_total - accum_deg;

			if (remaining_deg > 3.0f && target_deg_total > 45) {
				command_send("\r\n[WARN] COLLISION! Completing turn in REVERSE.\r\n");

				/* To continue the same yaw rotation while driving backward,
				 * we MUST invert the steering direction! */
				turn_left = !turn_left;
				dir_forward = -1;

				/* Reset accumulators for the reverse phase */
				target_deg_total = remaining_deg;
				accum_deg = 0.0f;
				left_pid_integral = 0.0f;
				right_pid_integral = 0.0f;

				/* Physically swing the wheels to the opposite lock */
				if (turn_left) servo_us(SERVO_LEFT);
				else           servo_us(SERVO_RIGHT);
				HAL_Delay(250);

				reset_speed_pid();

				/* THE CRITICAL FIX: Wake the motor ISR back up!
				 * stop_hardware() turned it off, so we must re-arm it.
				 * Kush (09-Oct): through start_move(), so a STOP during
				 * the waits above is not lost (see start_move). */
				if (!start_move(MODE_TURN_DEG)) return 0;  /* Continue the while loop, now in reverse! */
			} else {
				//stop_hardware(MOVE_BLOCKED);
				stop_hardware(MOVE_DONE);
				busy_flag = 0; /* Turn is basically complete, safe to abort */
				HAL_Delay(250);
				command_send("\r\n[WARN] Turn almost complete. Aborting.\r\n");
				return 0;
			}
		}
		HAL_Delay(5);
	}

	/* Final settle to ensure gyro is completely silent before next maneuver */
	/* Zhenxi: guarded for the same reason as in move_straight_mm -- do not
	 * overwrite a STALL / TIMEOUT / ABORT the ISR or STOP already recorded.
	 * The collision branch above deliberately leaves MOVE_DONE in place: a
	 * turn that finished in reverse, or was within 3 degrees, did reach its
	 * heading. The tablet learns about the displacement from the [WARN].  */
	if (last_result == MOVE_NONE) stop_hardware(MOVE_DONE);
	HAL_Delay(100);
	return 1;
}

void move_pivot_deg(int8_t left, int32_t degrees)
{
    if (motion_abort_requested()) return;
    if (degrees <= 0) return;

    turn_left           = left;
    dir_forward         = 1;
    target_deg_total    = (float)degrees;
    accum_deg           = 0.0f;
    move_ticks          = 0;
    stall_ticks_count   = 0;
    pivot_enc_l_accum   = 0;
    pivot_enc_r_accum   = 0;
    left_pid_integral = 0.0f;
	right_pid_integral = 0.0f;

    /* Command mechanical steering lock */
    if (left) {
        servo_us(SERVO_LEFT); /* 1000 us */
    } else {
        servo_us(2000);       /* Backed off from 2100 us to prevent binding */
    }
    HAL_Delay(220);

    reset_speed_pid();
    if (!start_move(MODE_PIVOT_DEG)) return;    /* stopped during the wait above */

    while (busy_flag) {
        command_poll();
        HAL_Delay(5);
    }
}

/**
 * @brief Executes a compact 90-degree 3-point turn.
 * @return 1 if successful, 0 if aborted due to obstacle.
 */
uint8_t move_kturn_90(int8_t left)
{
    if (motion_abort_requested()) return 0;
    uint8_t safe = 1;

    if (left)
    {
        /* 1. Forward-Left 45 degrees (Checks for obstacles) */
        safe = move_turn_deg(1, 1, 45);
        HAL_Delay(150);

        /* 2. Reverse-Right 45 degrees (Only if forward was safe) */
        if (safe) {
            move_turn_deg(0, 0, 45);
            HAL_Delay(150);
        }
    }
    else
    {
        /* 1. Forward-Right 45 degrees (Checks for obstacles) */
        safe = move_turn_deg(0, 1, 45);
        HAL_Delay(150);

        /* 2. Reverse-Left 45 degrees (Only if forward was safe) */
        if (safe) {
            move_turn_deg(1, 0, 45);
            HAL_Delay(150);
        }
    }

    /* Straighten wheels */
    servo_us(SERVO_CENTRE);
    HAL_Delay(100);

    return safe;
}

void move_turn(int8_t left, int8_t forward, int32_t counts)
{
    if (motion_abort_requested()) return;
    if (counts <= 0) return;

    turn_left           = left;
    dir_forward         = forward ? 1 : -1;
    target_counts_total = counts;
    accum_counts        = 0;
    move_ticks          = 0;
    stall_ticks_count   = 0;

    if (left) {
		servo_us(SERVO_LEFT);   /* 1000 us */
	} else {
		servo_us(SERVO_RIGHT);
	}
    HAL_Delay(200);

    reset_speed_pid();
    if (!start_move(MODE_TURN_RAW)) return;     /* stopped during the wait above */

    while (busy_flag) {
        command_poll();
        HAL_Delay(5);
    }
}

/* ------------------------------------------------------------------------- */
/* 100 Hz Control Interrupt (TIM6 Callback)                                  */
/* ------------------------------------------------------------------------- */

void control_tick(void)
{
    const float dt = 0.01f; /* 10 ms period */

    /* 1. Sample IMU Gyro Z */
    float gz = icm20948_read_gyro_z(); /* in deg/sec */
#if STRAIGHT_STEER_V2
    /* Kush: smoothed copy for the straight-line damping only, taken before
       the deadband so it has no steps. global_yaw_deg is unaffected. */
    gz_smooth += STEER_V2_RATE_SMOOTH * (gz - gz_smooth);
#endif
    if (fabsf(gz) < 0.25f) {  /* Ignore noise below 0.25 deg/sec */
        gz = 0.0f;
    }
    float delta_yaw = gz * dt;
    global_yaw_deg += delta_yaw;

    /* 2. Sample Encoders (ticks in this 10ms slice) */
    encoders_sample();
    int32_t left_delta  = enc_left_delta;
    int32_t right_delta = enc_right_delta;
    int32_t avg_delta   = (abs(left_delta) + abs(right_delta)) / 2;

    /* Kush: rear-axle odometry for the turn test, before the idle check so
     * it also follows the coast after a move. Heading is taken mid-tick.
     * Read-only bookkeeping: nothing below depends on it. */
    if (left_delta != 0 || right_delta != 0) {
        float ds_mm  = 0.5f * (float)(left_delta + right_delta) * MM_PER_COUNT;
        float hd_rad = (global_yaw_deg - 0.5f * delta_yaw) * (3.14159265f / 180.0f);
        odo_x_mm += ds_mm * cosf(hd_rad);
        odo_y_mm += ds_mm * sinf(hd_rad);
    }

    /* Kush (09-Oct): IR filtering and ultrasonic pings, every tick, idle too,
       so a filtered reading is always ready between moves (sensors.h). */
    sensors_tick();

    /* Kush (09-Oct): every tick, idle too (control.h). If it calls
       motion_stop(), the mode is idle by the check below. */
    {
        void (*hook)(void) = control_tick_hook;
        if (hook != 0) hook();
    }

    if (current_mode == MODE_IDLE) {
        return;
    }

    move_ticks++;

    /* --- Safety Checks (Stall & Timeout) --- */
    if (move_ticks > 25) { /* Grace period for initial motor startup */
        if (abs(left_delta) < STALL_MIN_COUNTS && abs(right_delta) < STALL_MIN_COUNTS) {
            stall_ticks_count++;
            if (stall_ticks_count >= STALL_TICKS) {
                stop_hardware(MOVE_STALL);
                return;
            }
        } else {
            stall_ticks_count = 0;
        }
    }

    if (move_ticks >= MOVE_TIMEOUT_TICKS) {
        stop_hardware(MOVE_TIMEOUT);
        return;
    }

    /* --- Closed-Loop Motion Modes --- */
    switch (current_mode)
    {
    	case MODE_STRAIGHT:
		{
			/* 1. Track cumulative ticks for target distance */
			accum_counts += avg_delta;
			const int32_t BRAKE_LEAD_COUNTS = 5/MM_PER_COUNT;

			/* Kush (09-Oct): measured speed for the brake point, and the sensor
			 * watch of move_straight_watch(), which can pull the target in. */
			straight_v_meas += 0.3f * (0.5f * (float)(abs(left_delta) + abs(right_delta)) - straight_v_meas);
			straight_watch_tick();

			if (accum_counts >= target_counts_total - BRAKE_LEAD_COUNTS) {
				stop_hardware(MOVE_DONE);
				return;
			}

			/* --- Slew-Rate Acceleration & Deceleration Ramp --- */
			/* Kush (09-Oct): cruise from the speed profile. While a watch is
			 * armed it runs at the watch's sense speed, and slows to it in
			 * time for the arm point (latched, so it cannot hunt). With the
			 * Task 2 extra feedforward it always does, even if the sense speed
			 * is not below the cruise: the cruise is then a real speed and the
			 * sense speed a command the wheels do ~45 % of. */
			float cruise = profile->straight_cruise;
			if (watch_rt.active && watch_rt.sense_speed > 0.0f &&
			    (watch_rt.slowed || watch_rt.sense_speed < cruise || straight_ff_extra_on())) {
				if (!watch_rt.slowed) {
					int32_t to_arm = watch_rt.arm_counts - accum_counts;
					if (watch_rt.armed ||
					    to_arm <= sense_slow_counts(fabsf(current_speed_ramp), watch_rt.sense_speed, straight_v_meas)) {
						watch_rt.ff_fade_from = fabsf(current_speed_ramp);
						watch_rt.slowed = 1u;
					}
				}
				if (watch_rt.slowed) cruise = watch_rt.sense_speed;
			}
			float target_speed = (float)dir_forward * cruise;
			int32_t remaining_counts = target_counts_total - accum_counts;
			const int32_t DECEL_TICKS = 250; /* Tune this! Distance to start braking */

			/* In control_tick() under MODE_STRAIGHT */
			int32_t remaining = abs(target_counts_total - accum_counts);

			/* If within 2 mm (~15 counts) and wheels have stopped spinning */
			if (remaining <= 15 && abs(left_delta) <= 1 && abs(right_delta) <= 1) {
			    static uint8_t in_pos_ticks = 0;
			    if (++in_pos_ticks > 20) { /* 200 ms motionless in tolerance */
			        in_pos_ticks = 0;
			        stop_hardware(MOVE_DONE);
			        return;
			    }
			}

			const float RAMP_STEP = STRAIGHT_RAMP_STEP; /* Accelerates/Decelerates smoothly */
			/* Kush (09-Oct): from the profile: 15 in both Task 1 and Task 2,
			 * the value Task 2 was tested with before 0c37bfe made it 50. A
			 * watch move always uses SPEED_CRAWL (see straight_stop_counts). */
			const float CRAWL_SPEED = watch_rt.active ? (float)SPEED_CRAWL : profile->straight_crawl;

			/* Kush: start slowing early enough to reach crawl speed before the
			 * target. Ramping down from speed v to crawl at RAMP_STEP per tick
			 * takes (v^2 - crawl^2) / (2 * RAMP_STEP) counts. The fixed 250-count
			 * (34 mm) zone alone only covers that from about 28 counts/tick, so
			 * from the 60 counts/tick cruise the car reached the target still
			 * doing ~55 and ran on past it. That started to show once the planner
			 * began joining FW010s into long FW moves. Latched, so a wheel that
			 * lags the ramp cannot flip it back to cruise speed.
			 * Kush (09-Oct): that formula is still what the Task 1 profile uses;
			 * the Task 2 profile works it out from the measured speed instead
			 * (straight_stop_counts / ramp_down_counts at the top). */
			int32_t brake_counts = straight_stop_counts();
			if (straight_braking || remaining_counts < DECEL_TICKS || remaining_counts < brake_counts) {
				straight_braking = 1u;
				target_speed = (float)dir_forward * CRAWL_SPEED; /* Crawl speed */
			}

			/* Kush (12-Oct, STRAIGHT_FAST_START in calib.h): speeding up (the
			 * command moving away from 0, either direction) goes
			 * STRAIGHT_RAMP_UP_X times as fast; slowing is RAMP_STEP as before,
			 * which is what the brake point (straight_stop_counts) works with. */
			const float RAMP_UP = STRAIGHT_FAST_START ? STRAIGHT_RAMP_UP_X * RAMP_STEP : RAMP_STEP;
			if (current_speed_ramp < target_speed) {
				current_speed_ramp += (current_speed_ramp >= 0.0f) ? RAMP_UP : RAMP_STEP;
				if (current_speed_ramp > target_speed) current_speed_ramp = target_speed;
			} else if (current_speed_ramp > target_speed) {
				current_speed_ramp -= (current_speed_ramp <= 0.0f) ? RAMP_UP : RAMP_STEP;
				if (current_speed_ramp < target_speed) current_speed_ramp = target_speed;
			}

			/* 2. Accumulate individual wheel ticks for differential steering */
			enc_left_straight_accum  += abs(left_delta);
			enc_right_straight_accum += abs(right_delta);

			/* ======================================================================
			 * SMALL DISTANCE PID BYPASS
			 * For tiny creep loops (e.g., 30mm parking adjustments), we bypass
			 * the steering PID to prevent wobbly wheels, and we freeze the
			 * velocity integral to prevent sudden lurching/overshoot.
			 * ====================================================================== */
			const int32_t PID_BYPASS_TICKS = 60/MM_PER_COUNT;
			uint8_t is_small_distance = (target_counts_total <= PID_BYPASS_TICKS);

			/* Kush: fixed steering trim while reversing (calib.h). Reversing,
			 * the front wheels get pushed off centre and the car drifted to its
			 * right. Unlike the old reverse_bias it is not faded with speed: the
			 * push is there in the slow start and stop too. */
			int16_t reverse_trim = (dir_forward == -1) ? (int16_t)SERVO_REVERSE_TRIM_US : 0;
			uint16_t servo_cmd = SERVO_CENTRE;   /* for straight_tick_hook only */

			if (is_small_distance) {
				/* Lock wheels dead center for tiny movements */
				servo_cmd = (uint16_t)(SERVO_CENTRE + reverse_trim);
				servo_us(servo_cmd);
			} else {
				/* --- SENSOR FUSION: IMU + Encoders (NORMAL PID LOGIC) --- */

				/* 1. Heading Error (Degrees) */
				float heading_error = global_yaw_deg - locked_heading_deg;
				if (dir_forward == -1) {
					heading_error = -heading_error;
				}

#if STRAIGHT_STEER_V2
				/* Kush (08-Oct), see STRAIGHT_STEER_V2 at the top. The car turns
				 * at a rate proportional to speed x steering angle, so scaling
				 * the gains by REF/speed keeps the correction the same at every
				 * speed: twice the steering in the crawl, less if the straights
				 * get faster. Measured wheel speed, lightly smoothed. */
				float v_now = 0.5f * (float)(abs(left_delta) + abs(right_delta));
				steer_speed_f += 0.2f * (v_now - steer_speed_f);
				float scale = STEER_REF_SPEED / ((steer_speed_f > 1.0f) ? steer_speed_f : 1.0f);
				if (scale > STEER_SCALE_MAX) scale = STEER_SCALE_MAX;
				if (scale < STEER_SCALE_MIN) scale = STEER_SCALE_MIN;

				/* Damping: how fast the heading error is changing, from the
				 * smoothed gyro rate (gz_smooth, top of control_tick). Cut with
				 * the gains at speed, but never boosted when slow: that only
				 * amplified the vibration in the crawl. */
				float heading_rate = (float)dir_forward * gz_smooth;
				float d_scale = (scale < 1.0f) ? scale : 1.0f;

				float steer_f = heading_error * STEER_V2_KP * scale
				              + heading_rate  * STEER_V2_KD * d_scale;
				if (steer_f >  220.0f) steer_f =  220.0f;   /* MAX_STEER_TRIM, as before */
				if (steer_f < -220.0f) steer_f = -220.0f;
				int16_t steer_correction = (int16_t)steer_f;
#else
				/* 2. Position Error (Ticks) */
				int32_t pos_error = (enc_right_straight_accum - enc_left_straight_accum);
				if (pos_error > 20)  pos_error = 20;
				if (pos_error < -20) pos_error = -20;

				/* 3. Rate Error (Derivative - Ticks per 10ms) */
				float rate_error = (float)(abs(right_delta) - abs(left_delta));

				/* --- The Direct Gains --- */
				float HEADING_KP = 130.0f;
				float POS_KP     = 0.0f;
				float STEER_KI   = 0.0f;//2.0f;
				float STEER_KD   = 3.5f;//5.0f;
				const int16_t MAX_STEER_TRIM = 220;

				/* 4. Integral Accumulation (Auto-Trim) */
				float combined_error = heading_error + ((float)pos_error * 0.1f);
				steer_integral += combined_error * dt;

				if (steer_integral > 150.0f)  steer_integral = 150.0f;
				if (steer_integral < -150.0f) steer_integral = -150.0f;

				/* Calculate dynamic steering correction */
				int16_t steer_correction = (int16_t)((heading_error * HEADING_KP) +
													 ((float)pos_error * POS_KP) +
													 (steer_integral * STEER_KI) +
													 (rate_error * STEER_KD));

				/* Understeer Fade for Deceleration */
				float speed_ratio = fabsf(current_speed_ramp) / (float)SPEED_STRAIGHT;
				steer_correction = (int16_t)(steer_correction * speed_ratio);

				/* Clamp maximum steering authority */
				if (steer_correction > MAX_STEER_TRIM)  steer_correction = MAX_STEER_TRIM;
				if (steer_correction < -MAX_STEER_TRIM) steer_correction = -MAX_STEER_TRIM;
#endif

				/* Apply to servo */
				uint16_t commanded_servo = (uint16_t)(SERVO_CENTRE + reverse_trim + steer_correction);
				servo_us(commanded_servo);
				servo_cmd = commanded_servo;
			}

			/* 3. Velocity PI Controller */
			float err_l = current_speed_ramp - (float)left_delta;
			float err_r = current_speed_ramp - (float)right_delta;

			if (!is_small_distance) {
				/* Normal PI integral tracking for long distances */
				left_pid_integral  += err_l * dt;
				right_pid_integral += err_r * dt;

				/* Anti-windup clamping */
				if (left_pid_integral > 250.0f)  left_pid_integral = 250.0f;
				if (left_pid_integral < -250.0f) left_pid_integral = -250.0f;
				if (right_pid_integral > 250.0f)  right_pid_integral = 250.0f;
				if (right_pid_integral < -250.0f) right_pid_integral = -250.0f;
			} else {
				/* Allow gentle integral accumulation to overcome stiction */
				left_pid_integral  += err_l * dt * 0.5f;
				right_pid_integral += err_r * dt * 0.5f;
				if (left_pid_integral > 120.0f)  left_pid_integral = 120.0f;
				if (left_pid_integral < -120.0f) left_pid_integral = -120.0f;
				if (right_pid_integral > 120.0f)  right_pid_integral = 120.0f;
				if (right_pid_integral < -120.0f) right_pid_integral = -120.0f;
			}

			/* Pushes baseline power so the weaker left motor doesn't stall when braking */
			int32_t ff = (int32_t)(dir_forward * (fabsf(current_speed_ramp) * 20.0f));

			/* Kush (09-Oct): the rest of the feedforward, Task 2 only.
			 * 20 duty per count/tick is about a tenth of what the motors need
			 * (~190: full PWM gives ~82), so the wheels only ever reached ~45 %
			 * of the command. This adds what is missing, but only for the part
			 * of the command ABOVE the crawl: at the crawl it adds nothing, so
			 * the crawl and the stop are exactly the Task 1 ones. It is also
			 * off while a sensing phase holds its sense speed (move_straight_
			 * watch), so those run as tuned, and when reversing, so the reverse
			 * steering trim keeps the speeds it was set at. MODE_TURN_DEG has
			 * its own feedforward below and is not touched. */
			if (profile->straight_ff_extra > 0.0f && dir_forward == 1) {
				const float sp = fabsf(current_speed_ramp);
				float extra = 0.0f;
				if (watch_rt.active && watch_rt.slowed) {
					/* A sensing phase has taken over: fade the extra out as the
					 * command falls from where it took over to the sense speed,
					 * so it is gone (and the phase runs as tuned) by then,
					 * without a step in the duty. A watch armed from the start
					 * takes over at 0 and never gets any. */
					const float hi = watch_rt.ff_fade_from, lo = watch_rt.sense_speed;
					if (hi > lo + 1.0f && sp > lo) {
						const float s = (sp < hi) ? sp : hi;
						extra = profile->straight_ff_extra * (hi - profile->straight_crawl) * (s - lo) / (hi - lo);
					}
				} else if (sp > profile->straight_crawl) {
					extra = profile->straight_ff_extra * (sp - profile->straight_crawl);
				}
				ff += (int32_t)extra;
			}

			int32_t duty_l = ff + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
			int32_t duty_r = ff + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

			motor_left(duty_l);
			motor_right(duty_r);

			/* Kush: CAL_TRACE only (control.h). Reads, never writes. */
			if (straight_tick_hook) {
				straight_tick_t t = {
					.ramp     = current_speed_ramp,
					.left     = left_delta,
					.right    = right_delta,
					.duty_l   = duty_l,
					.duty_r   = duty_r,
					.i_l      = SPEED_KI * left_pid_integral,
					.i_r      = SPEED_KI * right_pid_integral,
					.head_deg = global_yaw_deg - locked_heading_deg,
					.steer_us = (int16_t)((int32_t)servo_cmd - SERVO_CENTRE),
					.braking  = straight_braking,
				};
				straight_tick_hook(&t);
			}
			break;
		}

    	case MODE_TURN_DEG:
		{
			/* 1. True Ackermann Yaw Integration */
			float step_yaw = delta_yaw * (float)dir_forward;
			if (turn_left) {
				accum_deg += step_yaw;
			} else {
				accum_deg -= step_yaw;
			}

			/* 2. Direction-Aware Inertia Lead:
			 * Forward coasts ~2.5 deg on momentum.
			 * Reverse has heavy tire scrub and zero coast, so lead must be much smaller. */
			float braking_lead = 2.5f;
			if (target_deg_total <= 45){
				if (turn_left)
					braking_lead = (dir_forward == 1) ? 4.7f : 4.4f;
				else
					braking_lead = (dir_forward == 1) ? 4.0f : 3.3f;
			}
			float remaining_deg = target_deg_total - accum_deg;

			if (remaining_deg <= braking_lead) {
				stop_hardware(MOVE_DONE);
				return;
			}

			/* Settle timeout: only trigger if within 0.5 deg of lead and genuinely stopped */
			if (remaining_deg <= (braking_lead + 0.5f) && fabsf(gz) < 0.25f) {
				static uint8_t turn_settle_ticks = 0;
				if (++turn_settle_ticks > 20) { /* 200 ms stopped */
					turn_settle_ticks = 0;
					stop_hardware(MOVE_DONE);
					return;
				}
			}

			/* --- Slew-Rate Acceleration & Deceleration Ramp --- */
			float target_base_speed = (float)(dir_forward * SPEED_TURN);
			float ramp_up_step      = 1.2f;   /* RAMP_STEP below, while speeding up */
#if TURN_FAST
			/* Kush (11-Oct, speed step 6, TURN_FAST in calib.h): faster in the
			 * middle of the turn only. Back at SPEED_TURN TURN_FAST_HANDOVER_DEG
			 * before the slow-down below starts, so from there the end of the
			 * turn is the same as before. */
			{
				const float slow_from_deg = (target_deg_total > 45.0f) ? 20.0f : 6.0f; /* DECEL_DEG / SHORT_DECEL_DEG */
				if (remaining_deg >= slow_from_deg + TURN_FAST_HANDOVER_DEG) {
					target_base_speed = (float)(dir_forward * SPEED_TURN_FAST);
					ramp_up_step      = TURN_FAST_RAMP_UP;
				}
			}
#endif

			/* FIX: Change '>=' to '>' so 45-deg turns carry clean, constant speed
			 * without losing momentum over a 20-deg crawl window. */
			if (target_deg_total > 45.0f) {
				const float DECEL_DEG = 20.0f;
				if (remaining_deg < DECEL_DEG) {
					target_base_speed = (float)(dir_forward * 30.0f); /* Crawl speed */
				}
			}

			if (target_deg_total <= 45.0f){
				const float SHORT_DECEL_DEG = 6.0f;
				if (remaining_deg < SHORT_DECEL_DEG){
					target_base_speed = (float)(dir_forward * 22.0f);
				}
			}

			const float RAMP_STEP = 1.2f;

			/* Kush (11-Oct): ramp_up_step only while moving away from 0
			 * (speeding up, either direction); slowing is always RAMP_STEP.
			 * With TURN_FAST 0 both are 1.2, as before. */
			if (current_speed_ramp < target_base_speed) {
				current_speed_ramp += (current_speed_ramp >= 0.0f) ? ramp_up_step : RAMP_STEP;
				if (current_speed_ramp > target_base_speed) current_speed_ramp = target_base_speed;
			} else if (current_speed_ramp > target_base_speed) {
				current_speed_ramp -= (current_speed_ramp <= 0.0f) ? ramp_up_step : RAMP_STEP;
				if (current_speed_ramp < target_base_speed) current_speed_ramp = target_base_speed;
			}

			/* 3. Differential Ackermann Wheel Speeds */
			float target_l, target_r;

			if (turn_left) {
				target_l = current_speed_ramp * 0.70f;
				target_r = current_speed_ramp * 1.30f;
			} else {
				target_l = current_speed_ramp * 1.30f;
				target_r = current_speed_ramp * 0.70f;
			}

			float err_l = target_l - (float)left_delta;
			float err_r = target_r - (float)right_delta;

			left_pid_integral  += err_l * dt;
			right_pid_integral += err_r * dt;

			/* Anti-windup clamping */
			if (left_pid_integral > 250.0f)  left_pid_integral = 250.0f;
			if (left_pid_integral < -250.0f) left_pid_integral = -250.0f;
			if (right_pid_integral > 250.0f)  right_pid_integral = 250.0f;
			if (right_pid_integral < -250.0f) right_pid_integral = -250.0f;

			/* 4. DYNAMIC Direction-Aware Feedforward */
			/* Scales the feedforward down as the speed ramps down, preserving the turning radius */
			float speed_ratio = fabsf(current_speed_ramp) / (float)SPEED_TURN;
			int32_t ff = (int32_t)((float)dir_forward * 1200.0f * speed_ratio);

			int32_t duty_l = ff + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
			int32_t duty_r = ff + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

			motor_left(duty_l);
			motor_right(duty_r);
			break;
		}
    	case MODE_PIVOT_DEG:
		{
			/* 1. Integrate Absolute Gyro Yaw */
			if (turn_left) {
				accum_deg += delta_yaw;
			} else {
				accum_deg -= delta_yaw;
			}

			/* 2. Target Completion Check */
			if (accum_deg >= target_deg_total) {
				stop_hardware(MOVE_DONE);
				return;
			}

			/* 3. Symmetric Counter-Rotating Speed Targets */
			const float MAX_PIVOT_SPEED = (float)SPEED_TURN;
			const float RAMP_STEP = 1.5f; /* Accelerates by 1.5 ticks every 10ms */

			if (pivot_speed_ramp < MAX_PIVOT_SPEED) {
			    pivot_speed_ramp += RAMP_STEP;
			    if (pivot_speed_ramp > MAX_PIVOT_SPEED) pivot_speed_ramp = MAX_PIVOT_SPEED;
			}

			float target_l = turn_left ? -pivot_speed_ramp :  pivot_speed_ramp;
			float target_r = turn_left ?  pivot_speed_ramp : -pivot_speed_ramp;

			/* 4. PID Error Calculations */
			float err_l = target_l - (float)left_delta;
			float err_r = target_r - (float)right_delta;

			left_pid_integral  += err_l * dt;
			right_pid_integral += err_r * dt;

			/* Anti-windup clamping */
			if (left_pid_integral > 300.0f)  left_pid_integral = 300.0f;
			if (left_pid_integral < -300.0f) left_pid_integral = -300.0f;
			if (right_pid_integral > 300.0f)  right_pid_integral = 300.0f;
			if (right_pid_integral < -300.0f) right_pid_integral = -300.0f;

			/* 5. Anti-Creep (Forward Drift Prevention) */
			pivot_enc_l_accum += left_delta;
			pivot_enc_r_accum += right_delta;
			int32_t net_translation = pivot_enc_l_accum + pivot_enc_r_accum;

			int32_t creep_trim = 0;
			if (net_translation > 10) { /* If chassis creeps forward more than ~1.5mm */
				creep_trim = (net_translation - 10) * 12; /* Kp of 12 for drift correction */
				if (creep_trim > 600) creep_trim = 600;   /* Cap trim to prevent violent vibration */
			}

			/* 6. Symmetric Feedforward */
			/* 1500 is roughly 9% duty cycle, enough to break average stiction */
			int32_t ff_l = turn_left ? -3000 :  3000;
			int32_t ff_r = turn_left ?  3000 : -3000;

			/* 7. Motor Output */
			/* Subtracting creep_trim forces the robot backward if it drifts forward */
			int32_t duty_l = ff_l + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral) - creep_trim;
			int32_t duty_r = ff_r + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral) - creep_trim;

			motor_left(duty_l);
			motor_right(duty_r);
			break;
		}

        case MODE_TURN_RAW:
        {
            accum_counts += avg_delta;

            if (accum_counts >= target_counts_total) {
                stop_hardware(MOVE_DONE);
                return;
            }

            float target_speed = (float)(dir_forward * SPEED_TURN);
            float err_l = target_speed - (float)left_delta;
            float err_r = target_speed - (float)right_delta;

            left_pid_integral  += err_l * dt;
            right_pid_integral += err_r * dt;

            int32_t duty_l = (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
            int32_t duty_r = (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

            motor_left(duty_l);
            motor_right(duty_r);
            break;
        }

        case MODE_IDLE:
        default:
            break;
    }
}
