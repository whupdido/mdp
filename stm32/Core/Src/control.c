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
uint8_t t2_no_collision = 0;  // set by obstacle_nav.c during the obs2 dodge

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
#define SPEED_KP    80.0f
#define SPEED_KI    15.0f

/* Kush (15-Oct): rear wheel sync on forward straights. The left motor is
   ~6 % stronger (85 against 80 counts/tick at full PWM, calib.h) and the
   speed loops' integral (SPEED_KI) takes ~20 s to even that out, so for a
   whole move the left wheel ran faster and turned the car right. The
   steering only held the heading against it by staying a little off it,
   so the car drifted right (simulated: 24 mm over 1.5 m, 60 mm over 3 m).
   This moves duty from the faster wheel to the slower one until they match:
   half each way, or all of it off the faster one when the slower is already
   flat out, so at full speed both run at the weaker motor's top (~80).
   Simulated: 1-2 mm over the same moves, the same response to a knock, and
   stops a few mm closer. Reversing is left as it was (SERVO_REVERSE_TRIM_US
   was set with the mismatch in it). KP and KI both 0 switch it off. */
#define WHEEL_SYNC_KP     40.0f    /* duty per count/tick of left-right speed difference */
#define WHEEL_SYNC_KI     1500.0f  /* duty per count/tick x s                            */
#define WHEEL_SYNC_MAX    6000.0f  /* most it may move between the wheels, duty         */
static float wheel_sync_int = 0.0f;   /* left-right difference x time, reset every move */

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
 * Check with CAL_TRACE (hdg per phase) and CAL_STRAIGHT.
 *
 * Kush (15-Oct): retuned for the Task 2 straights at full speed, ~80
 * counts/tick, three times the 27 these were first set at. The car turns at
 * a rate proportional to speed x steering, so the same steering now turns it
 * three times as fast, while the servo still takes just as long to get
 * there. Two things had made that oscillate:
 *  - The speed scaling (scale = STEER_REF_SPEED / speed) is what keeps the
 *    gains in step with the speed, and it stopped at STEER_SCALE_MIN 0.4,
 *    i.e. above ~67/tick, so at full speed the gains were ~20 % too strong.
 *    It now goes down to 0.2 (~135/tick).
 *  - The damping (KD on the gyro rate) is what stops each swing from
 *    overshooting into the next, and it had been cut from 10 to 1, with the
 *    gyro smoothing slowed from 0.35 to 0.1 (~90 ms behind instead of ~20),
 *    while KP went up.
 * Simulated (hosttest/world, world_st.c) at ~1.1 m/s after a 3 deg knock,
 * with the servo taking anywhere from 40 to 260 ms to follow:
 *   KP 120, KD 1, 0.1, floor 0.4: up to 6 swings; never settled with a slow
 *     servo, and grew with a very slow one.
 *   KP 70, KD 8, 0.35, floor 0.2: 1-2 swings, back within 0.3 deg in
 *     0.3-0.5 s, a little less servo jitter from vibration than KD 10.
 * Tuning on the floor at full speed (CAL_STRAIGHT with CAL_USE_TASK2_PROFILE
 * 1 in main.c): knock the nose sideways mid-move.
 *   Swings past the line more than once: raise KD by 2, or lower KP by 10.
 *   Comes back slowly, or holds a small angle: raise KP by 10.
 *   Servo buzzes on a plain straight: lower KD by 2 (or RATE_SMOOTH to 0.25).
 *   Always ends a little to the same side: STEER_V2_KI 20-40.
 * Change one value at a time.
 * Kush (15-Oct, later): STEER_V2_KI 0 -> 30, with WHEEL_SYNC (above) for
 * the right drift: the sync takes out the motor mismatch, the integral what
 * is left (a steering a little off centre: simulated 10 us right, 22 mm off
 * over 3 m without it, 14 mm with it). A knock now overshoots 13 % instead
 * of 10 %, same 1 swing. If it still ends to one side, check SERVO_CENTRE. */
#define STRAIGHT_STEER_V2   1
#define STEER_V2_KP         70.0f   /* us per deg of heading error (120 before 15-Oct) */
#define STEER_V2_KD         8.0f    /* us per deg/s of yaw rate, from the gyro (1)   */
#define STEER_V2_RATE_SMOOTH 0.35f  /* share of each new gyro reading in that rate (0.1) */
#define STEER_V2_KI         40.0f   /* us per deg*s of heading error, 0 = off. Reset at
                                       the start of every straight: it used to carry
                                       over from one move into the next             */
#define STEER_V2_I_MAX      60.0f   /* most the integral may add, us                 */
#define STEER_REF_SPEED     27.0f   /* counts/tick the gains hold at: the cruise
                                       speed CAL_TRACE measured on 08-Oct            */
#define STEER_SCALE_MIN     0.2f    /* gains never cut below this (above ~135/tick; 0.4 before) */
/* Kush (15-Oct): steering held in while pulling away. At the start of a
   straight the car is slow, so a heading error asks for a lot of steering
   (the gains scale up as the speed drops), and the servo only gets there
   ~100 ms later, by which time the car is going two or three times as fast
   and that steering turns it two or three times as hard: it swings past and
   rings for a couple of swings. For the first STEER_LAUNCH_MM of a straight
   the steering is held to +-STEER_LAUNCH_US instead of +-220. Simulated, a
   2 deg knock 0.1 s into a move: 92 % overshoot and 3 swings before, 17 %
   and 1 after (0.2 s in: 46 % / 4 swings before, 20 % / 2 after). At speed
   and for the drift nothing changes. Still wobbles at the start: lower
   STEER_LAUNCH_US to 30-40. */
#define STEER_LAUNCH_MM     150.0f
#define STEER_LAUNCH_US     50.0f
#define STEER_SCALE_MAX     2.0f    /* or KP boosted above this (below ~13/tick);
                                       the damping is never boosted, only cut       */
/* Kush (15-Oct): hold the line, not only the heading. The steering above
   keeps the car pointing where the straight started, but any stretch spent
   pointing off that (pulling away, a knock, the integral still learning a
   steering offset) moves the car sideways, and pointing straight again does
   not undo it: the car ended square but off to one side. The ISR's odometry
   (motion_odometry_mm) says how far the rear-axle centre is off the line the
   straight started on, and this aims the car back at that line, at
   atan(off / STEER_LINE_MM), held to +-STEER_LINE_MAX_DEG. It lets go once
   the straight starts braking, so the car still ends square.
   Simulated over 1.5 m / 3 m with the steering 25 us off centre: 18 / 28 mm
   to the side before, 7 / 3 mm after, and still square at the end. A knock
   swings the same and ends ~1 mm off the line instead of ~4. It only
   sees what the gyro and encoders see: a car that slides sideways while
   pointing straight is SERVO_CENTRE or the tyres. Smaller STEER_LINE_MM
   pulls back harder (300: a little closer, a little livelier after a
   knock); 0 switches it off. */
#define STEER_LINE_MM       400.0f  /* closes a sideways gap over about this, mm */
#define STEER_LINE_MAX_DEG  2.0f    /* most it aims off the start heading, deg   */
static float line_x0 = 0.0f, line_y0 = 0.0f;   /* odometry where the straight began */
static float line_cos = 1.0f, line_sin = 0.0f; /* and the heading it holds          */
static float steer_speed_f = 0.0f;  /* smoothed wheel speed for that, counts/tick   */
#if STRAIGHT_STEER_V2
static float gz_smooth     = 0.0f;  /* smoothed gyro rate for the damping, deg/s    */
#endif

/* Encoder counts of crawl kept before a straight move's target, so the
   wheels have settled at crawl speed when the brake goes on (~14 mm). */
#define BRAKE_MARGIN_COUNTS 100
float steer_integral = 0.0f;
#if STRAIGHT_STEER_V2
static float steer_v2_int = 0.0f;   /* what STEER_V2_KI has added up, us         */
#endif

/* Accelerates/decelerates the commanded speed by this much per tick. */
#define STRAIGHT_RAMP_STEP  1.2f

/* Kush (15-Oct): how a move_straight_watch() with a blind stretch (arm_mm,
   only the obstacle 1 search on the way home) hands over from the cruise to
   its sense speed. The cruise runs on Task 2's extra feedforward and the
   sense speed without it, and the extra used to fade out as the command fell
   to the sense speed. With T2_SENSE_OBS1 above the cruise the command never
   falls, so the extra went in one tick: the duty dropped from full to about a
   quarter, and the motors (the AT8236 brakes in the off part of each PWM
   cycle) braked the rear wheels from ~80 to ~45 counts/tick in ~0.1 s.
   Simulated that is ~9 m/s2, far past what rear tyres grip, so on the floor
   they skid, the rear steps out and the car weaves just as it reaches speed
   heading home; the steering also scales its gains on the encoder speed,
   which reads low while the wheels skid. Now the extra fades out by distance
   over this share of the way from the takeover to the arm point (where the
   watch starts looking), whatever the sense speed. Simulated with sense 120:
   ~2.5 m/s2 at most, the same as the brake ramp at the end of every straight,
   at the sense speed by the arm point as before, and ~0.1 s quicker. Lower
   sense speeds now slow about that hard too (70 took ~4 m/s2 before, 95 the
   same one-tick drop as 120). Car still above the sense speed when the watch
   starts looking: lower to 0.75. */
#define WATCH_FADE_SHARE    0.85f

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
    float    ff_fade_extra; /* Task 2 extra feedforward when it slowed, duty:
                               it fades out from there (WATCH_FADE_SHARE)  */
    int32_t  fade_counts0;  /* accum_counts when it slowed                 */
    int32_t  arm_counts;    /* in accum_counts units, like the move's target   */
    int32_t  after_counts;
    int32_t  l0, r0;        /* encoder totals at the start of the move         */
    int32_t  min_seen_mm;
    uint8_t  stop_at_trigger;
    uint8_t  blips;
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

    /* Kush (14-Oct): seen over less than min_seen_mm, then lost: a flicker
       of noise, not a face. Forget it (and what the face log took from it)
       and keep looking. */
    if (watch_rt.kind == WATCH_IR_PASS && (mm - watch_rt.seen_mm) < watch_rt.min_seen_mm) {
        watch_rt.seen       = 0u;
        watch_rt.seen_ticks = 0u;
        watch_rt.cond_ticks = 0u;
        if (watch_rt.blips < 255u) watch_rt.blips++;
        if (watch_rt.log_face) face_log_n = 0u;
        return;
    }

    /* Confirmed. Stop after_counts from here -- or further, if the car needs
       more room than that to slow to its crawl, so it always stops the
       normal way instead of braking hard from speed.
       Kush (15-Oct): stop_at_trigger brakes once after_counts is covered,
       from whatever speed the car has (MODE_STRAIGHT stops the move as soon
       as accum_counts reaches the target, this tick if after_counts is 0). */
    watch_rt.triggered      = 1u;
    watch_rt.trigger_mm     = mm;
    watch_rt.trigger_speed  = straight_v_meas;
    int32_t room = watch_rt.stop_at_trigger ? 0 : straight_stop_counts();
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
    wheel_sync_int            = 0.0f;
#if STRAIGHT_STEER_V2
    steer_v2_int              = 0.0f;
#endif
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
    /* Kush (15-Oct): the line this straight holds (STEER_LINE_MM). */
    {
        float x0, y0;
        motion_odometry_mm(&x0, &y0);
        line_x0  = x0;
        line_y0  = y0;
        line_cos = cosf(locked_heading_deg * (3.14159265f / 180.0f));
        line_sin = sinf(locked_heading_deg * (3.14159265f / 180.0f));
    }

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
		if (!t2_no_collision && dir_forward == 1 && check_front_collision() ) {
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
    w->blips     = 0u;
    if (max_mm <= 0 || motion_abort_requested()) return 0;

    const int32_t l0 = enc_left_total, r0 = enc_right_total;

    /* The ISR only looks at the watch while a straight move is running, and
       move_straight_mm() does not start one until it has reset everything. */
    watch_rt.active        = 0u;
    watch_rt.kind          = (uint8_t)w->kind;
    watch_rt.side          = w->ir_side;
    watch_rt.log_face      = w->log_face;
    watch_rt.min_seen_mm   = (w->kind == WATCH_IR_PASS) ? w->min_seen_mm : 0;
    watch_rt.stop_at_trigger = w->stop_at_trigger;
    watch_rt.blips         = 0u;
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
    watch_rt.ff_fade_extra = 0.0f;
    watch_rt.fade_counts0  = 0;
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
    w->blips      = watch_rt.blips;
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

uint8_t watch_face_level(float trim, uint16_t min_n, float *level_cm, uint16_t *n_used)
{
    static float s[FACE_LOG_MAX];
    const uint16_t n = face_log_n;
    const uint16_t cut = (uint16_t)((float)n * trim);
    const uint16_t lo = cut;
    const uint16_t hi = (n > cut) ? (uint16_t)(n - cut) : 0u;   /* exclusive */
    uint16_t m = 0u;

    for (uint16_t i = lo; i < hi; i++) s[m++] = face_log_cm[i];
    *n_used   = m;
    *level_cm = 0.0f;
    if (m == 0u || m < min_n) return 0u;

    for (uint16_t i = 1u; i < m; i++) {          /* insertion sort, m <= 256 */
        const float v = s[i];
        int16_t j = (int16_t)(i - 1u);
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    *level_cm = (m & 1u) ? s[m / 2u] : 0.5f * (s[m / 2u - 1u] + s[m / 2u]);
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
		if (!t2_no_collision && dir_forward == 1 && check_front_collision()) {
			stop_hardware(MOVE_DONE);

			/* GYRO FIX: Let the physical crash shockwave dissipate so
			 * the gyro returns to absolute 0 before calculating remaining angle! */
			//HAL_Delay(200);

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
				//HAL_Delay(100);

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
				//HAL_Delay(100);
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
						/* The extra feedforward the last tick had (MODE_STRAIGHT
						   below), to fade out from. */
						const float sp0 = fabsf(current_speed_ramp);
						watch_rt.ff_fade_extra = (straight_ff_extra_on() && sp0 > profile->straight_crawl)
						    ? profile->straight_ff_extra * (sp0 - profile->straight_crawl) : 0.0f;
						watch_rt.fade_counts0 = accum_counts;
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
			/* Kush (15-Oct): a watch with stop_at_trigger keeps its speed to
			 * the target and brakes there (control.h), instead of slowing to
			 * the crawl for it: after the trigger, and also on the way to its
			 * max_mm, which at a high sense speed it used to start slowing for
			 * before it had even reached what it was looking for (from a sense
			 * speed of 120 the crawl needs ~55 cm; the obstacle 1 search window
			 * is 50 cm, so it crawled the whole search). */
			const uint8_t hold_to_stop = (watch_rt.active && watch_rt.stop_at_trigger) ? 1u : 0u;
			if (straight_braking ||
			    (!hold_to_stop && (remaining_counts < DECEL_TICKS || remaining_counts < brake_counts))) {
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
				/* Kush (15-Oct): back onto the line (STEER_LINE_MM at the top).
				 * off = how far left (+) of it the rear-axle centre is. The same
				 * sign works reversing: heading_error is already flipped there. */
				if (STEER_LINE_MM > 0.0f && !straight_braking) {
					const float lx  = odo_x_mm - line_x0;
					const float ly  = odo_y_mm - line_y0;
					const float off = -lx * line_sin + ly * line_cos;
					float aim = atanf(off / STEER_LINE_MM) * (180.0f / 3.14159265f);
					if (aim >  STEER_LINE_MAX_DEG) aim =  STEER_LINE_MAX_DEG;
					if (aim < -STEER_LINE_MAX_DEG) aim = -STEER_LINE_MAX_DEG;
					heading_error += aim;
				}
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

				/* Integral (STEER_V2_KI): reset with each move, and held at
				 * +-STEER_V2_I_MAX us so it cannot wind up past it.
				 * Kush (15-Oct): added up in us. It used to be deg*s multiplied
				 * by the scale of the moment, so slowing for the target (scale
				 * going up) grew it and turned the car a little at the end of a
				 * straight; what it learns is a steering offset, the same at any
				 * speed. Nothing is added while pulling away (STEER_LAUNCH_MM):
				 * that is a passing kick, and it would keep it for the whole move. */
				if (STEER_V2_KI > 0.0f && accum_counts >= (int32_t)(STEER_LAUNCH_MM / MM_PER_COUNT)) {
					steer_v2_int += heading_error * STEER_V2_KI * scale * dt;
					if (steer_v2_int >  STEER_V2_I_MAX) steer_v2_int =  STEER_V2_I_MAX;
					if (steer_v2_int < -STEER_V2_I_MAX) steer_v2_int = -STEER_V2_I_MAX;
				}
				const float i_term = steer_v2_int;

				float steer_f = heading_error * STEER_V2_KP * scale
				              + heading_rate  * STEER_V2_KD * d_scale
				              + i_term;
				/* MAX_STEER_TRIM as before, less while pulling away (STEER_LAUNCH_*). */
				const float steer_max = (accum_counts < (int32_t)(STEER_LAUNCH_MM / MM_PER_COUNT))
				                        ? STEER_LAUNCH_US : 220.0f;
				if (steer_f >  steer_max) steer_f =  steer_max;
				if (steer_f < -steer_max) steer_f = -steer_max;
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
				float STEER_KI   = 1.0f;//2.0f;
				float STEER_KD   = 4.0f;//5.0f;
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
					/* A sensing phase has taken over: fade the extra out by
					 * distance, over WATCH_FADE_SHARE of the way from where it
					 * took over to the arm point, so it is gone (and the phase
					 * runs as tuned) by then, without a step in the duty. A
					 * watch armed from the start takes over at 0 and never gets
					 * any. See WATCH_FADE_SHARE for why not with the command. */
					const float span = WATCH_FADE_SHARE * (float)(watch_rt.arm_counts - watch_rt.fade_counts0);
					const float done = (float)(accum_counts - watch_rt.fade_counts0);
					if (watch_rt.ff_fade_extra > 0.0f && span > 0.0f && done < span) {
						extra = watch_rt.ff_fade_extra * (1.0f - done / span);
					}
				} else if (sp > profile->straight_crawl) {
					extra = profile->straight_ff_extra * (sp - profile->straight_crawl);
				}
				ff += (int32_t)extra;
			}

			int32_t duty_l = ff + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
			int32_t duty_r = ff + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

			/* Kush (15-Oct): rear wheel sync, forward only (WHEEL_SYNC_* at the top). */
			if (!is_small_distance && dir_forward == 1 &&
			    (WHEEL_SYNC_KP > 0.0f || WHEEL_SYNC_KI > 0.0f)) {
				const float dv = (float)(left_delta - right_delta);   /* + = left faster */
				wheel_sync_int += dv * dt;
				float sy = WHEEL_SYNC_KP * dv + WHEEL_SYNC_KI * wheel_sync_int;
				if (sy > WHEEL_SYNC_MAX || sy < -WHEEL_SYNC_MAX) {
					const float lim = (sy > 0.0f) ? WHEEL_SYNC_MAX : -WHEEL_SYNC_MAX;
					if (WHEEL_SYNC_KI > 0.0f) wheel_sync_int = (lim - WHEEL_SYNC_KP * dv) / WHEEL_SYNC_KI;
					sy = lim;
				}
				if (duty_l > PWM_MAX) duty_l = PWM_MAX;
				if (duty_r > PWM_MAX) duty_r = PWM_MAX;
				const int32_t half = (int32_t)(0.5f * sy);
				duty_l -= half;
				duty_r += half;
				/* The slower wheel already flat out: take the rest off the faster one. */
				if (duty_r > PWM_MAX) { duty_l -= duty_r - PWM_MAX; duty_r = PWM_MAX; }
				if (duty_l > PWM_MAX) { duty_r -= duty_l - PWM_MAX; duty_l = PWM_MAX; }
			}

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
			float braking_lead = 2.9f;
			if(turn_left) braking_lead = 3.5f;
			else braking_lead = 2.9f;
			if (target_deg_total <= 45){
				if (turn_left)
					braking_lead = (dir_forward == 1) ? 5.3f : 4.4f;
				else
					braking_lead = (dir_forward == 1) ? 3.7f : 3.3f;
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
					target_base_speed = (float)(dir_forward * 40.0f); /* Crawl speed */
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
