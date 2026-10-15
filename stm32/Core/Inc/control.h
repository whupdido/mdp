/*
 * control.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CONTROL_H
#define CONTROL_H
#include <stdint.h>

/* How the last move ended. Lets the caller (and therefore the Pi) tell a
   completed move from one the safety logic aborted -- previously both
   looked identical, so a robot wedged against an obstacle reported success
   and the Pi kept dead-reckoning from a wrong position. */
typedef enum {
    MOVE_NONE = 0,   /* nothing has run yet                        */
    MOVE_DONE,       /* reached the target distance normally       */
    MOVE_STALL,      /* both wheels stopped turning -> aborted     */
    MOVE_TIMEOUT,    /* exceeded MOVE_TIMEOUT_TICKS -> aborted     */
    MOVE_ABORT,       /* cancelled by motion_stop() / STOP command  */
    /* Zhenxi: the IR collision stop in move_straight_mm used to record
       MOVE_DONE, so a move that stopped 15 cm short replied DONE and the
       tablet drew the robot where the planner expected it, not where it
       was. Its own result lets command.c say BLOCKED instead. */
    MOVE_BLOCKED,    /* stopped short: IR saw an obstacle -> aborted */
} move_result_t;

void    control_init(void);
void    control_tick(void);        /* called from TIM6 ISR */
uint8_t motion_busy(void);
void    motion_stop(void);
uint8_t motion_abort_requested(void);
void    motion_abort_clear(void);
move_result_t motion_result(void); /* valid once motion_busy() is false */
float   motion_yaw_deg(void);        /* ISR-integrated heading, deg, CCW + */

/* Where the rear-axle centre has moved, in mm, integrated by the 100 Hz ISR
   in every mode (idle too, so it catches the coast after a move): distance
   from the mean of the two rear-wheel encoders, heading from the gyro. The
   origin is arbitrary and x points along yaw 0, so take differences. Read
   only by the turn test; nothing in the motion control uses it. */
void    motion_odometry_mm(float *x_mm, float *y_mm);

/* One 10 ms tick of a straight move, as the controller saw it. */
typedef struct {
    float   ramp;           /* commanded speed, counts per tick, signed       */
    int32_t left, right;    /* encoder counts this tick, signed               */
    int32_t duty_l, duty_r; /* motor duty sent (PWM_MAX = full), before clamp */
    float   i_l, i_r;       /* the integral part of those duties              */
    float   head_deg;       /* heading minus the one locked at the start,
                               + = turned left (CCW)                          */
    int16_t steer_us;       /* servo command minus SERVO_CENTRE, + = right    */
    uint8_t braking;        /* slowing down for the target                    */
} straight_tick_t;

/* Kush: test hook for CAL_TRACE (cal_programs.c). When set, the 100 Hz
   interrupt calls it on every tick of a straight move, after the motors and
   servo are set. Read-only: nothing it does goes back into the control.
   NULL in normal runs, which costs one pointer check per tick. */
extern void (*volatile straight_tick_hook)(const straight_tick_t *t);

/* Kush (09-Oct): called by the 100 Hz interrupt on every tick, idle or not,
   before the move logic. NULL in normal runs. CAL_TASK2 (cal_programs.c)
   points it at an SW1 check that calls motion_stop(), so SW1 stops a run
   with no Pi; it runs through every HAL_Delay(), which no polling loop does.
   Interrupt context: read a pin, count, call motion_stop(), nothing slower. */
extern void (*volatile control_tick_hook)(void);


uint8_t move_straight_mm(int32_t mm);

/* ------------------------------------------------------------------------- */
/* Kush (09-Oct): speed profiles                                              */
/* ------------------------------------------------------------------------- */
/* The speeds the controller drives at, chosen per run. The PID itself (gains,
   feedforward, steering) is the same in every profile.
     MOTION_PROFILE_TASK1  exactly the settings that ran Task 1 (commit
                           570c4d6): cruise 60, crawl 15, brake point from the
                           commanded speed. The default; the Pi's FW/BW/FL/...
                           commands always run on it.
     MOTION_PROFILE_TASK2  task_2() switches to it for the run and back after.
                           Since 11-Oct (speed step 5): cruise 80, crawl 15,
                           brake point from the measured speed. Tested before
                           that with cruise 60, crawl 15, brake point from the
                           commanded speed (the same as Task 1). See calib.h.
   Every turn runs at SPEED_TURN in both profiles, as in the original code
   (with TURN_FAST in calib.h on, faster in its middle, since 11-Oct). A
   move_straight_watch() always uses SPEED_CRAWL and the measured brake point
   (control.c, straight_stop_counts). */
typedef struct {
    float   straight_cruise;   /* commanded counts/tick on straights            */
    float   straight_crawl;    /* commanded counts/tick when the brake goes on  */
    uint8_t brake_measured;    /* 1: start slowing from the MEASURED speed      */
    float   straight_ff_extra; /* extra feedforward, duty per count/tick of
                                  command ABOVE the crawl, forward straights
                                  only (0 = none). See control.c.              */
} motion_profile_t;

extern const motion_profile_t MOTION_PROFILE_TASK1;
extern const motion_profile_t MOTION_PROFILE_TASK2;
void motion_set_profile(const motion_profile_t *p);   /* call between moves only */
const motion_profile_t *motion_get_profile(void);

/* move_straight_mm(), plus how far the wheels actually took the car (mm,
   signed like mm, coast included). Task 2 dead-reckons with this. */
int32_t move_straight_measured(int32_t mm);

/* ------------------------------------------------------------------------- */
/* Kush (09-Oct): straight moves that end on a sensor                         */
/* ------------------------------------------------------------------------- */
/* Drives forward up to max_mm while the 100 Hz tick watches one sensor. When
   the condition holds for WATCH_CONFIRM_TICKS ticks in a row the move is
   retargeted to stop after_mm further on (or later, if the car needs more room
   than that to slow to its crawl, so it always stops the normal way; not with
   stop_at_trigger). Replaces the old stop / read IR / move 50-200 mm / repeat
   loops.                                                                      */
typedef enum {
    WATCH_IR_ABOVE = 1,   /* IR reads >= thresh: the face beside the car ended  */
    WATCH_IR_BELOW,       /* IR reads <= thresh: something came into view       */
    WATCH_IR_PASS,        /* IR saw something (<= thresh), then lost it (>=)    */
    WATCH_US_BELOW        /* ultrasonic reads <= thresh                         */
} watch_kind_t;

typedef struct {
    /* --- set by the caller --- */
    watch_kind_t kind;
    uint8_t  ir_side;      /* IR_SIDE_LEFT / IR_SIDE_RIGHT (sensors.h)            */
    float    thresh_cm;
    int32_t  arm_mm;       /* ignore the sensor for the first arm_mm of the move  */
    int32_t  after_mm;     /* keep going at least this far past the trigger       */
    float    sense_speed;  /* commanded counts/tick while armed (0 = cruise). The
                              car slows to it before the arm point.               */
    uint8_t  log_face;     /* record (distance, IR) while the IR sees something,
                              for watch_face_fit()                                */
    int32_t  min_seen_mm;  /* WATCH_IR_PASS: something seen over less than this
                              much driving, then lost, was noise: forget it and
                              keep looking (0 = off, as before)                   */
    uint8_t  stop_at_trigger; /* Kush (15-Oct): 1 = keep the sense speed for
                              after_mm past the trigger and brake there, instead
                              of slowing to the crawl first (which takes ~15 cm
                              and most of a second from a sense speed of 70).
                              0 = the normal stop, as before                      */
    /* --- filled in by the move --- */
    uint8_t  triggered;
    uint8_t  seen;         /* WATCH_IR_PASS: it did see something                 */
    int32_t  seen_mm;      /* where it was first seen                             */
    int32_t  trigger_mm;   /* where the condition was confirmed                   */
    float    trigger_speed_mm_tick; /* measured speed then, mm per 10 ms tick     */
    int32_t  driven_mm;    /* encoder distance actually covered, coast included   */
    uint8_t  blips;        /* WATCH_IR_PASS: flickers min_seen_mm threw away      */
} watch_t;

#define WATCH_CONFIRM_TICKS  2u

/* Returns what move_straight_mm returned (0 = aborted / collision stop). */
uint8_t move_straight_watch(int32_t max_mm, watch_t *w);

/* Straight-line fit of the IR distance against distance driven, over the
   samples the last move_straight_watch() logged (log_face = 1), with the
   first and last `trim` fraction dropped (edge-of-face readings). slope is
   cm of IR per mm driven: + means the face is getting further away.
   Returns 0 if fewer than min_n samples or less than min_span_mm remain. */
uint8_t watch_face_fit(float trim, uint16_t min_n, float min_span_mm,
                       float *slope_cm_per_mm, uint16_t *n_used, float *span_mm);

/* Kush (14-Oct): median IR distance over the same samples, trimmed the same
   way: how far away the face was. Returns 0 if fewer than min_n remain. */
uint8_t watch_face_level(float trim, uint16_t min_n, float *level_cm, uint16_t *n_used);

/* Raw form: counts is the arc length in encoder counts. Used for calibration. */
void move_turn(int8_t left, int8_t forward, int32_t counts);

/* Preferred form: degrees is 1..360, closed on the gyro.
   NOTE: this no longer uses the TURN_COUNTS_* constants in calib.h at all. */
uint8_t move_turn_deg(int8_t left, int8_t forward, int32_t degrees);

/* In-place pivot turn (counter-rotating rear wheels) */
void move_pivot_deg(int8_t left, int32_t degrees);

uint8_t move_kturn_90(int8_t left);

#endif
