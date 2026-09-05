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
} move_result_t;

void    control_init(void);
void    control_tick(void);        /* called from TIM6 ISR */
uint8_t motion_busy(void);
void    motion_stop(void);
move_result_t motion_result(void); /* valid once motion_busy() is false */


uint8_t move_straight_mm(int32_t mm);

/* Raw form: counts is the arc length in encoder counts. Used for calibration. */
void move_turn(int8_t left, int8_t forward, int32_t counts);

/* Preferred form: degrees is 1..360, closed on the gyro.
   NOTE: this no longer uses the TURN_COUNTS_* constants in calib.h at all. */
uint8_t move_turn_deg(int8_t left, int8_t forward, int32_t degrees);

/* In-place pivot turn (counter-rotating rear wheels) */
void move_pivot_deg(int8_t left, int32_t degrees);

uint8_t move_kturn_90(int8_t left);

#endif
