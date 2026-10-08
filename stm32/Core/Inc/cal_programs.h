/*
 * cal_programs.h -- SW1-driven calibration programs.
 *
 * Pick one with CAL_PROGRAM in main.c and flash. Hold SW1 for 2 s to lock the
 * gyro bias, then tap SW1 to step through it. Results go to the OLED and to
 * USART3. The turn radius program lives in turn_test.c (CAL_TURN).
 */

#ifndef CAL_PROGRAMS_H
#define CAL_PROGRAMS_H

#include <stdint.h>

#define CAL_OFF        0   /* normal runs                                   */
#define CAL_STRAIGHT   1   /* FW/BW stop distance and straightness          */
#define CAL_ODOMETER   2   /* push by hand: live distance (COUNTS_PER_REV)  */
#define CAL_TURN       3   /* 30 degree turns, all four kinds (turn_test.c) */
#define CAL_TURN_FR    4   /* 30 degree turns, one kind only (turn_test.c)  */
#define CAL_TURN_FL    5
#define CAL_TURN_BR    6
#define CAL_TURN_BL    7
#define CAL_SIX_FR     8   /* six 30 degree turns of one kind in a row      */
#define CAL_SIX_FL     9
#define CAL_SIX_BR    10
#define CAL_SIX_BL    11
#define CAL_SEQ       12   /* turns then a straight, as in a run: tilt check */
#define CAL_TRACE     13   /* one straight traced tick by tick: why it turns */

/* Usable in #if. The kind is 0 FR, 1 FL, 2 BR, 3 BL: the order turn_test.c
   runs them in, and TT_FR..TT_BL in turn_test.h. */
#define CAL_IS_TURN_ONE(p)  ((p) >= CAL_TURN_FR && (p) <= CAL_TURN_BL)
#define CAL_IS_SIX(p)       ((p) >= CAL_SIX_FR  && (p) <= CAL_SIX_BL)
#define CAL_TURN_KIND(p)    (CAL_IS_SIX(p) ? (p) - CAL_SIX_FR : (p) - CAL_TURN_FR)

/* CAL_STRAIGHT */
void cal_straight_show_idle(void);
void cal_straight_step(void);

/* CAL_ODOMETER */
void cal_odometer_start(void);   /* free-wheel the motors, centre the steering, zero */
void cal_odometer_zero(void);
void cal_odometer_poll(void);    /* call every main-loop pass; redraws every 200 ms   */

/* CAL_SIX_*: kind 0 FR, 1 FL, 2 BR, 3 BL */
void cal_six_show_idle(uint8_t kind);
void cal_six_step(uint8_t kind);

/* CAL_SEQ: FL030, FR030, FW100 (SEQ_MOVES in cal_programs.c), 5 runs */
void cal_seq_show_idle(void);
void cal_seq_step(void);

/* CAL_TRACE: FW100 watched from the control loop, 3 pages per run */
void cal_trace_show_idle(void);
void cal_trace_step(void);

#endif /* CAL_PROGRAMS_H */
