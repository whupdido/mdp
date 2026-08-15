/*
 * calib.h -- physical constants and per-robot calibration
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CALIB_H
#define CALIB_H

/* Encoder: counts per full revolution of the WHEEL, in x4 mode.
   13 PPR hall encoder x 30:1 gearbox x 4 = 1560 (still unverified --
   push the car exactly 1 m by hand and read enc_left_total to confirm) */
#define COUNTS_PER_REV      1560.0f

/* Wheel diameter in mm, measured under load                    */
#define WHEEL_DIA_MM        65.0f

#define MM_PER_COUNT        (3.14159265f * WHEEL_DIA_MM / COUNTS_PER_REV)

/* Servo pulse widths in microseconds                           */
#define SERVO_CENTRE        1500
#define SERVO_LEFT          1100
#define SERVO_RIGHT         1900

/* Encoder counts to complete a 90 degree change of heading.
   Four separate values -- they are NOT symmetric.               */
#define TURN_COUNTS_FL      2400   /* forward-left  90 deg */
#define TURN_COUNTS_FR      2400   /* forward-right 90 deg */
#define TURN_COUNTS_BL      2400   /* reverse-left  90 deg */
#define TURN_COUNTS_BR      2400   /* reverse-right 90 deg */

/* Target speeds in ENCODER COUNTS PER 10 ms CONTROL TICK.
   These are a physical quantity and are independent of PWM_MAX --
   do NOT rescale them if you change the PWM period.            */
#define SPEED_STRAIGHT      20
#define SPEED_TURN          14

/* Set to 1 if a motor's positive duty drives it backwards.
   Verify with the SELFTEST block in main.c before trusting the
   closed loop: a wrong sign here makes the PI controller wind up
   and drive the wheel to full duty in the wrong direction.     */
#define INVERT_LEFT         1     /* was 0 */
#define INVERT_RIGHT        1

/* --- safety limits, in 10 ms control ticks ---
   These exist because a stalled motor held at full duty is what
   tripped the driver/battery protection during bring-up.        */
#define STALL_MIN_COUNTS    2       /* |delta| below this counts as stalled */
#define STALL_TICKS         100u    /* 1.0 s of no motion -> abort the move */
#define MOVE_TIMEOUT_TICKS  2000u   /* 20 s hard ceiling on any one move    */

#endif
