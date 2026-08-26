/*
 * calib.h -- physical constants and per-robot calibration
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CALIB_H
#define CALIB_H

/* Encoder counts per full revolution of the WHEEL, x4 mode.
   MEASURED: 1 m hand-push gave 7318 counts, corroborated by two driven runs
   (200 mm -> 210 mm, 500 mm -> 485 mm). The nameplate 1560 was wrong.      */
#define COUNTS_PER_REV      1494.0f

/* Wheel diameter in mm, measured under load                                */
#define WHEEL_DIA_MM        65.0f

#define MM_PER_COUNT        (3.14159265f * WHEEL_DIA_MM / COUNTS_PER_REV)

/* Servo pulse widths in microseconds.  VERIFIED ON HARDWARE:
     - 1500 is true mechanical centre. Confirmed three ways: the vendor
       firmware initialises both TIM12 channels to 1500; driven straight runs
       track true; a hand-pushed roll at 1500 shows no curve.
     - LEFT saturates at 1000. Going to 900 produces no further wheel angle,
       so 1000 is the useful limit on that side.
     - RIGHT keeps gaining slightly past 2000, up to about 2100. Beyond that
       the extra wheel angle produces no extra turn (front tyres scrub).
   The sides are asymmetric because the linkage converts left horn rotation
   into wheel angle more efficiently than right. Mechanical, not fixable in
   firmware, and the reason four separate turn constants exist.             */
#define SERVO_CENTRE        1500
#define SERVO_LEFT          1000
#define SERVO_RIGHT         2100

/* Encoder counts to complete a 90 degree change of heading.
   Four separate values -- they are NOT symmetric. All verified on hardware.
   move_turn_deg() scales these linearly for other angles.                  */
#define TURN_COUNTS_FL      3350   /* forward-left  90 deg */
#define TURN_COUNTS_FR      4150   /* forward-right 90 deg */
#define TURN_COUNTS_BL      3300   /* reverse-left  90 deg */
#define TURN_COUNTS_BR      4100   /* reverse-right 90 deg */

/* Target speeds in ENCODER COUNTS PER 10 ms CONTROL TICK.
   A physical quantity, independent of PWM_MAX -- do NOT rescale these
   if you change the PWM period.                                            */
// Max 85 for left motor, 80 for right on my floor
#define SPEED_STRAIGHT      40
#define SPEED_TURN          50

/* VERIFIED ON HARDWARE -- DO NOT CHANGE.
   Both motors drive the car backwards on positive duty, so both are 1.
   The matching encoder sign fix lives in encoders.c: the RIGHT delta is
   negated there, the left is not. These four settings were established
   together by open-loop test and must be changed together if at all.       */
#define INVERT_LEFT         1
#define INVERT_RIGHT        1

/* --- safety limits, in 10 ms control ticks ---
   These exist because a stalled motor held at full duty is what tripped the
   driver/battery protection during bring-up.                               */
#define STALL_MIN_COUNTS    2       /* |delta| below this counts as stalled */
#define STALL_TICKS         100u    /* 1.0 s of no motion -> abort the move */
#define MOVE_TIMEOUT_TICKS  2000u   /* 20 s hard ceiling on any one move    */

#endif
