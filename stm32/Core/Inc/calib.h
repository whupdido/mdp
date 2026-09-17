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

/* --- Turn geometry, MEASURED on hardware, 5 runs per case ---
   (the test that produced these is archived in tools/turn_test/)
   Radius = arc / gyro-measured heading change, including the coast after the
   controller stops steering. The "+-" is half the observed run-to-run range.

   The controller does NOT read these. move_turn_deg() closes on the gyro and
   stops at the commanded angle whatever radius the car traces. They are here
   because the RADIUS is what displaces the car on the grid: after a 90 degree
   turn the car has moved one radius forward (or back) and one radius sideways,
   so the path planner needs them even though the controller does not.

                    before decel      after decel
        FL           317 +-5           277 +-13      -12.6 %
        FR           413 +-3           365 +-2       -11.6 %
        BL           312 +-4           281 +-14       -9.9 %
        BR           421 +-3           383 +-2        -9.0 %

   Two things to read off that table. Every radius tightened by about 10 %: the
   speed_ratio scaling on the turn feedforward in control.c is meant to hold the
   radius constant through the ramp and it does not fully manage it. And the
   LEFT turns lost their repeatability, going from about 1.5 % run-to-run to
   about 5 %, while the right turns stayed at 0.5 %. The left spread is the
   problem worth chasing: 26 mm of scatter per turn accumulates across a route
   in a way a biased but repeatable radius does not.                        */
#define TURN_RADIUS_FL_MM   277     /* +-13 mm -- NOT repeatable, see above */
#define TURN_RADIUS_FR_MM   365     /* +-2 mm                               */
#define TURN_RADIUS_BL_MM   281     /* +-14 mm -- NOT repeatable, see above */
#define TURN_RADIUS_BR_MM   383     /* +-2 mm                               */

/* Target speeds in ENCODER COUNTS PER 10 ms CONTROL TICK.
   A physical quantity, independent of PWM_MAX -- do NOT rescale these
   if you change the PWM period.                                            */
// Max 85 for left motor, 80 for right on my floor
#define SPEED_STRAIGHT      60
#define SPEED_TURN          50

/* VERIFIED ON HARDWARE -- DO NOT CHANGE.
   Both motors drive the car backwards on positive duty, so both are 1.
   The matching encoder sign fix lives in encoders.c: the RIGHT delta is
   negated there, the left is not. These four settings were established
   together by open-loop test and must be changed together if at all.       */
#define INVERT_LEFT         1
#define INVERT_RIGHT        1

/* --- Sharp analog IR distance model ---
   Three parameters, not two:      V = m/(d + k) + b
   inverted at run time to         d = m/(V - b) - k.

   The k term is the point of it. The ideal 1/d law assumes the emitter and the
   detector sit at the same place as the point you are measuring from, and they
   do not: there is a fixed optical offset between the sensor's baseline and its
   front face. Forcing k to 0 makes a two parameter fit bend to cover that
   offset, and it pays for the bend at the near end of the range, which is
   exactly where the readings are being used to avoid hitting things.

   These are the values measured on this car. The on-car calibration UI that
   produced them has been removed, so they are now fixed here.               */
#define IR_LEFT_M           19.22f
#define IR_LEFT_B           0.210f
#define IR_LEFT_K           0.0f
#define IR_RIGHT_M          20.80f
#define IR_RIGHT_B          0.073f
#define IR_RIGHT_K          0.0f

/* Range the readings are trusted over. Below IR_MIN_CM the response curve
   folds back on itself, so a 5 cm target reads the same as a 20 cm one.
   IR_MAX_CM is deliberately just past the top of the calibration sweep: past
   that the curve is too flat for the reading to mean much, so it saturates
   instead of pretending to resolve 60 from 75.                              */
#define IR_MIN_CM           10.0f
#define IR_MAX_CM           60.0f

/* --- safety limits, in 10 ms control ticks ---
   These exist because a stalled motor held at full duty is what tripped the
   driver/battery protection during bring-up.                               */
#define STALL_MIN_COUNTS    2       /* |delta| below this counts as stalled */
#define STALL_TICKS         100u    /* 1.0 s of no motion -> abort the move */
#define MOVE_TIMEOUT_TICKS  2000u   /* 20 s hard ceiling on any one move    */

#endif
