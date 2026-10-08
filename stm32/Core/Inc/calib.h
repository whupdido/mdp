/*
 * calib.h -- physical constants and per-robot calibration
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CALIB_H
#define CALIB_H

/* Encoder counts per full revolution of the WHEEL, x4 mode.
   07-Oct-2026, with the front weights on: 1515.3, from CAL_ODOMETER hand
   pushes (about 7420 counts per metre).
   Before the weights: a 1 m hand-push gave 7318 counts (1494.0), corroborated
   by two driven runs (200 mm -> 210 mm, 500 mm -> 485 mm). The nameplate 1560
   was wrong.                                                               */
#define COUNTS_PER_REV      1500.0f

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

/* How long a turn waits after commanding full lock before it drives, so the
   wheels are at the angle the radius was measured at. Weight on the front
   wheels can make the servo slower to get there. To check: run CAL_TURN with
   250 and again with 400. If the radii differ by more than their spread, the
   servo was not reaching lock in time, so keep the longer value.            */
#define SERVO_SETTLE_MS     250u

/* Steering trim while reversing straight (FW is untouched), in microseconds
   added to SERVO_CENTRE. Negative steers left. Reversing, caster pushes the
   front wheels off centre instead of centring them, and the car drifted to
   its right; this holds the wheels the other way for the whole move. Replaces
   the old reverse_bias of -20 in control.c, which faded out at low speed.
   Scale: full left lock is 500 us for roughly 27 deg at the wheels (the
   282 mm FL radius with a wheelbase of about 15 cm), so about 18 us per
   degree; -55 is about 3 deg.
   Tune with CAL_STRAIGHT: BW100's "side" should read about 0 (+ = left).
   Still drifting right: more negative. Now drifting left: less negative.    */
#define SERVO_REVERSE_TRIM_US   (-55)

/* --- Turn geometry, MEASURED on hardware ---
   Current values: tape measured 07-Oct-2026 with the front weights on, on
   the 30 degree turns the planner drives (CAL_TURN_* in main.c). The
   rear-axle midpoint was marked on the floor before and after a turn and the
   straight-line chord c measured (average of the runs), then
       R = c / (2 sin(theta/2)) = c / 0.5176   for theta = 30 deg.
                    chord mean     radius
        FR           182.0 mm      352
        FL           146.0 mm      282
        BR           184.6 mm      357
        BL           131.6 mm      254
   A 30 degree turn moves the car R/2 along its old heading and 0.134 R
   sideways. The radius is taken at 30 deg, the angle the planner assumes,
   so the planner's step matches the measured chord. If a turn's braking lead
   changes, re-measure that turn: the coast is part of the chord.

   Earlier, 25-Sep-2026, before the weights, 5 runs per case: each 90 degree
   turn was run from SW1 with
   TURN_TEST in main.c; the rear-axle midpoint was marked on the floor at start
   and finish and the straight-line chord c measured. Then
       R = c / (2 sin(theta/2)) = c / sqrt(2)   for theta = 90 deg.
   The gyro read within 0.5 deg of 90 on every run, which moves R by under
   0.5 %. The "+-" is half the observed run-to-run range; the tape was read to
   5 mm, so no radius here is resolved better than about +-2 mm.

   The motion controller does NOT read these. move_turn_deg() closes on the
   gyro and stops at the commanded angle whatever radius the car traces. The
   PLANNER does: obstacle_nav.c uses them, because after a 90 degree turn the
   car has moved one radius forward (or back) and one radius sideways.

                    chord mean     radius         previous (05-Sep, encoders)
        FL           385 mm        272 +-2         277 +-13
        FR           518 mm        366 +-4         365 +-2
        BL           395 mm        279 +-4         281 +-14
        BR           523 mm        370 +-2         383 +-2

   The left-turn scatter seen on 05-Sep (+-13/14 mm) did not show up here: the
   left turns now repeat as well as the right ones. BR is the only case whose
   value moved by more than the spread (-13 mm).                           */
#define TURN_RADIUS_FL_MM   282     /* 07-Oct, 30 deg chord 146.0 mm (was 262) */
#define TURN_RADIUS_FR_MM   352     /* 07-Oct, 30 deg chord 182.0 mm (was 360) */
#define TURN_RADIUS_BL_MM   254     /* 07-Oct, 30 deg chord 131.6 mm (was 260) */
#define TURN_RADIUS_BR_MM   357     /* 07-Oct, 30 deg chord 184.6 mm (was 361) */

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
