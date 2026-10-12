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
   if you change the PWM period.

   Kush (09-Oct): these are COMMANDS, not what the wheels do. The speed loop is
   close to proportional only (feedforward 20 duty per count/tick against the
   ~190 the motors need), so the wheels settle at about 45 % of the command:
   CAL_TRACE measured 27 counts/tick (0.37 m/s) for a command of 60. That is
   why commanding 80 does not saturate a motor that tops out at ~80.

   Two sets, see motion_profile_t in control.h:
     Task 1 (the Pi's FW/BW/FL/FR/BL/BR): SPEED_STRAIGHT / SPEED_CRAWL /
       SPEED_TURN, exactly what ran Task 1 on 09-Oct (commit 570c4d6). WRT4's
       80 / 50 from 0c37bfe are not used for Task 1 any more: a crawl of 50
       makes the car brake from ~4x the speed BRAKE_LEAD_COUNTS was set at,
       so every move ran long.
     Task 2 (task_2() only): T2_*, the settings Task 2 was tested with on the
       course before 0c37bfe (570c4d6 / 4879da0): cruise 60, crawl 15, the
       original brake point, i.e. the same as Task 1. 0c37bfe (WRT4, 09-Oct
       12:32) raised them to 80 / 50, and the faster Task 2 straights tried
       later on 09-Oct went further still; with either, the car finished the
       obstacle 1 dodge much closer to obstacle 2 than in the tested runs.
       Only the sensing phases after obstacle 2 (move_straight_watch) use the
       measured brake point their stops were tuned with. Turns are SPEED_TURN
       in both.
       Kush (11-Oct, speed step 5): Task 2 cruise 60 -> 80, crawl kept at 15,
       and every Task 2 straight now starts slowing from the measured wheel
       speed (MOTION_PROFILE_TASK2.brake_measured = 1 in control.c), as the
       sensing phases already did. The 09-Oct 80 kept the original brake
       point, which from 80 left ~20 cm of crawl, so 0c37bfe raised the crawl
       to 50 and the car then stopped from ~20 counts/tick and ran long. Here
       the brake goes on at the same crawl as the tested runs, the crawl is cut
       to a few cm, and the stop is the same. To go back: 60 here and
       brake_measured 0u in control.c.                                       */
// Max 85 for left motor, 80 for right on my floor (full PWM, see the note above)
#define SPEED_STRAIGHT      60      /* Task 1 cruise                             */
#define SPEED_CRAWL         15      /* commanded speed the brake goes on at      */
#define SPEED_TURN          50      /* every turn, Task 1 and Task 2             */

/* Kush (11-Oct, speed step 6): faster turns. TURN_FAST 1 = on, 0 = every turn
   exactly as before.
   When on, a turn speeds up twice as fast (TURN_FAST_RAMP_UP command steps a
   tick instead of 1.2) to SPEED_TURN_FAST, and is back down at SPEED_TURN by
   TURN_FAST_HANDOVER_DEG before the point where every turn starts slowing
   (20 deg from the end of a turn over 45 deg, 6 deg from the end of a smaller
   one). From there on the turn runs exactly as before, and that last part is
   what sets the angle it ends at and the coast that is part of the measured
   radii (TURN_RADIUS_*), so those still hold. A 90 is fast for its first
   ~50 deg, a 45 for its first ~19, a 30 for its first ~4; anything under
   26 deg (the arrow-scan retries, the parking shift) is unchanged. Applies
   to every move_turn_deg(), Task 1 included.
   If the turns start ending at a different angle, set TURN_FAST to 0 first. */
#ifndef TURN_FAST
#define TURN_FAST               1
#endif
#define SPEED_TURN_FAST         65      /* commanded, middle of a turn               */
#define TURN_FAST_RAMP_UP       2.4f    /* command steps per tick speeding up (1.2)  */
#define TURN_FAST_HANDOVER_DEG  20.0f   /* back at SPEED_TURN this far before slowing */

/* Kush (12-Oct): three more speed-ups, each with its own switch: 1 = on,
   0 = exactly as before. None changes the path or how a move ends.
     STRAIGHT_FAST_START  a straight speeds up STRAIGHT_RAMP_UP_X times as fast
                          as before; slowing down, the brake point and the
                          stop are unchanged. ~1.4 s a Task 2 run, simulated.
                          If a straight wheelspins as it starts, set it to 0.
     STRAIGHT_SETTLE_FAST the pause after a straight, before the next move,
                          50 ms instead of 100 (the car stops from its crawl,
                          so there is next to no coast left). After a turn it
                          stays 100 ms: the steering is coming back from full
                          lock then, and the next move's timing was tested
                          with that. ~0.6 s a run, simulated.
     T2_FEWER_PAUSES      task_2(): no steering pause before TURN 1 (the arrow
                          read always ends on a straight, so the wheels are
                          centred) or TURN 3 (same lock as TURN 2), nor the two
                          before the parking reads (standing still, wheels
                          centred). The pauses before a turn the opposite way
                          stay. ~0.6 s a run, simulated.
   Together, simulated over 200 layouts: same obstacle 1 detection, no
   collisions, parked within ~1 cm of before on average. Applies to Task 1
   too (the first two). */
#ifndef STRAIGHT_FAST_START
#define STRAIGHT_FAST_START     1
#endif
#define STRAIGHT_RAMP_UP_X      2.0f
#ifndef STRAIGHT_SETTLE_FAST
#define STRAIGHT_SETTLE_FAST    1
#endif
#ifndef T2_FEWER_PAUSES
#define T2_FEWER_PAUSES         1
#endif

#define T2_SPEED_STRAIGHT   80      /* Task 2 cruise (60 as tested; 80 since 11-Oct) */
#define T2_SPEED_CRAWL      15      /* Task 2 crawl, as tested (pre-0c37bfe)     */
/* Extra feedforward for Task 2's forward straights, in duty per count/tick of
   command above the crawl (see MODE_STRAIGHT in control.c). 0 = off, as in
   the original. 250 made the wheels really do the command (0.82 m/s at 60
   instead of 0.37 m/s); reverted on 09-Oct, see above. Do not turn it back on
   without checking where Task 2's straights stop on the floor.             */
#define T2_STRAIGHT_FF_EXTRA 0

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
   The calibration sweep went to about 60 cm; past that the curve is flat, so
   a reading of 60-80 cm only says roughly how far, not exactly.
   Kush (11-Oct): IR_MAX_CM 60 -> 80 (the sensor's rated range) so the Task 2
   search for obstacle 1 on the way back can tell "something at 60-70 cm"
   from open space (obstacle_nav.c, T2_OBS1_FAR_CM). Every other IR check
   switches below 60 cm, so a reading above 60 meant the same to them before
   (60, clamped) as it does now (60-80).                                      */
#define IR_MIN_CM           10.0f
#define IR_MAX_CM           80.0f

/* --- safety limits, in 10 ms control ticks ---
   These exist because a stalled motor held at full duty is what tripped the
   driver/battery protection during bring-up.                               */
#define STALL_MIN_COUNTS    2       /* |delta| below this counts as stalled */
#define STALL_TICKS         100u    /* 1.0 s of no motion -> abort the move */
#define MOVE_TIMEOUT_TICKS  2000u   /* 20 s hard ceiling on any one move    */

#endif
