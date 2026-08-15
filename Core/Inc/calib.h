/*
 * calib.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CALIB_H
#define CALIB_H

/* Encoder: counts per full revolution of the WHEEL, in x4 mode.
   13 PPR hall encoder x 30:1 gearbox x 4 = 1560 (verify!)      */
#define COUNTS_PER_REV      1560.0f

/* Wheel diameter in mm, measured under load                    */
#define WHEEL_DIA_MM        65.0f

#define MM_PER_COUNT        (3.14159265f * WHEEL_DIA_MM / COUNTS_PER_REV)

/* Servo pulse widths in microseconds                           */
#define SERVO_CENTRE        1500
#define SERVO_LEFT          1100
#define SERVO_RIGHT         1900

/* Encoder counts to complete a 90 degree change of heading.
   Four separate values — they are NOT symmetric.               */
#define TURN_COUNTS_FL      2400   /* forward-left  90 deg */
#define TURN_COUNTS_FR      2400   /* forward-right 90 deg */
#define TURN_COUNTS_BL      2400   /* reverse-left  90 deg */
#define TURN_COUNTS_BR      2400   /* reverse-right 90 deg */

/* Speeds in encoder counts per 10 ms control tick              */
#define SPEED_STRAIGHT      20
#define SPEED_TURN          14

/* Set to 1 if a motor's positive duty drives it backwards      */
#define INVERT_LEFT         0
#define INVERT_RIGHT        1

#endif
