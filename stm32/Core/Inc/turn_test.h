/*
 * turn_test.h -- SW1-stepped turning radius measurement.
 *
 * One SW1 tap = one 30 degree turn (TT_ANGLE in turn_test.c; 90 gives the old
 * test), then the car stops and shows its radius two ways and its angle.
 * TT_ALL: 5 x FR, 5 x FL, 5 x BR, 5 x BL, then four summary pages (radius
 * from the arc, radius from the chord, angle, braking-lead change).
 * One kind (TT_FR .. TT_BL): 5 of that turn, then one page with all four.
 *
 * Enabled with CAL_PROGRAM = CAL_TURN or CAL_TURN_FR .. CAL_TURN_BL in main.c.
 * The car MOVES on every tap.
 */

#ifndef TURN_TEST_H
#define TURN_TEST_H

#include <stdint.h>

/* Which turns to run: all four, or one kind. */
#define TT_ALL   0xFFu
#define TT_FR    0u
#define TT_FL    1u
#define TT_BR    2u
#define TT_BL    3u

/** @brief Draw the "next up" screen. Call once at boot and after a gyro calib. */
void turn_test_show_idle(uint8_t only);

/** @brief Advance one step: run the next turn, or page through the summary. */
void turn_test_step(uint8_t only);

#endif /* TURN_TEST_H */
