/*
 * turn_test.h -- SW1-stepped turning radius measurement.
 *
 * One SW1 tap = one 90 degree turn, then the car stops and shows the result.
 * Order: 5 x FR, 5 x FL, 5 x BR, 5 x BL, then a summary.
 *
 * Enabled with TURN_TEST in main.c. The car MOVES on every tap.
 */

#ifndef TURN_TEST_H
#define TURN_TEST_H

/** @brief Draw the "next up" screen. Call once at boot and after a gyro calib. */
void turn_test_show_idle(void);

/** @brief Advance one step: run the next turn, or page through the summary. */
void turn_test_step(void);

#endif /* TURN_TEST_H */
