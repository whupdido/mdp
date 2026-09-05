/*
 * turn_test.h -- on-car measurement of the four 90 degree turn geometries.
 */

#ifndef TURN_TEST_H
#define TURN_TEST_H

/**
 * @brief Measure the turning radius and the achieved angle for forward-left,
 *        forward-right, reverse-left and reverse-right 90 degree turns.
 *
 * Driven entirely by SW1 and the OLED. Repeats each case so you get a spread
 * rather than a single sample, calibrates the gyro bias first, and reports the
 * radius in mm, which for a 90 degree turn is also the forward and the lateral
 * grid offset.
 *
 * BLOCKING and the car MOVES. Clear floor space first.
 */
void turn_test_run(void);

#endif /* TURN_TEST_H */
