#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Kush (09-Oct): all sensing now runs off the 100 Hz TIM6 tick.
 *
 * sensors_tick() is called from control_tick() every 10 ms:
 *   - IR: both Sharp sensors are read (each an average of IR_OVERSAMPLE DMA
 *     samples), converted to cm, and passed through a 5-tick median, so a
 *     straight move can watch them while it drives instead of stopping to
 *     look (see move_straight_watch() in control.h).
 *   - Ultrasonic: the HC-SR04 is pinged every US_PING_TICKS ticks. The TIM8
 *     capture interrupt turns each echo into a distance. Main-loop code no
 *     longer triggers it: it reads the background value instead.
 * ------------------------------------------------------------------------- */

/* Called from control_tick() (TIM6 ISR, 100 Hz). Nothing else should call it. */
void sensors_tick(void);

/* --- Ultrasonic (HC-SR04: TRIG on PC8, ECHO on TIM8 CH2) --- */

/* Latest valid reading, cm. Kept for old code; prefer the functions below,
   which know whether the value is fresh. */
extern volatile float ultrasonic_distance_cm;

/* Fires one ping. Called by sensors_tick() only. */
void trigger_ultrasonic(void);

/* Count of completed valid readings since boot (+1 per echo). */
uint32_t ultrasonic_seq(void);

/* Milliseconds since the latest valid reading (large if there has been none). */
uint32_t ultrasonic_age_ms(void);

/* Median of the valid readings from the last ~250 ms (up to 3; the nearer of
   two), or -1 if there are none. Does not wait. */
float ultrasonic_median_cm(void);

/* Waits for `new_readings` fresh echoes (about 60 ms each), then returns
   ultrasonic_median_cm(). Gives up after timeout_ms and returns whatever is
   fresh, or -1. Keeps the UART polled while it waits, so STOP still works. */
float ultrasonic_read_cm(uint8_t new_readings, uint32_t timeout_ms);

/* Kush (09-Oct): one ping taken as the original code took its readings: the
   background pings pause, US_QUIET_MS of silence (60 ms since 11-Oct, was
   100), one ping, up to 60 ms for the echo. ~65-120 ms. Returns cm, or -1 for
   no echo. Polls the UART while it waits. For the distances the Task 2
   approaches drive on. */
float ultrasonic_quiet_ping_cm(void);

/**
 * @brief Checks if the forward path is blocked using the HC-SR04.
 *        Two background readings in a row under 5 cm, the latest fresh.
 * @return 1 if collision is imminent, 0 if path is clear.
 */
uint8_t check_front_collision(void);

/* --- Sharp IR --- */

/* Index of each sensor, also the DMA rank order (rank 1 = PA2 = right). */
#define IR_SIDE_RIGHT   0u
#define IR_SIDE_LEFT    1u

/**
 * @brief Call this ONCE before your main loop starts to kick off the background
 *        DMA (and, since 09-Oct, to set up the ultrasonic capture and the cycle
 *        counter used for its trigger pulse).
 */
void ir_sensors_init(void);

/**
 * @brief Reads the left Sharp IR sensor via ADC, right now.
 * @return Distance in cm (average of IR_OVERSAMPLE samples, one instant).
 */
float get_sharp_ir_left_cm(void);

/**
 * @brief Reads the right Sharp IR sensor via ADC, right now.
 * @return Distance in cm.
 */
float get_sharp_ir_right_cm(void);

/* Median of the last 5 ticks (50 ms) of one sensor, as the 100 Hz tick sees
   it. This is what the Task 2 sensing uses. side = IR_SIDE_LEFT / _RIGHT. */
float ir_filtered_cm(uint8_t side);

/* Kush (14-Oct): the voltage one sensor gives right now (the mean of its
   IR_OVERSAMPLE DMA samples), before the conversion to cm. For CAL_IR. */
float ir_voltage(uint8_t side);

/**
 * @brief Formats the voltages into strings for your OLED screen.
 */
void display_ir_voltages_oled(void);

/**
 * @brief Infinite loop to test the HC-SR04 and display on the OLED.
 *        WARNING: This is a blocking loop. Call this in main() for testing only.
 */
void test_ultrasonic_oled(void);

/**
 * @brief Display both IR and Ultrasonic
 */
void display_both_sensors_oled(void);

extern volatile uint8_t image_found;

/**
 * @brief Stop and re-arm the ADC/DMA. Returns 1 if the restart took.
 */
uint8_t ir_sensors_restart(void);

#endif /* SENSORS_H */
