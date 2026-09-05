#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

/* Global variables updated in the background by hardware interrupts */
extern volatile float ultrasonic_distance_cm;

/* --- Ultrasonic Functions --- */
/**
 * @brief Sends the 10us trigger pulse to the HC-SR04.
 */
void trigger_ultrasonic(void);

/**
 * @brief Checks if the forward path is blocked using the HC-SR04.
 * @return 1 if collision is imminent, 0 if path is clear.
 */
uint8_t check_front_collision(void);

/* --- IR Sensor Data (To be implemented with your ADC code) --- */

/**
 * @brief Call this ONCE before your main loop starts to kick off the background DMA
 */
void ir_sensors_init(void);

/**
 * @brief Reads the left Sharp IR sensor via ADC.
 * @return Distance in cm.
 */
float get_sharp_ir_left_cm(void);

/**
 * @brief Reads the right Sharp IR sensor via ADC.
 * @return Distance in cm.
 */
float get_sharp_ir_right_cm(void);

/**
 * @brief Formats the voltages into strings for your OLED screen.
 */
void display_ir_voltages_oled(void);

/**
 * @brief Infinite loop to test the HC-SR04 and display on the OLED.
 *        WARNING: This is a blocking loop. Call this in main() for testing only.
 */
void test_ultrasonic_oled(void);

extern volatile uint8_t image_found;

/**
 * @brief Stop and re-arm the ADC/DMA. Returns 1 if the restart took.
 */
uint8_t ir_sensors_restart(void);

#endif /* SENSORS_H */
