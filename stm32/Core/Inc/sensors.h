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

/* --- Navigation Safety Functions --- */
/**
 * @brief Checks if the forward path is blocked using the Sharp IR sensors.
 * @return 1 if collision is imminent, 0 if path is clear.
 */
uint8_t check_front_collision(void);

/**
 * @brief Checks if the reverse path is blocked using the HC-SR04.
 * @return 1 if collision is imminent, 0 if path is clear.
 */
uint8_t check_rear_collision(void);

/* --- IR Sensor Data (To be implemented with your ADC code) --- */
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
 * @brief Infinite loop to test the HC-SR04 and display on the OLED.
 *        WARNING: This is a blocking loop. Call this in main() for testing only.
 */
void test_ultrasonic_oled(void);

#endif /* SENSORS_H */
