#ifndef MOTORS_H
#define MOTORS_H
#include <stdint.h>

/* PWM period for TIM9 / TIM10 / TIM11.
 *
 * ARR = 16799, PSC = 0, timer clock 168 MHz  ->  168e6 / 16800 = 10.000 kHz.
 *
 * Duty is expressed as -PWM_MAX .. +PWM_MAX.
 *
 */
#define PWM_MAX 16799

void motors_init(void);
void motor_left (int32_t duty);   /* -PWM_MAX .. +PWM_MAX */
void motor_right(int32_t duty);

void motors_brake(void);          /* both bridge inputs HIGH -> active brake */
void motors_coast(void);          /* both bridge inputs LOW  -> free wheel   */

#endif
