/*
 * motors.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef MOTORS_H
#define MOTORS_H
#include <stdint.h>

#define PWM_MAX 8399

void motors_init(void);
void motor_left(int32_t duty);    /* -PWM_MAX .. +PWM_MAX */
void motor_right(int32_t duty);
void motors_coast(void);

#endif
