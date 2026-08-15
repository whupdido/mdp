/*
 * servo.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>

void servo_init(void);
void servo_us(uint16_t us);   /* pulse width in microseconds */

#endif
