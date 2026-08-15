/*
 * control.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef CONTROL_H
#define CONTROL_H
#include <stdint.h>

void    control_init(void);
void    control_tick(void);        /* called from TIM6 ISR */
uint8_t motion_busy(void);
void    motion_stop(void);

void move_straight_mm(int32_t mm);
void move_turn(int8_t left, int8_t forward, int32_t counts);

#endif
