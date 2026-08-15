/*
 * encoders.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef ENCODERS_H
#define ENCODERS_H
#include <stdint.h>

extern volatile int32_t enc_left_total;
extern volatile int32_t enc_right_total;
extern volatile int32_t enc_left_delta;    /* counts in last 10 ms */
extern volatile int32_t enc_right_delta;

void encoders_init(void);
void encoders_reset(void);
void encoders_sample(void);                /* call at 100 Hz */

#endif
