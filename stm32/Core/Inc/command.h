/*
 * command.h
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */

#ifndef COMMAND_H
#define COMMAND_H
#include <stdint.h>

void command_init(void);
void command_poll(void);            /* call from main loop */
void command_send(const char *s);

/* Kush (09-Oct): for Task 2 runs started on the car itself, without the Pi
   (CAL_TASK2, cal_programs.c). */

/* GC (hands-off gyro zero, see command.c) without its DONE / ERR reply.
   1 if a new zero was kept, 0 if the car moved or a STOP / motion_stop()
   came in while it measured (the old zero is kept). */
uint8_t command_gyro_zero(void);

/* task_2_with_arrows() under START2's re-entry guard, with no ACK / DONE.
   Does nothing if a Task 2 is already running. */
void command_task2_local(uint8_t arrow_obs1, uint8_t arrow_obs2);

#endif
