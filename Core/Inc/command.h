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

#endif
