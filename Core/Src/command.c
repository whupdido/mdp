/*
 * command.c
 *
 *  Created on: 15-Aug-2026
 *      Author: Kush Agrawal
 */


#include "command.h"
#include "control.h"
#include "calib.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

#define LINE_MAX 16

static uint8_t rx_byte;
static char    line[LINE_MAX];
static uint8_t idx = 0;
static volatile uint8_t line_ready = 0;
static char    pending[LINE_MAX];
static uint8_t awaiting_ack = 0;

void command_send(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

void command_init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/* ISR context: assemble a line, do nothing else. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            /* Only latch if the previous line has actually been consumed,
               otherwise a fast second command overwrites pending[] while
               command_poll() is still reading it. */
            if (idx > 0u && line_ready == 0u) {
                line[idx] = '\0';
                memcpy(pending, line, (size_t)idx + 1u);
                line_ready = 1u;
            }
            idx = 0u;
        } else if (idx < (LINE_MAX - 1u)) {
            line[idx++] = (char)rx_byte;
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* An overrun (ORE) aborts the HAL receive state machine and stops it
   re-arming, which shows up as "the UART worked for a while then went dead".
   Clear the flag and restart reception. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        idx = 0u;
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* Protocol
 *   FWxxx   forward xxx cm          BWxxx   backward xxx cm
 *   FLxxx   forward-left  xxx deg   FRxxx   forward-right  xxx deg
 *   BLxxx   reverse-left  xxx deg   BRxxx   reverse-right  xxx deg
 *   STOP    abort the current move
 *
 * Turn angle is 1..360 degrees; xxx = 000 means 90 for backwards
 * compatibility, so FL000 still turns 90 degrees.
 *
 * Replies
 *   DONE      move completed normally
 *   STALL     aborted: both wheels stopped turning for 1 s
 *   TIMEOUT   aborted: exceeded 20 s
 *   ACK       STOP acknowledged
 *   BUSY      a move was already running; this command was DISCARDED
 *   ERR       unrecognised command
 */
static void dispatch(const char *cmd)
{
    if (strncmp(cmd, "STOP", 4) == 0) { motion_stop(); command_send("ACK\r\n"); return; }
    if (motion_busy()) { command_send("BUSY\r\n"); return; }
    if (strlen(cmd) < 2u) { command_send("ERR\r\n"); return; }

    int32_t arg = (strlen(cmd) >= 5u) ? atoi(cmd + 2) : 0;

    if      (!strncmp(cmd, "FW", 2)) move_straight_mm( arg * 10);
    else if (!strncmp(cmd, "BW", 2)) move_straight_mm(-arg * 10);
    else if (!strncmp(cmd, "FL", 2)) move_turn_deg(1, 1, arg);
    else if (!strncmp(cmd, "FR", 2)) move_turn_deg(0, 1, arg);
    else if (!strncmp(cmd, "BL", 2)) move_turn_deg(1, 0, arg);
    else if (!strncmp(cmd, "BR", 2)) move_turn_deg(0, 0, arg);
    else { command_send("ERR\r\n"); return; }

    /* A zero-length move (e.g. FW000) never arms the state machine, so it
       would otherwise sit here waiting for a motion that never starts. */
    if (!motion_busy()) { command_send("DONE\r\n"); return; }

    awaiting_ack = 1u;
}

void command_poll(void)
{
    if (line_ready) {
        char local[LINE_MAX];
        memcpy(local, pending, LINE_MAX);
        line_ready = 0u;              /* release the buffer before dispatch */
        dispatch(local);
    }

    /* Report only once the movement has genuinely finished, and say HOW it
       finished. A stalled move used to be indistinguishable from a completed
       one, so the Pi would keep dead-reckoning from a position the robot
       never reached. */
    if (awaiting_ack && !motion_busy()) {
        awaiting_ack = 0u;
        switch (motion_result()) {
            case MOVE_DONE:    command_send("DONE\r\n");    break;
            case MOVE_STALL:   command_send("STALL\r\n");   break;
            case MOVE_TIMEOUT: command_send("TIMEOUT\r\n"); break;
            default:           command_send("ACK\r\n");     break;
        }
    }
}
