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
            if (idx > 0u) {
                line[idx] = '\0';
                memcpy(pending, line, LINE_MAX);
                line_ready = 1u;
            }
            idx = 0u;
        } else if (idx < (LINE_MAX - 1u)) {
            line[idx++] = (char)rx_byte;
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* Protocol:  FW090  forward 90 cm      BW050  back 50 cm
              FL000  fwd-left 90 deg    FR000  fwd-right 90 deg
              BL000  rev-left 90 deg    BR000  rev-right 90 deg
              STOP                                             */
static void dispatch(const char *cmd)
{
    if (strncmp(cmd, "STOP", 4) == 0) { motion_stop(); command_send("ACK\n"); return; }
    if (motion_busy()) { command_send("BUSY\n"); return; }
    if (strlen(cmd) < 2u) { command_send("ERR\n"); return; }

    int32_t arg = (strlen(cmd) >= 5u) ? atoi(cmd + 2) : 0;

    if      (!strncmp(cmd, "FW", 2)) move_straight_mm( arg * 10);
    else if (!strncmp(cmd, "BW", 2)) move_straight_mm(-arg * 10);
    else if (!strncmp(cmd, "FL", 2)) move_turn(1, 1, TURN_COUNTS_FL);
    else if (!strncmp(cmd, "FR", 2)) move_turn(0, 1, TURN_COUNTS_FR);
    else if (!strncmp(cmd, "BL", 2)) move_turn(1, 0, TURN_COUNTS_BL);
    else if (!strncmp(cmd, "BR", 2)) move_turn(0, 0, TURN_COUNTS_BR);
    else { command_send("ERR\n"); return; }

    awaiting_ack = 1u;
}

void command_poll(void)
{
    if (line_ready) {
        line_ready = 0u;
        dispatch(pending);
    }
    /* Only ACK once the movement has genuinely finished. */
    if (awaiting_ack && !motion_busy()) {
        awaiting_ack = 0u;
        command_send("ACK\n");
    }
}
