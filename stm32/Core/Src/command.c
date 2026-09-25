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
#include "sensors.h"
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "obstacle_nav.h"

#define LINE_MAX 16

static uint8_t rx_byte;
static char    line[LINE_MAX];
static uint8_t idx = 0;
static volatile uint8_t line_ready = 0;
static char    pending[LINE_MAX];
static uint8_t awaiting_ack = 0;

void oled_countdown(){
	OLED_ShowString(10,0,(const uint8_t* )"Get Ready...");
	OLED_ShowString(10,10,(const uint8_t* )"In 5...");
	OLED_Refresh_Gram();
	HAL_Delay(1000);
	OLED_ShowString(10,20,(const uint8_t* )"4...");
	OLED_Refresh_Gram();
	HAL_Delay(1000);
	OLED_ShowString(10,30,(const uint8_t* )"3...");
	OLED_Refresh_Gram();
	HAL_Delay(1000);
	OLED_ShowString(10,40,(const uint8_t* )"2...");
	OLED_Refresh_Gram();
	HAL_Delay(1000);
	OLED_ShowString(10,50,(const uint8_t* )"1...");
	OLED_Refresh_Gram();
	HAL_Delay(1000);
}

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
 *   BLOCKED   aborted: IR saw an obstacle, stopped short (Zhenxi)
 *   ACK       STOP acknowledged
 *   BUSY      a move was already running; this command was DISCARDED
 *   ERR       unrecognised command
 */

/* Zhenxi: the one place a move result becomes a reply.
 *
 * Since the moves in control.c became blocking (08-26), dispatch() only
 * gets control back once the move is over -- so its "not busy, so DONE"
 * shortcut below fired for every move, and the switch that used to live
 * in command_poll() (and told STALL from DONE) was never reached. The
 * tablet has a "position no longer trustworthy" warning that keys off
 * STALL / TIMEOUT; it had gone silent. Both paths now come through here.
 *
 * MOVE_ABORT is deliberately silent: the STOP that caused it was answered
 * with ACK by its own dispatch() call, from inside the move's poll loop.
 * Reporting it again here would give the tablet two ACKs for one STOP. */
static void report_result(void)
{
    switch (motion_result()) {
        case MOVE_DONE:    command_send("DONE\r\n");    break;
        case MOVE_STALL:   command_send("STALL\r\n");   break;
        case MOVE_TIMEOUT: command_send("TIMEOUT\r\n"); break;
        case MOVE_BLOCKED: command_send("BLOCKED\r\n"); break;
        case MOVE_ABORT:   /* already ACKed, see above */ break;
        default:           command_send("ACK\r\n");     break;
    }
}

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
    /* Zhenxi: IM is not a move, so it must not go through report_result()
       -- that would echo whatever the *previous* move's verdict was. It
       replied DONE before (by falling through) and still does.          */
    else if (!strncmp(cmd, "IM", 2)) { image_found = (uint8_t)arg; command_send("DONE\r\n"); return; }
    else if (!strncmp(cmd, "START2", 6)) { task_2(); return; }
    else { command_send("ERR\r\n"); return; }

    /* Zhenxi: with blocking moves this is the normal path, not just the
       zero-length one. Report how it ended, not just that it ended.
       (A zero-length move sets MOVE_DONE itself before returning, so it
       still reads as DONE here.)                                         */
    if (!motion_busy()) { report_result(); return; }

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
       never reached.
       Zhenxi: unreachable while the moves block, kept for the day they stop
       blocking again; routed through report_result() so the two agree.  */
    if (awaiting_ack && !motion_busy()) {
        awaiting_ack = 0u;
        report_result();
    }
}
