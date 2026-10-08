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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "obstacle_nav.h"
#include "icm20948.h"
#include "encoders.h"
#include <math.h>

#define LINE_MAX 16
#define COMMAND_QUEUE_SIZE 8u

static uint8_t rx_byte;
static char    line[LINE_MAX];
static uint8_t idx = 0;
static char    command_queue[COMMAND_QUEUE_SIZE][LINE_MAX];
static volatile uint8_t queue_head = 0u;
static volatile uint8_t queue_tail = 0u;
static volatile uint8_t queue_overflow = 0u;
static volatile uint8_t stop_pending = 0u;
static uint8_t awaiting_ack = 0;
static char    last_motion_cmd[LINE_MAX] = "";

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
            if (idx > 0u) {
                uint8_t next_head = (uint8_t)((queue_head + 1u) % COMMAND_QUEUE_SIZE);
                line[idx] = '\0';
                if (strcmp(line, "STOP") == 0) {
                    /* Stop PWM immediately from the UART ISR.  The motor and
                       servo helpers only write timer registers, so this does
                       not wait for the main loop or a blocking motion delay. */
                    motion_stop();
                    stop_pending = 1u;
                } else if (!stop_pending && next_head != queue_tail) {
                    /* Publish queue_head only after the complete line has
                       been copied.  The main loop owns queue_tail, making
                       this a lock-free single-producer/single-consumer
                       queue between the UART ISR and command_poll(). */
                    memcpy(command_queue[queue_head], line, (size_t)idx + 1u);
                    queue_head = next_head;
                } else {
                    queue_overflow = 1u;
                }
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
 *   START2  run the whole Task 2 routine (blocks until it finishes)
 *   GC      re-measure the gyro zero with nobody touching the car (Kush,
 *           08-Oct). For the Pi's pre-flight. ACK,GC, then a "[GC] ..." line,
 *           then DONE, or ERR if the car moved while it measured (the old
 *           zero is kept). Takes about 2 s, up to 6 s if it has to retry.
 *           Meanwhile '?' gets STATUS,BUSY,GC, as during a move, and STOP
 *           abandons it (old zero kept; the STOP's ACK is the only reply).
 *
 * Turn angle is 1..360 degrees; xxx = 000 means 90 for backwards
 * compatibility, so FL000 still turns 90 degrees.
 *
 * Replies
 *   DONE      move completed normally, or Task 2 finished
 *   STALL     aborted: both wheels stopped turning for 1 s
 *   TIMEOUT   aborted: exceeded 20 s
 *   BLOCKED   aborted: IR saw an obstacle, stopped short (Zhenxi)
 *   ACK,<cmd> movement accepted and starting
 *   ACK       STOP acknowledged, or START2 accepted
 *   BUSY      a move was already running; this command was DISCARDED
 *   ERR       unrecognised command
 */

/* Zhenxi: guards task_2() against re-entering itself.
 *
 * task_2() blocks for the whole run and calls the move functions, and those
 * now poll the UART inside their wait loops so that a STOP can interrupt a
 * move. That polling is what makes re-entry reachable:
 *
 *   command_poll -> dispatch -> task_2 -> move_straight_mm
 *                -> command_poll -> dispatch -> task_2   <-- nested
 *
 * A second START2 arriving mid-run would start a whole second Task 2 inside
 * the first, and the outer one would then carry on from a position it no
 * longer understands. motion_busy() does not catch it, because between moves
 * there is no move running. This does.
 *
 * I widened that window when I added the polling, so the guard belongs with
 * it rather than with task_2().
 */
static volatile uint8_t task2_running = 0u;

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

/* A status probe is deliberately separate from the normal BUSY reply.
 * BUSY means a newly submitted movement command was rejected; STATUS,BUSY
 * means the already accepted movement is still running.  When idle, retain
 * and report the last movement verdict so the Pi can recover if the original
 * terminal line was lost on the UART. */
/* Kush: GC is not a move, so motion_busy() and motion_result() know nothing
   about it. These stand in for them while it runs and once it has ended
   ("DONE", "ERR" or "STOPPED"); see report_status() and dispatch(). */
static volatile uint8_t gc_running = 0u;
static const char *gc_status = "NONE";

static void report_status(void)
{
    char response[48];

    if (motion_busy() || gc_running) {
        snprintf(response, sizeof(response), "STATUS,BUSY,%s\r\n",
                 last_motion_cmd[0] ? last_motion_cmd : "NONE");
        command_send(response);
        return;
    }

    const char *result;
    if (strcmp(last_motion_cmd, "GC") == 0) {
        /* Kush: a probe answered after a GC must give GC's own verdict. The
           Pi ignores a bare ERR while its probe is outstanding (it reads it
           as old firmware rejecting '?'), so if this said the last move's
           DONE instead, a failed GC would pass the pre-flight. */
        result = gc_status;
    } else switch (motion_result()) {
        case MOVE_DONE:    result = "DONE";    break;
        case MOVE_STALL:   result = "STALL";   break;
        case MOVE_TIMEOUT: result = "TIMEOUT"; break;
        case MOVE_BLOCKED: result = "BLOCKED"; break;
        case MOVE_ABORT:   result = "STOPPED"; break;
        default:           result = "NONE";    break;
    }

    snprintf(response, sizeof(response), "STATUS,IDLE,%s,%s\r\n",
             result, last_motion_cmd[0] ? last_motion_cmd : "NONE");
    command_send(response);
}

static void remember_motion_command(const char *cmd)
{
    strncpy(last_motion_cmd, cmd, LINE_MAX - 1u);
    last_motion_cmd[LINE_MAX - 1u] = '\0';
}

static void begin_motion(const char *cmd)
{
    char response[LINE_MAX + 8u];

    motion_abort_clear();
    remember_motion_command(cmd);
    snprintf(response, sizeof(response), "ACK,%s\r\n", cmd);
    command_send(response);
}

static void handle_stop(void)
{
    /* STOP invalidates every command accepted before its acknowledgement.
       Flushing here also discards commands which arrived just behind STOP
       before the main loop observed stop_pending. */
    queue_tail = queue_head;
    queue_overflow = 0u;
    awaiting_ack = 0u;
    stop_pending = 0u;
    motion_stop();
    command_send("ACK\r\n");
}

/* ------------------------------------------------------------------ GC ---
 * Kush (08-Oct): hands-off gyro zero. Holding SW1 to set the zero means a
 * hand is on or near the car while it measures, and a nudge in that window
 * leaves the zero off by the angle turned over the time taken: 1.5 deg of
 * nudge during the 1.5 s it measures is a 1 deg/s turn that is not
 * happening. The straights then steer the car to cancel it, and the run is
 * visibly tilted by the first obstacle. The Pi sends GC in its pre-flight
 * instead, when nobody is touching the car.
 *
 * Each try reads the gyro for 1.5 s and keeps the new zero only if the car
 * was still. The test that matters is the turn: the readings integrated
 * against their own mean. For a still car that stays within a few
 * hundredths of a degree all the way through; a nudge shows up as a step of
 * the angle it turned. Refusing any turn over GC_MAX_TURN_DEG keeps the new
 * zero within about 2 * 0.05 deg / 1.5 s = 0.07 deg/s of right after any
 * nudge shorter than half the window. A push slow and smooth enough to last
 * the whole window would look like an offset and get through, but nothing
 * turns a parked car like that. The other three tests mostly give the OLED
 * a clearer reason. Simulated against all of this before it went in.
 *
 * While it measures, GC keeps reading the UART as a move does, so the Pi's
 * '?' is answered at once (STATUS,BUSY,GC) instead of queueing up behind it
 * and being answered late, where the Pi could take the answer for its next
 * command's. */
#define GC_SETTLE_MS        500u   /* before each try, so a bump can die away       */
#define GC_SAMPLES          500    /* 3 ms apart (read + HAL_Delay(2)): 1.5 s a try */
#define GC_TRIES            3
#define GC_MAX_TURN_DEG     0.05f  /* net turn at any point. Still car: ~0.015      */
#define GC_MAX_NOISE_DPS    0.8f   /* standard deviation. Still car: ~0.25          */
#define GC_MAX_SPAN_DPS     6.0f   /* highest minus lowest reading. Still car: ~2   */
#define GC_MAX_WHEEL_COUNTS 2      /* both encoders together                         */

extern int calibrated;             /* main.c: a gyro zero has been set */

typedef struct {
    float       shift;   /* mean reading: how far the old zero was off, deg/s */
    float       noise;   /* standard deviation of the readings, deg/s         */
    float       span;    /* highest minus lowest reading, deg/s               */
    float       turn;    /* largest net turn during the window, deg           */
    int32_t     wheels;  /* encoder counts moved, both wheels together        */
    const char *why;     /* NULL if the car was still, else what gave it away */
} gc_window_t;

static float gc_buf[GC_SAMPLES];   /* 2 KB, kept off the stack */

/* HAL_Delay that keeps the UART answered. Ends early on STOP. */
static void gc_wait_ms(uint32_t ms)
{
    const uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms && !motion_abort_requested()) {
        command_poll();
        HAL_Delay(1);
    }
}

/* One window. Readings come back already minus the current zero, so their
   mean is how far that zero is off. Moves the zero only if the car was
   still. Returns early, changing nothing, on STOP. */
static uint8_t gc_try(gc_window_t *w)
{
    const int32_t  l0 = enc_left_total, r0 = enc_right_total;
    const uint32_t t0 = HAL_GetTick();
    float sum = 0.0f, lo = 1e9f, hi = -1e9f;

    for (int i = 0; i < GC_SAMPLES; i++) {
        float g = icm20948_read_gyro_z();
        gc_buf[i] = g;
        sum += g;
        if (g < lo) lo = g;
        if (g > hi) hi = g;
        command_poll();
        if (motion_abort_requested()) return 0u;
        HAL_Delay(2);
    }
    const float n    = (float)GC_SAMPLES;
    const float mean = sum / n;
    const float dt_s = (float)(HAL_GetTick() - t0) * 0.001f / n;

    float sq = 0.0f, angle = 0.0f, turn = 0.0f;
    for (int i = 0; i < GC_SAMPLES; i++) {
        float d = gc_buf[i] - mean;
        sq    += d * d;
        angle += d * dt_s;
        if (fabsf(angle) > turn) turn = fabsf(angle);
    }
    w->shift  = mean;
    w->noise  = sqrtf(sq / n);
    w->span   = hi - lo;
    w->turn   = turn;
    w->wheels = (int32_t)(labs((long)(enc_left_total - l0)) +
                          labs((long)(enc_right_total - r0)));

    if      (w->wheels > GC_MAX_WHEEL_COUNTS) w->why = "wheels turned";
    else if (w->span   > GC_MAX_SPAN_DPS)     w->why = "car was bumped";
    else if (w->noise  > GC_MAX_NOISE_DPS)    w->why = "car was shaking";
    else if (w->turn   > GC_MAX_TURN_DEG)     w->why = "car turned";
    else                                      w->why = NULL;

    if (w->why != NULL) return 0u;
    gyro_z_bias += mean;           /* the ISR only reads it; one 32-bit store */
    return 1u;
}

static void gyro_zero_command(void)
{
    char u[128];
    char row[24];
    gc_window_t w = { 0 };
    uint8_t ok = 0u;
    int tries = 0;

    OLED_Clear();
    OLED_ShowString(0, 0,  (const uint8_t *)"GYRO ZERO");
    OLED_ShowString(0, 20, (const uint8_t *)"HANDS OFF...");
    OLED_Refresh_Gram();

    gc_running = 1u;
    while (!ok && tries < GC_TRIES && !motion_abort_requested()) {
        gc_wait_ms(GC_SETTLE_MS);
        if (motion_abort_requested()) break;
        ok = gc_try(&w);
        tries++;
    }
    gc_running = 0u;

    if (!ok && motion_abort_requested()) {
        /* STOP. handle_stop() answers it with ACK, now or on the next poll,
           and that is the only reply: like a stopped move, no DONE or ERR
           follows, so nothing is left over for the Pi's next wait to take
           as its own. */
        gc_status = "STOPPED";
        command_send("[GC] stopped, old zero kept\r\n");
        OLED_Clear();
        OLED_ShowString(0, 0,  (const uint8_t *)"GYRO ZERO STOPPED");
        OLED_ShowString(0, 20, (const uint8_t *)"old zero kept");
        OLED_Refresh_Gram();
        return;
    }

    snprintf(u, sizeof u,
             "[GC] %s try=%d shift_dps=%+.3f noise_dps=%.3f span_dps=%.2f"
             " turn_deg=%.3f wheels=%ld\r\n",
             ok ? "ok" : "FAILED", tries, (double)w.shift, (double)w.noise,
             (double)w.span, (double)w.turn, (long)w.wheels);
    command_send(u);
    if (!ok) {
        snprintf(u, sizeof u, "[GC] %s, old zero kept\r\n", w.why);
        command_send(u);
    }

    OLED_Clear();
    if (ok) {
        OLED_ShowString(0, 0, (const uint8_t *)"GYRO ZERO OK");
        snprintf(row, sizeof row, "shift %+.2f dps", (double)w.shift);
        OLED_ShowString(0, 20, (const uint8_t *)row);
        snprintf(row, sizeof row, "noise %.2f dps", (double)w.noise);
        OLED_ShowString(0, 30, (const uint8_t *)row);
        OLED_ShowString(0, 40, (const uint8_t *)"ready for START");
    } else {
        OLED_ShowString(0, 0,  (const uint8_t *)"GYRO ZERO FAILED");
        OLED_ShowString(0, 20, (const uint8_t *)w.why);
        OLED_ShowString(0, 30, (const uint8_t *)"hands off the car");
        OLED_ShowString(0, 40, (const uint8_t *)"and try again");
    }
    OLED_Refresh_Gram();

    if (ok) {
        calibrated = 1;
        gc_status  = "DONE";
        command_send("DONE\r\n");
    } else {
        gc_status  = "ERR";
        command_send("ERR\r\n");
    }
}

static void dispatch(const char *cmd)
{
    /* Handle probes before the busy guard.  A probe observes the outstanding
       move; it is not a second movement command and must never be rejected as
       BUSY. */
    if (strcmp(cmd, "?") == 0) { report_status(); return; }
    if (strncmp(cmd, "STOP", 4) == 0) { handle_stop(); return; }
    if (motion_busy() || gc_running) { command_send("BUSY\r\n"); return; }
    if (strlen(cmd) < 2u) { command_send("ERR\r\n"); return; }

    int32_t arg = (strlen(cmd) >= 5u) ? atoi(cmd + 2) : 0;

    if      (!strncmp(cmd, "FW", 2)) { begin_motion(cmd); move_straight_mm( arg * 10); }
    else if (!strncmp(cmd, "BW", 2)) { begin_motion(cmd); move_straight_mm(-arg * 10); }
    else if (!strncmp(cmd, "FL", 2)) { begin_motion(cmd); move_turn_deg(1, 1, arg); }
    else if (!strncmp(cmd, "FR", 2)) { begin_motion(cmd); move_turn_deg(0, 1, arg); }
    else if (!strncmp(cmd, "BL", 2)) { begin_motion(cmd); move_turn_deg(1, 0, arg); }
    else if (!strncmp(cmd, "BR", 2)) { begin_motion(cmd); move_turn_deg(0, 0, arg); }
    /* Zhenxi: IM is not a move, so it must not go through report_result()
       -- that would echo whatever the *previous* move's verdict was. It
       replied DONE before (by falling through) and still does.          */
    else if (!strncmp(cmd, "IM", 2)) { image_found = (uint8_t)arg; command_send("DONE\r\n"); return; }
    /* Kush: GC replies DONE or ERR itself (see gyro_zero_command). Exact
       match, so report_status() can tell a GC from a move by name. */
    else if (!strcmp(cmd, "GC")) { begin_motion(cmd); gyro_zero_command(); return; }
    /* Zhenxi: Task 2. Answers on the same contract as everything else, so
       the Pi and the tablet are not left guessing for three minutes.
         ACK   accepted, the routine has begun
         DONE  the routine returned
         BUSY  one is already running (see task2_running above)
       Without the ACK the bridge waits STM_TIMEOUT_SECONDS and reports
       NO_REPLY, which the tablet shows as a warning while the robot is in
       fact running Task 2 perfectly well. */
    else if (!strncmp(cmd, "START2", 6)) {
        if (task2_running) { command_send("BUSY\r\n"); return; }
        motion_abort_clear();
        task2_running = 1u;
        command_send("ACK\r\n");
        task_2();
        task2_running = 0u;
        if (!motion_abort_requested()) command_send("DONE\r\n");
        return;
    }
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
    if (stop_pending) {
        handle_stop();
        return;
    }

    if (queue_tail != queue_head) {
        char local[LINE_MAX];
        uint8_t tail = queue_tail;
        memcpy(local, command_queue[tail], LINE_MAX);
        /* Release the queue slot before dispatch.  dispatch() may block in a
           move and recursively call command_poll(), so advancing first lets
           a queued probe, STOP, and following movement drain in order. */
        queue_tail = (uint8_t)((tail + 1u) % COMMAND_QUEUE_SIZE);
        dispatch(local);
    }

    if (queue_overflow) {
        queue_overflow = 0u;
        command_send("ERR\r\n");
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
