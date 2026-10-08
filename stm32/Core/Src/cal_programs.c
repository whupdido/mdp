/*
 * cal_programs.c -- SW1-driven calibration programs. Pick one with
 * CAL_PROGRAM in main.c; the turn radius program (CAL_TURN) is turn_test.c.
 *
 * CAL_STRAIGHT  Each SW1 tap drives one straight move with move_straight_mm(),
 *               the same call a FW/BW from the Pi makes, and shows where it
 *               came to rest:
 *                   FW 10 cm, BW 10 cm, FW 100 cm, BW 100 cm, 5 rounds (20 taps)
 *               Each pair brings the car back to about where it started; it
 *               needs a clear metre in front. Then three summary taps (stop,
 *               head, side) and a fourth to start over.
 *                 stop  how far past (+) or short of (-) the target the car
 *                       stopped, from the wheel encoders, coast included. If
 *                       the 10 cm and 100 cm moves are off by about the same
 *                       amount, add it to BRAKE_LEAD_COUNTS in control.c (mm).
 *                       If the 100 cm error is about ten times the 10 cm one,
 *                       it is the distance scale instead: use CAL_ODOMETER.
 *                 head  heading change over the move (gyro), + = turned left.
 *                 side  sideways drift of the rear-axle centre, + = left.
 *                       Both should stay near 0. A steady drift one way when
 *                       reversing is reverse_bias in control.c; forward drift
 *                       or weaving is the straight steering gains (STEER_KI).
 *               The encoders count wheel turns, not the floor, so check a few
 *               runs with a tape. Where the two disagree, trust the tape.
 *
 * CAL_ODOMETER  Free-wheels the motors so the car can be pushed by hand, and
 *               shows distance, both wheel counts, heading and sideways drift
 *               live. A tap zeroes it. Push the car exactly 1 m along a tape,
 *               watching the rear-axle centre and keeping it straight, then
 *               read CPR@1m: that is the COUNTS_PER_REV in calib.h that makes
 *               1 m read as 1 m. Turning the car by hand one full turn, back
 *               against a straight edge, should read 360 deg: a gyro check.
 *
 * CAL_SIX_FR / _FL / _BR / _BL
 *               Six identical 30 degree turns of that kind back to back
 *               (WRT4's check from the integration branch). The car should finish facing exactly
 *               the other way, with the rear-axle centre straight across the
 *               circle from where it started: mark the floor under it before
 *               and after, and the radius is half the gap. The OLED shows the
 *               total angle the gyro saw and the radius from the encoders.
 *
 * CAL_SEQ       Turns followed by a straight, back to back as in a real run:
 *                   FL030, FR030, FW100     (SEQ_MOVES below, Pi command strings)
 *               Each tap runs the whole sequence once; 5 runs, then a summary
 *               tap and one more to start over. Square the car up to a floor
 *               line before each tap and give it about 1.5 m of room ahead
 *               (the two turns alone move it 32 cm on and 8 cm left).
 *               A straight holds whatever heading it starts with, so if the
 *               turns before it come out at 31 and 29 deg instead of 30 and 30,
 *               the straight runs dead straight but on a tilted line. CAL_STRAIGHT
 *               cannot show that, because there you line the car up by hand.
 *                 ang   how far each turn actually turned (gyro), target 30.
 *                 tilt  direction the straight actually drove, against the
 *                       direction the plan expects, + = left. It is the sum
 *                       of the turn errors before it when those are to blame.
 *                 side  where the straight ended against a line drawn from
 *                       its start in the planned direction, + = left. Check it
 *                       on the floor with a tape.
 *                 end   heading at the end against the plan, + = left.
 *               SEQ_GAP_MS is the pause between moves; the Pi sends the next
 *               command as soon as DONE arrives, so it is short.
 *
 * CAL_TRACE     Watches one straight (TR_MM, FW100) from inside the 100 Hz
 *               loop, to find out why straights end up turned. Tap to drive,
 *               tap twice more for pages 2 and 3, then tap for the next run.
 *               The move is split by what the commanded speed is doing:
 *               acc rising, cru flat at cruise, dec falling for the target,
 *               crl flat at crawl (the last stretch before the brake).
 *                 page 1  L% R%  each wheel's actual speed as a % of the
 *                                commanded speed, phase by phase.
 *                         hdg    heading change in that phase, + = left.
 *                 page 2  how far and how long it crawled, the average
 *                         steering during the crawl (R/L), ticks where one
 *                         wheel stood while the other turned, the heading
 *                         change after the brake, and over the whole move.
 *                 page 3  average motor duty per wheel at cruise and crawl,
 *                         and the integral part of it at the brake.
 *               The idea it tests: the speed loop is close to proportional
 *               only, so the weaker left wheel lags; the car slows too early,
 *               crawls a long way, and pivots left there, where the steering
 *               is at a quarter strength. That shows up as cru L% and R% well
 *               under 100 with L below R; a crawl far longer than ~14 mm;
 *               crl L% well below R% or L stalls; hdg + in crl while the
 *               crawl steering is R; and Iend small next to the duty.
 *               Every tick also goes to USART3 as "[TRS]" CSV lines.
 *
 * Nothing here changes how the car drives. As in a real run the brake stays
 * on after each move, and the readings keep running for CAL_SETTLE_MS
 * afterwards so the coast is included. Every result also goes to USART3:
 * "[CS]" straight, "[OD]" odometer, "[T30]" six turns, "[SEQ]" sequence,
 * "[TR]"/"[TRS]" trace.
 */

#include "main.h"
#include "cal_programs.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"
#include "servo.h"
#include "command.h"
#include "calib.h"
#include "oled.h"
#include "icm20948.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CAL_SETTLE_MS   700u   /* keep reading after a move: the coast           */
#define CAL_START_MS    800u   /* after the tap, so your hand is off the car     */
#define CAL_DEG2RAD     (3.14159265f / 180.0f)

extern int calibrated;         /* main.c: set once the gyro bias is locked       */

/* ---------------------------------------------------------------- helpers */

/* One OLED text line. This display fits 16 characters (8 px each) and wraps
   anything longer onto the next line, so every line is cut and padded to
   exactly 16. The padding also overwrites what was there, which lets the
   odometer redraw without OLED_Clear() (that refreshes the panel: flicker). */
__attribute__((format(printf, 2, 3)))
static void cal_line(uint8_t y, const char *fmt, ...)
{
    char text[48];
    char line[17];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    snprintf(line, sizeof line, "%-16.16s", text);
    OLED_ShowString(0, y, (const uint8_t *)line);
}

static const char *cal_result_name(move_result_t r)
{
    switch (r) {
        case MOVE_DONE:    return "DONE";
        case MOVE_STALL:   return "STALL";
        case MOVE_TIMEOUT: return "TIMEOUT";
        case MOVE_ABORT:   return "ABORT";
        case MOVE_BLOCKED: return "BLOCKED";
        default:           return "?";
    }
}

/* Every angle is a gyro integral, so an uncalibrated bias becomes an angle
   error that grows with time. Same routine as the turn test's. */
static void cal_gyro_if_needed(void)
{
    if (calibrated) return;
    OLED_Clear();
    cal_line(0,  "Gyro bias");
    cal_line(10, "HOLD CAR STILL");
    OLED_Refresh_Gram();
    HAL_Delay(1500);
    icm20948_calib_gyro_bias();
    calibrated = 1;
    cal_line(30, "locked");
    OLED_Refresh_Gram();
    HAL_Delay(600);
}

static float cal_clamp(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

static void cal_stats(const float *x, uint8_t n, float *mean, float *half_range)
{
    float sum = 0.0f, lo = 1e9f, hi = -1e9f;
    for (uint8_t i = 0; i < n; i++) {
        sum += x[i];
        if (x[i] < lo) lo = x[i];
        if (x[i] > hi) hi = x[i];
    }
    *mean       = sum / (float)n;
    *half_range = (hi - lo) * 0.5f;
}

/* Where the car is now, to measure a move against. */
typedef struct {
    int32_t l0, r0;
    float   yaw0, x0, y0;
} cal_mark_t;

/* What changed since a mark, in the frame the car was facing at the mark. */
typedef struct {
    int32_t dl, dr;     /* encoder counts                                 */
    float   path_mm;    /* distance the wheels turned, + = forward        */
    float   along_mm;   /* rear-axle centre displacement along the start  */
    float   side_mm;    /*   heading, and to its left (+)                 */
    float   turn_deg;   /* heading change, + = left (CCW)                 */
} cal_delta_t;

static void cal_mark(cal_mark_t *m)
{
    m->l0   = enc_left_total;
    m->r0   = enc_right_total;
    m->yaw0 = motion_yaw_deg();
    motion_odometry_mm(&m->x0, &m->y0);
}

static void cal_since(const cal_mark_t *m, cal_delta_t *d)
{
    float x, y;
    motion_odometry_mm(&x, &y);
    d->dl = enc_left_total  - m->l0;
    d->dr = enc_right_total - m->r0;
    d->path_mm = 0.5f * (float)(d->dl + d->dr) * MM_PER_COUNT;

    float dx = x - m->x0;
    float dy = y - m->y0;
    float h0 = m->yaw0 * CAL_DEG2RAD;
    d->along_mm =  dx * cosf(h0) + dy * sinf(h0);
    d->side_mm  = -dx * sinf(h0) + dy * cosf(h0);
    d->turn_deg = motion_yaw_deg() - m->yaw0;
}

/* ----------------------------------------------------------- CAL_STRAIGHT */

typedef struct {
    const char *name;
    int32_t     mm;
} cs_case_t;

/* Run order: case = run % CS_NCASES, so each round goes out and back. */
static const cs_case_t CS_CASES[] = {
    { "FW100",  500 },
};
/* Counts the list, so entries can be added or removed. It was a fixed 4: with
   one entry left, taps 2 to 4 read past the end of CS_CASES and drove
   whatever number happened to sit there in flash. */
#define CS_NCASES   ((uint8_t)(sizeof CS_CASES / sizeof CS_CASES[0]))
#define CS_REPEATS  5u

/* CS_SUM_* mean "the next tap shows that page". */
typedef enum { CS_RUNNING = 0, CS_SUM_STOP, CS_SUM_HEAD, CS_SUM_SIDE, CS_DONE } cs_phase_t;

static cs_phase_t cs_phase = CS_RUNNING;
static uint8_t    cs_run   = 0;            /* counted runs so far, 0..19 */
static float      cs_stop[CS_NCASES][CS_REPEATS];
static float      cs_head[CS_NCASES][CS_REPEATS];
static float      cs_side[CS_NCASES][CS_REPEATS];

void cal_straight_show_idle(void)
{
    const cs_case_t *c = &CS_CASES[cs_run % CS_NCASES];
    OLED_Clear();
    cal_line(0,  "STRAIGHT TEST");
    cal_line(10, "next %s %u/%u", c->name,
             (unsigned)(cs_run / CS_NCASES + 1u), (unsigned)CS_REPEATS);
    cal_line(30, "SW1 = drive");
    if (!calibrated) cal_line(40, "(gyro cal 1st)");
    cal_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

/* page 0 = stop, 1 = heading, 2 = sideways */
static void cs_show_summary(uint8_t page)
{
    static const char *const TITLE[] = { "STOP mm  mean +-", "HEAD deg mean +-", "SIDE mm  mean +-" };
    static const char *const FIELD[] = { "stop_mm", "heading_deg", "side_mm" };
    static const char *const NEXT[]  = { "SW1 = heading", "SW1 = sideways", "SW1 = restart" };
    char u[80];

    OLED_Clear();
    cal_line(0, "%s", TITLE[page]);
    for (uint8_t ci = 0; ci < CS_NCASES; ci++) {
        const float *x = (page == 0) ? cs_stop[ci] : (page == 1) ? cs_head[ci] : cs_side[ci];
        float mean, hr;
        cal_stats(x, CS_REPEATS, &mean, &hr);
        if (page == 1) {
            cal_line((uint8_t)(10u + ci * 10u), "%s%+6.2f %4.2f", CS_CASES[ci].name,
                     (double)cal_clamp(mean, 99.99f), (double)cal_clamp(hr, 9.99f));
        } else {
            cal_line((uint8_t)(10u + ci * 10u), "%s%+6.1f %4.1f", CS_CASES[ci].name,
                     (double)cal_clamp(mean, 999.9f), (double)cal_clamp(hr, 99.9f));
        }
        snprintf(u, sizeof u, "[CS] SUMMARY %s %s mean=%.2f +-%.2f\r\n",
                 CS_CASES[ci].name, FIELD[page], (double)mean, (double)hr);
        command_send(u);
    }
    cal_line(50, "%s", NEXT[page]);
    OLED_Refresh_Gram();
}

void cal_straight_step(void)
{
    char u[200];

    /* --- summary pages --- */
    if (cs_phase == CS_SUM_STOP) { cs_show_summary(0); cs_phase = CS_SUM_HEAD; return; }
    if (cs_phase == CS_SUM_HEAD) { cs_show_summary(1); cs_phase = CS_SUM_SIDE; return; }
    if (cs_phase == CS_SUM_SIDE) { cs_show_summary(2); cs_phase = CS_DONE;     return; }
    if (cs_phase == CS_DONE) {
        cs_phase = CS_RUNNING;
        cs_run   = 0;
        command_send("[CS] restart\r\n");
        cal_straight_show_idle();
        return;
    }

    /* --- one move --- */
    cal_gyro_if_needed();

    uint8_t ci  = (uint8_t)(cs_run % CS_NCASES);
    uint8_t rep = (uint8_t)(cs_run / CS_NCASES);
    const cs_case_t *c = &CS_CASES[ci];

    OLED_Clear();
    cal_line(0,  "%s run %u/%u", c->name, (unsigned)(rep + 1u), (unsigned)CS_REPEATS);
    cal_line(20, "running...");
    OLED_Refresh_Gram();
    HAL_Delay(CAL_START_MS);

    cal_mark_t m;
    cal_mark(&m);
    move_straight_mm(c->mm);
    move_result_t res = motion_result();
    HAL_Delay(CAL_SETTLE_MS);   /* brake on, as in a run; keep counting the coast */
    cal_delta_t d;
    cal_since(&m, &d);

    float went = (c->mm > 0) ? d.path_mm : -d.path_mm;   /* along the move */
    float stop = went - (float)((c->mm > 0) ? c->mm : -c->mm);

    snprintf(u, sizeof u,
             "[CS] %s run=%u result=%s target_mm=%ld went_mm=%.1f stop_mm=%.1f"
             " heading_deg=%.2f side_mm=%.1f encL=%ld encR=%ld\r\n",
             c->name, (unsigned)(rep + 1u), cal_result_name(res), (long)c->mm,
             (double)went, (double)stop, (double)d.turn_deg, (double)d.side_mm,
             (long)d.dl, (long)d.dr);
    command_send(u);

    OLED_Clear();
    cal_line(0,  "%s run %u/%u", c->name, (unsigned)(rep + 1u), (unsigned)CS_REPEATS);
    cal_line(10, "went %.1f mm", (double)went);
    cal_line(20, "stop %+.1f mm", (double)stop);
    cal_line(30, "head %+.2f deg", (double)d.turn_deg);
    cal_line(40, "side %+.1f mm", (double)d.side_mm);

    if (res != MOVE_DONE) {
        /* Not counted; the next tap repeats this run. */
        cal_line(50, "!! %s", cal_result_name(res));
        OLED_Refresh_Gram();
        return;
    }

    cs_stop[ci][rep] = stop;
    cs_head[ci][rep] = d.turn_deg;
    cs_side[ci][rep] = d.side_mm;
    cs_run++;

    if (cs_run >= CS_NCASES * CS_REPEATS) {
        cal_line(50, "SW1 = summary");
        cs_phase = CS_SUM_STOP;
    } else {
        const cs_case_t *n = &CS_CASES[cs_run % CS_NCASES];
        cal_line(50, "SW1: %s %u/%u", n->name,
                 (unsigned)(cs_run / CS_NCASES + 1u), (unsigned)CS_REPEATS);
    }
    OLED_Refresh_Gram();
}

/* ----------------------------------------------------------- CAL_ODOMETER */

static cal_mark_t od_mark;
static uint32_t   od_last_draw = 0;
static uint32_t   od_last_log  = 0;

void cal_odometer_start(void)
{
    motors_coast();             /* free wheel, so the car can be pushed      */
    servo_us(SERVO_CENTRE);
    cal_odometer_zero();
}

void cal_odometer_zero(void)
{
    cal_mark(&od_mark);
    OLED_Clear();
    od_last_draw = HAL_GetTick() - 1000u;   /* redraw on the next poll */
    command_send("[OD] zero\r\n");
}

void cal_odometer_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (now - od_last_draw < 200u) return;
    od_last_draw = now;

    cal_delta_t d;
    cal_since(&od_mark, &d);
    /* COUNTS_PER_REV that would make this push read exactly 1000 mm. */
    float counts = 0.5f * (float)(d.dl + d.dr);
    float cpr    = fabsf(counts) * (3.14159265f * WHEEL_DIA_MM) / 1000.0f;

    cal_line(0,  "ODOMETER  tap=0");
    cal_line(10, "dist %8.1f mm", (double)d.path_mm);
    cal_line(20, "L%6ld  R%6ld", (long)d.dl, (long)d.dr);
    cal_line(30, "head %+7.2f deg", (double)d.turn_deg);
    cal_line(40, "side %+7.1f mm", (double)d.side_mm);
    cal_line(50, "CPR@1m %8.1f", (double)cpr);
    OLED_Refresh_Gram();

    if (now - od_last_log >= 1000u) {
        char u[160];
        od_last_log = now;
        snprintf(u, sizeof u,
                 "[OD] dist_mm=%.1f encL=%ld encR=%ld heading_deg=%.2f side_mm=%.1f"
                 " cpr_if_1m=%.1f\r\n",
                 (double)d.path_mm, (long)d.dl, (long)d.dr, (double)d.turn_deg,
                 (double)d.side_mm, (double)cpr);
        command_send(u);
    }
}

/* --------------------------------------------------------------- CAL_SIX_* */

/* Same order as turn_test.c: 0 FR, 1 FL, 2 BR, 3 BL. */
static const struct {
    const char *name;
    int8_t      left;      /* 1 = left, 0 = right   */
    int8_t      forward;   /* 1 = forward, 0 = back */
} SIX_KINDS[4] = {
    { "FR030", 0, 1 },
    { "FL030", 1, 1 },
    { "BR030", 0, 0 },
    { "BL030", 1, 0 },
};

void cal_six_show_idle(uint8_t kind)
{
    if (kind > 3u) kind = 3u;
    OLED_Clear();
    cal_line(0,  "SIX TURNS %s", SIX_KINDS[kind].name);
    cal_line(10, "mark rear axle");
    cal_line(30, "SW1 = 6 turns");
    if (!calibrated) cal_line(40, "(gyro cal 1st)");
    cal_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

void cal_six_step(uint8_t kind)
{
    char u[160];
    if (kind > 3u) kind = 3u;
    const char *type    = SIX_KINDS[kind].name;
    int8_t      left    = SIX_KINDS[kind].left;
    int8_t      forward = SIX_KINDS[kind].forward;

    cal_gyro_if_needed();

    OLED_Clear();
    cal_line(0,  "6 x %s", type);
    cal_line(20, "running...");
    OLED_Refresh_Gram();
    HAL_Delay(CAL_START_MS);

    cal_mark_t m;
    cal_mark(&m);
    move_result_t res = MOVE_DONE;
    unsigned turns = 0;
    for (int i = 0; i < 6; i++) {
        move_turn_deg(left, forward, 30);
        res = motion_result();
        if (res != MOVE_DONE) break;
        turns++;
        HAL_Delay(250);
    }
    HAL_Delay(500);   /* let the last turn's coast finish */
    cal_delta_t d;
    cal_since(&m, &d);

    float turned = fabsf(d.turn_deg);
    float chord  = sqrtf(d.along_mm * d.along_mm + d.side_mm * d.side_mm);
    float theta  = turned * CAL_DEG2RAD;
    float r_mm   = (theta > 0.01f) ? (chord / (2.0f * sinf(0.5f * theta))) : 0.0f;

    OLED_Clear();
    cal_line(0,  "6 x %s", type);
    cal_line(10, "turned %.1f deg", (double)turned);
    cal_line(20, "target 180.0");
    cal_line(30, "R odo %.0f mm", (double)r_mm);
    cal_line(40, "R tape = gap/2");
    if (turns < 6u) cal_line(50, "!! %s at %u", cal_result_name(res), turns + 1u);
    else            cal_line(50, "SW1 = again");
    OLED_Refresh_Gram();

    snprintf(u, sizeof u,
             "[T30] type=%s turns=%u result=%s turned_deg=%.1f chord_mm=%.1f radius_mm=%.1f\r\n",
             type, turns, cal_result_name(res), (double)turned, (double)chord, (double)r_mm);
    command_send(u);
}

/* ---------------------------------------------------------------- CAL_SEQ */

/* The sequence, written exactly as the Pi sends it, at most SEQ_MAX_MOVES.
   Turn, straight, turn would be { "FL030", "FW100", "FR030" }. */
static const char *const SEQ_MOVES[] = { "FL030", "FR030", "FW100" };

#define SEQ_MAX_MOVES  4u
#define SEQ_N          ((uint8_t)(sizeof SEQ_MOVES / sizeof SEQ_MOVES[0]))
#define SEQ_REPEATS    5u
#define SEQ_GAP_MS     20u   /* between moves: the Pi sends the next command
                                as soon as DONE arrives, so keep it short     */

_Static_assert(sizeof SEQ_MOVES / sizeof SEQ_MOVES[0] <= SEQ_MAX_MOVES,
               "SEQ_MOVES holds at most SEQ_MAX_MOVES moves");

/* One move string, read the way dispatch() in command.c reads it. */
typedef struct {
    uint8_t is_turn;
    int8_t  left;        /* turns: 1 = left, 0 = right                    */
    int8_t  forward;     /* 1 = forward, 0 = back                         */
    int32_t arg;         /* straights: cm, turns: deg                     */
    float   plan_deg;    /* heading change the plan expects, + = left     */
} seq_move_t;

static uint8_t seq_parse(const char *cmd, seq_move_t *mv)
{
    mv->is_turn  = 1u;
    mv->left     = 0;
    mv->forward  = 1;
    mv->arg      = (strlen(cmd) >= 5u) ? (int32_t)atoi(cmd + 2) : 0;
    mv->plan_deg = 0.0f;

    if      (!strncmp(cmd, "FW", 2)) { mv->is_turn = 0u; }
    else if (!strncmp(cmd, "BW", 2)) { mv->is_turn = 0u; mv->forward = 0; }
    else if (!strncmp(cmd, "FL", 2)) { mv->left = 1; }
    else if (!strncmp(cmd, "FR", 2)) { /* the defaults */ }
    else if (!strncmp(cmd, "BL", 2)) { mv->left = 1; mv->forward = 0; }
    else if (!strncmp(cmd, "BR", 2)) { mv->forward = 0; }
    else return 0u;

    if (mv->is_turn) {
        /* Forward-left and back-right both turn the car anticlockwise. */
        mv->plan_deg = (mv->left == mv->forward) ? (float)mv->arg : -(float)mv->arg;
    }
    return 1u;
}

/* Run one move as dispatch() does: clear any old STOP, then the same call. */
static move_result_t seq_run_move(const seq_move_t *mv)
{
    motion_abort_clear();
    if (mv->is_turn) (void)move_turn_deg(mv->left, mv->forward, mv->arg);
    else             (void)move_straight_mm((mv->forward ? 10 : -10) * mv->arg);
    return motion_result();
}

typedef enum { SEQ_RUNNING = 0, SEQ_SUMMARY, SEQ_DONE } seq_phase_t;

static seq_phase_t seq_phase = SEQ_RUNNING;
static uint8_t     seq_run   = 0;                          /* counted runs so far */
static float       seq_main[SEQ_MAX_MOVES][SEQ_REPEATS];   /* turn: angle, straight: tilt */
static float       seq_side[SEQ_MAX_MOVES][SEQ_REPEATS];   /* straight: sideways, mm      */
static float       seq_end[SEQ_REPEATS];                   /* heading off plan at the end */

/* The OLED has room for one "side" line: the last straight's. */
static int8_t seq_last_straight(void)
{
    int8_t last = -1;
    for (uint8_t i = 0; i < SEQ_N; i++) {
        seq_move_t mv;
        if (seq_parse(SEQ_MOVES[i], &mv) && !mv.is_turn) last = (int8_t)i;
    }
    return last;
}

void cal_seq_show_idle(void)
{
    char names[2][17];
    names[0][0] = '\0';
    names[1][0] = '\0';
    for (uint8_t i = 0; i < SEQ_N; i++) {
        char  *line = names[i / 2u];
        size_t used = strlen(line);
        snprintf(line + used, sizeof names[0] - used, "%s%s", (i % 2u) ? " " : "", SEQ_MOVES[i]);
    }

    OLED_Clear();
    if (seq_phase == SEQ_RUNNING) {
        cal_line(0, "SEQ TEST  %u/%u", (unsigned)(seq_run + 1u), (unsigned)SEQ_REPEATS);
    } else {
        cal_line(0, "SEQ TEST");
    }
    cal_line(10, "%s", names[0]);
    cal_line(20, "%s", names[1]);
    cal_line(30, "%s", (seq_phase == SEQ_RUNNING) ? "SW1 = run"
                     : (seq_phase == SEQ_SUMMARY) ? "SW1 = summary" : "SW1 = restart");
    cal_line(40, "%s", calibrated ? "square to a line" : "(gyro cal 1st)");
    if (seq_phase == SEQ_RUNNING) cal_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

static void seq_show_summary(void)
{
    char    u[120];
    float   mean, hr;
    int8_t  ls = seq_last_straight();
    uint8_t y  = 10u;

    OLED_Clear();
    cal_line(0, "SEQ  mean  +-");
    for (uint8_t i = 0; i < SEQ_N; i++) {
        seq_move_t mv;
        if (!seq_parse(SEQ_MOVES[i], &mv)) continue;

        cal_stats(seq_main[i], SEQ_REPEATS, &mean, &hr);
        if (y <= 50u) {
            if (mv.is_turn) {
                cal_line(y, "%s%5.1f +-%3.1f", SEQ_MOVES[i],
                         (double)cal_clamp(mean, 999.9f), (double)cal_clamp(hr, 9.9f));
            } else {
                cal_line(y, "%s%+5.1f +-%3.1f", SEQ_MOVES[i],
                         (double)cal_clamp(mean, 99.9f), (double)cal_clamp(hr, 9.9f));
            }
            y += 10u;
        }
        snprintf(u, sizeof u, "[SEQ] SUMMARY %s %s mean=%.2f +-%.2f\r\n", SEQ_MOVES[i],
                 mv.is_turn ? "angle_deg" : "tilt_deg", (double)mean, (double)hr);
        command_send(u);

        if (!mv.is_turn) {
            cal_stats(seq_side[i], SEQ_REPEATS, &mean, &hr);
            if ((int8_t)i == ls && y <= 50u) {
                cal_line(y, "side %+.0f +-%.0f mm", (double)mean, (double)hr);
                y += 10u;
            }
            snprintf(u, sizeof u, "[SEQ] SUMMARY %s side_mm mean=%.1f +-%.1f\r\n",
                     SEQ_MOVES[i], (double)mean, (double)hr);
            command_send(u);
        }
    }

    cal_stats(seq_end, SEQ_REPEATS, &mean, &hr);
    if (y <= 50u) cal_line(y, "end %+.1f +-%.1f", (double)mean, (double)hr);
    snprintf(u, sizeof u, "[SEQ] SUMMARY end heading_err_deg mean=%.2f +-%.2f\r\n",
             (double)mean, (double)hr);
    command_send(u);
    OLED_Refresh_Gram();
}

void cal_seq_step(void)
{
    char u[200];

    /* --- summary, then back to the start --- */
    if (seq_phase == SEQ_SUMMARY) { seq_show_summary(); seq_phase = SEQ_DONE; return; }
    if (seq_phase == SEQ_DONE) {
        seq_phase = SEQ_RUNNING;
        seq_run   = 0;
        command_send("[SEQ] restart\r\n");
        cal_seq_show_idle();
        return;
    }

    /* --- check the whole list before anything moves --- */
    seq_move_t mv[SEQ_MAX_MOVES];
    for (uint8_t i = 0; i < SEQ_N; i++) {
        if (!seq_parse(SEQ_MOVES[i], &mv[i])) {
            OLED_Clear();
            cal_line(0,  "SEQ_MOVES typo");
            cal_line(20, "!! bad %s", SEQ_MOVES[i]);
            cal_line(40, "fix and reflash");
            OLED_Refresh_Gram();
            snprintf(u, sizeof u, "[SEQ] unknown move %s in SEQ_MOVES\r\n", SEQ_MOVES[i]);
            command_send(u);
            return;
        }
    }

    /* --- one run of the whole sequence --- */
    cal_gyro_if_needed();

    OLED_Clear();
    cal_line(0,  "SEQ run %u/%u", (unsigned)(seq_run + 1u), (unsigned)SEQ_REPEATS);
    cal_line(20, "running...");
    OLED_Refresh_Gram();
    HAL_Delay(CAL_START_MS);

    /* The car was squared up by hand, so it starts on the heading the plan
       assumes; plan then follows the commanded turns. */
    cal_mark_t before, after;
    cal_mark(&before);
    float plan = before.yaw0;

    float         main_v[SEQ_MAX_MOVES] = { 0.0f };
    float         side_v[SEQ_MAX_MOVES] = { 0.0f };
    move_result_t res  = MOVE_DONE;
    uint8_t       done = 0u;   /* moves that finished */

    for (uint8_t i = 0; i < SEQ_N; i++) {
        res = seq_run_move(&mv[i]);
        if (res != MOVE_DONE) break;
        /* The next move follows straight away, as from the Pi. After the
           last one, wait out the coast before reading. */
        HAL_Delay(((uint8_t)(i + 1u) < SEQ_N) ? SEQ_GAP_MS : CAL_SETTLE_MS);
        cal_mark(&after);

        float plan_after = plan + mv[i].plan_deg;
        if (mv[i].is_turn) {
            main_v[i] = fabsf(after.yaw0 - before.yaw0);
            snprintf(u, sizeof u,
                     "[SEQ] run=%u move=%s angle_deg=%.2f heading_err_deg=%.2f\r\n",
                     (unsigned)(seq_run + 1u), SEQ_MOVES[i], (double)main_v[i],
                     (double)(after.yaw0 - plan_after));
        } else {
            /* Against the planned direction: along it, and to its left. */
            float h     = plan * CAL_DEG2RAD;
            float dx    = after.x0 - before.x0;
            float dy    = after.y0 - before.y0;
            float along =  dx * cosf(h) + dy * sinf(h);
            float side  = -dx * sinf(h) + dy * cosf(h);
            float s     = mv[i].forward ? 1.0f : -1.0f;   /* reversing, it faces the other way */
            float went  = s * 0.5f * (float)((after.l0 - before.l0) + (after.r0 - before.r0))
                        * MM_PER_COUNT;
            main_v[i] = atan2f(s * side, s * along) / CAL_DEG2RAD;
            side_v[i] = side;
            snprintf(u, sizeof u,
                     "[SEQ] run=%u move=%s went_mm=%.1f tilt_deg=%.2f side_mm=%.1f"
                     " start_heading_err_deg=%.2f heading_change_deg=%.2f\r\n",
                     (unsigned)(seq_run + 1u), SEQ_MOVES[i], (double)went,
                     (double)main_v[i], (double)side, (double)(before.yaw0 - plan),
                     (double)(after.yaw0 - before.yaw0));
        }
        command_send(u);

        plan   = plan_after;
        before = after;
        done++;
    }
    float end_err = before.yaw0 - plan;   /* after the last move that finished */

    /* --- show it: one line per move, the last straight's side, the end --- */
    OLED_Clear();
    int8_t  ls = seq_last_straight();
    uint8_t y  = 10u;
    for (uint8_t i = 0; i < done && y <= 50u; i++) {
        if (mv[i].is_turn) cal_line(y, "%s ang %.1f", SEQ_MOVES[i], (double)main_v[i]);
        else               cal_line(y, "%s tilt %+.1f", SEQ_MOVES[i], (double)main_v[i]);
        y += 10u;
        if ((int8_t)i == ls && y <= 50u) {
            cal_line(y, "side %+.0f mm", (double)side_v[i]);
            y += 10u;
        }
    }

    if (res != MOVE_DONE) {
        /* Not counted; the next tap repeats this run. */
        const char *what = (done < SEQ_N) ? SEQ_MOVES[done] : "?";
        cal_line(0, "SEQ %u/%u SW1=redo", (unsigned)(seq_run + 1u), (unsigned)SEQ_REPEATS);
        cal_line(50, "!! %s %s", cal_result_name(res), what);
        OLED_Refresh_Gram();
        snprintf(u, sizeof u, "[SEQ] run=%u not counted: %s at %s\r\n",
                 (unsigned)(seq_run + 1u), cal_result_name(res), what);
        command_send(u);
        return;
    }

    if (y <= 50u) cal_line(y, "end head %+.1f", (double)end_err);
    snprintf(u, sizeof u, "[SEQ] run=%u end heading_err_deg=%.2f\r\n",
             (unsigned)(seq_run + 1u), (double)end_err);
    command_send(u);

    for (uint8_t i = 0; i < SEQ_N; i++) {
        seq_main[i][seq_run] = main_v[i];
        seq_side[i][seq_run] = side_v[i];
    }
    seq_end[seq_run] = end_err;
    seq_run++;

    if (seq_run >= SEQ_REPEATS) {
        cal_line(0, "SEQ %u/%u SW1=sum", (unsigned)seq_run, (unsigned)SEQ_REPEATS);
        seq_phase = SEQ_SUMMARY;
    } else {
        cal_line(0, "SEQ %u/%u SW1=next", (unsigned)seq_run, (unsigned)SEQ_REPEATS);
    }
    OLED_Refresh_Gram();
}

/* -------------------------------------------------------------- CAL_TRACE */

#define TR_MM          1000   /* the move, mm (FW100). Long enough for every phase */
#define TR_PRINT_TICKS 1      /* 1 = also print every tick on USART3 ("[TRS]")     */
#define TR_KEEP        500u   /* ticks kept for that printout: the last 5 s        */

/* Phases of a straight, told apart by what the commanded speed (the ramp)
   is doing: rising, flat at cruise, falling while braking, flat at crawl. */
enum { TR_ACC = 0, TR_CRU, TR_DEC, TR_CRL, TR_NPH };
static const char *const TR_NAME[TR_NPH] = { "acc", "cru", "dec", "crl" };

typedef struct {
    uint16_t ticks;
    int32_t  l, r;             /* wheel counts, summed (absolute)              */
    float    target;           /* commanded speed, summed (absolute)           */
    float    head_in, head_out;/* heading entering and leaving the phase       */
    int32_t  steer;            /* servo minus centre, summed, us (+ = right)   */
    int32_t  duty_l, duty_r;   /* motor duty, summed                           */
    uint16_t l_stall, r_stall; /* ticks one wheel stood while the other turned */
} tr_phase_t;

typedef struct {
    int8_t  l, r;
    uint8_t ph;
    int16_t ramp10, head100, steer, duty_l, duty_r;
} tr_sample_t;

static tr_phase_t        tr_ph[TR_NPH];
static tr_sample_t       tr_buf[TR_KEEP];
static volatile uint16_t tr_n;               /* ticks seen in this move          */
static float             tr_prev_ramp, tr_prev_head;
static float             tr_i_l, tr_i_r;     /* integral part of the duty, last tick */
static uint8_t           tr_page = 0;        /* next tap: 0 drive, 1 page 2, 2 page 3 */
static uint16_t          tr_run  = 0;
static move_result_t     tr_res  = MOVE_NONE;
static float             tr_total, tr_stop;  /* heading change: whole move, after the brake */

static int32_t tr_abs(int32_t v) { return (v < 0) ? -v : v; }

static int16_t tr_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)v;
}

/* Runs in the 100 Hz interrupt on every tick of the traced straight. */
static void tr_hook(const straight_tick_t *t)
{
    float   v  = fabsf(t->ramp);
    float   pv = fabsf(tr_prev_ramp);
    int32_t al = tr_abs(t->left), ar = tr_abs(t->right);
    uint8_t ph;
    if (t->braking) ph = (v < pv - 0.001f) ? TR_DEC : TR_CRL;
    else            ph = (v > pv + 0.001f) ? TR_ACC : TR_CRU;

    tr_phase_t *p = &tr_ph[ph];
    if (p->ticks == 0u) p->head_in = tr_prev_head;
    p->ticks++;
    p->l       += al;
    p->r       += ar;
    p->target  += v;
    p->head_out = t->head_deg;
    p->steer   += t->steer_us;
    p->duty_l  += t->duty_l;
    p->duty_r  += t->duty_r;
    if (al <= 1 && ar >= 3) p->l_stall++;
    if (ar <= 1 && al >= 3) p->r_stall++;

    tr_sample_t *s = &tr_buf[tr_n % TR_KEEP];
    s->l       = (int8_t)((t->left  > 127) ? 127 : (t->left  < -127) ? -127 : t->left);
    s->r       = (int8_t)((t->right > 127) ? 127 : (t->right < -127) ? -127 : t->right);
    s->ph      = ph;
    s->ramp10  = tr_i16(t->ramp * 10.0f);
    s->head100 = tr_i16(t->head_deg * 100.0f);
    s->steer   = t->steer_us;
    s->duty_l  = tr_i16((float)t->duty_l);
    s->duty_r  = tr_i16((float)t->duty_r);
    if (tr_n < 65535u) tr_n++;

    tr_prev_ramp = t->ramp;
    tr_prev_head = t->head_deg;
    tr_i_l = t->i_l;
    tr_i_r = t->i_r;
}

static void tr_title(uint8_t page)
{
    if (tr_res == MOVE_DONE) {
        cal_line(0, "TR %u FW%03ld %u/3", (unsigned)tr_run, (long)(TR_MM / 10), (unsigned)page);
    } else {
        cal_line(0, "TR%u !%s %u/3", (unsigned)tr_run, cal_result_name(tr_res), (unsigned)page);
    }
}

/* Page 1: each wheel's speed as a % of the commanded speed, and how much the
   heading changed, phase by phase. */
static void tr_page1(void)
{
    OLED_Clear();
    tr_title(1);
    cal_line(10, "     L%%  R%%  hdg");
    for (uint8_t i = 0; i < TR_NPH; i++) {
        const tr_phase_t *p = &tr_ph[i];
        uint8_t y = (uint8_t)(20u + 10u * i);
        if (p->ticks == 0u || p->target < 0.5f) {
            cal_line(y, "%s   -   -    -", TR_NAME[i]);
            continue;
        }
        float lp = 100.0f * (float)p->l / p->target;
        float rp = 100.0f * (float)p->r / p->target;
        cal_line(y, "%s%4.0f%4.0f%+5.1f", TR_NAME[i], (double)cal_clamp(lp, 999.0f),
                 (double)cal_clamp(rp, 999.0f), (double)cal_clamp(p->head_out - p->head_in, 99.9f));
    }
    OLED_Refresh_Gram();
}

/* Page 2: the crawl, what the steering did in it, stalls, and the heading. */
static void tr_page2(void)
{
    const tr_phase_t *c = &tr_ph[TR_CRL];
    float    crawl_mm = 0.5f * (float)(c->l + c->r) * MM_PER_COUNT;
    float    steer    = c->ticks ? (float)c->steer / (float)c->ticks : 0.0f;
    unsigned ls = 0u, rs = 0u;
    for (uint8_t i = 0; i < TR_NPH; i++) { ls += tr_ph[i].l_stall; rs += tr_ph[i].r_stall; }

    OLED_Clear();
    tr_title(2);
    cal_line(10, "crawl %.0fmm %.1fs", (double)crawl_mm, (double)((float)c->ticks * 0.01f));
    cal_line(20, "crl steer %c %.0fus", (steer >= 0.0f) ? 'R' : 'L', (double)fabsf(steer));
    cal_line(30, "stall L%3u R%3u", ls, rs);
    cal_line(40, "stop hdg %+.1f", (double)tr_stop);
    cal_line(50, "total hdg %+.1f", (double)tr_total);
    OLED_Refresh_Gram();
}

/* Page 3: what the speed loop asked of each motor. */
static void tr_page3(void)
{
    OLED_Clear();
    tr_title(3);
    cal_line(10, "duty     L     R");
    const uint8_t which[2] = { TR_CRU, TR_CRL };
    for (uint8_t k = 0; k < 2u; k++) {
        const tr_phase_t *p = &tr_ph[which[k]];
        uint8_t y = (uint8_t)(20u + 10u * k);
        if (p->ticks == 0u) { cal_line(y, "%-4s     -     -", TR_NAME[which[k]]); continue; }
        cal_line(y, "%-4s%6.0f%6.0f", TR_NAME[which[k]],
                 (double)((float)p->duty_l / (float)p->ticks), (double)((float)p->duty_r / (float)p->ticks));
    }
    cal_line(40, "Iend%6.0f%6.0f", (double)tr_i_l, (double)tr_i_r);
    cal_line(50, "full = %d", (int)PWM_MAX);
    OLED_Refresh_Gram();
}

static void tr_print(void)
{
    char u[200];
    snprintf(u, sizeof u,
             "[TR] run=%u mm=%d result=%s ticks=%u total_hdg_deg=%.2f stop_hdg_deg=%.2f"
             " i_end_l=%.0f i_end_r=%.0f\r\n",
             (unsigned)tr_run, (int)TR_MM, cal_result_name(tr_res), (unsigned)tr_n,
             (double)tr_total, (double)tr_stop, (double)tr_i_l, (double)tr_i_r);
    command_send(u);
    for (uint8_t i = 0; i < TR_NPH; i++) {
        const tr_phase_t *p = &tr_ph[i];
        if (p->ticks == 0u) continue;
        float n = (float)p->ticks;
        snprintf(u, sizeof u,
                 "[TR] phase=%s ticks=%u mm=%.1f target=%.1f left=%.1f right=%.1f"
                 " hdg_deg=%.2f steer_us=%.0f duty_l=%.0f duty_r=%.0f lstall=%u rstall=%u\r\n",
                 TR_NAME[i], (unsigned)p->ticks, (double)(0.5f * (float)(p->l + p->r) * MM_PER_COUNT),
                 (double)(p->target / n), (double)((float)p->l / n), (double)((float)p->r / n),
                 (double)(p->head_out - p->head_in), (double)((float)p->steer / n),
                 (double)((float)p->duty_l / n), (double)((float)p->duty_r / n),
                 (unsigned)p->l_stall, (unsigned)p->r_stall);
        command_send(u);
    }
#if TR_PRINT_TICKS
    command_send("[TRS] tick,phase,ramp,left,right,hdg_deg,steer_us,duty_l,duty_r\r\n");
    uint16_t n     = tr_n;
    uint16_t first = (n > TR_KEEP) ? (uint16_t)(n - TR_KEEP) : 0u;
    for (uint16_t i = first; i < n; i++) {
        const tr_sample_t *s = &tr_buf[i % TR_KEEP];
        snprintf(u, sizeof u, "[TRS] %u,%s,%.1f,%d,%d,%.2f,%d,%d,%d\r\n",
                 (unsigned)i, TR_NAME[s->ph], (double)s->ramp10 / 10.0, (int)s->l, (int)s->r,
                 (double)s->head100 / 100.0, (int)s->steer, (int)s->duty_l, (int)s->duty_r);
        command_send(u);
    }
#endif
}

void cal_trace_show_idle(void)
{
    tr_page = 0u;
    OLED_Clear();
    cal_line(0,  "STRAIGHT TRACE");
    cal_line(10, "FW%03ld, 3 pages", (long)(TR_MM / 10));
    cal_line(20, "SW1 = drive");
    cal_line(30, "then SW1 = pages");
    cal_line(40, "%s", calibrated ? "needs 1.3 m" : "(gyro cal 1st)");
    cal_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

void cal_trace_step(void)
{
    if (tr_page == 1u) { tr_page2(); tr_page = 2u; return; }
    if (tr_page == 2u) { tr_page3(); tr_page = 0u; return; }

    cal_gyro_if_needed();

    OLED_Clear();
    cal_line(0,  "TR %u FW%03ld", (unsigned)(tr_run + 1u), (long)(TR_MM / 10));
    cal_line(20, "running...");
    OLED_Refresh_Gram();
    HAL_Delay(CAL_START_MS);

    memset(tr_ph, 0, sizeof tr_ph);
    tr_n         = 0u;
    tr_prev_ramp = 0.0f;
    tr_prev_head = 0.0f;
    tr_i_l = tr_i_r = 0.0f;

    /* The same call a FW from the Pi makes, with the hook watching it. */
    float yaw0 = motion_yaw_deg();
    motion_abort_clear();
    straight_tick_hook = tr_hook;
    move_straight_mm(TR_MM);
    straight_tick_hook = 0;
    tr_res = motion_result();
    HAL_Delay(CAL_SETTLE_MS);           /* brake on, as in a run; catch the coast */
    tr_total = motion_yaw_deg() - yaw0;
    tr_stop  = tr_total - tr_prev_head; /* what changed after the last driven tick */
    tr_run++;

    tr_page1();
    tr_print();
    tr_page = 1u;
}
