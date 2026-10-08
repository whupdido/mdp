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
 * Nothing here changes how the car drives. As in a real run the brake stays
 * on after each move, and the readings keep running for CAL_SETTLE_MS
 * afterwards so the coast is included. Every result also goes to USART3:
 * "[CS]" straight, "[OD]" odometer, "[T30]" six turns.
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
    { "FW100",  1000 },
};
#define CS_NCASES   4u
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
