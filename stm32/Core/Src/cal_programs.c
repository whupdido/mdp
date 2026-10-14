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
 * CAL_TASK2     Kush (09-Oct): the whole Task 2 with no Pi and no image
 *               recognition, for calibrating the run itself. The arrows are
 *               CAL_T2_ARROW_OBS1 / CAL_T2_ARROW_OBS2 in main.c. A tap:
 *                 1. waits CAL_START_MS so your hand is off the car,
 *                 2. zeroes the gyro hands-off, the same GC the Pi runs in its
 *                    pre-flight (about 2 s; it refuses if the car moved),
 *                 3. runs task_2() exactly as START2 does, except that each
 *                    arrow is taken as given at the standoff instead of being
 *                    scanned (no SCAN, no wait, no retries),
 *                 4. shows how long the run took.
 *               SW1 at any point after the tap stops the car (the same
 *               motion_stop() a STOP from the Pi calls). The time leaves out
 *               the scans a real run waits for, so a real run is slower by
 *               however long the Pi takes to answer, twice. "[T2]" lines for
 *               every phase and a "[T2L]" summary go to USART3 as usual.
 *
 * CAL_IR        Kush (14-Oct): the two side IRs. Nothing drives. You need a
 *               flat board (an obstacle face will do), a tape and the
 *               carpark (or two walls IRC_GAP_CM apart).
 *                 1. Stand the board upright beside the car, parallel to
 *                    its LEFT side, the distance the OLED asks for from the
 *                    left IR's lens (IRC_POINTS_CM: 10 to 70 cm), measured
 *                    square to the board, and tap; one tap per distance. It
 *                    averages that IR for 1 s and shows what it reads now.
 *                 2. After the last one it fits V = m/(d + k) + b to that
 *                    side and shows the fit's worst error (w, cm at cm) and
 *                    calib.h's ("now w"). USART3 gets every point, both
 *                    errors, and the three #define lines for calib.h.
 *                 3. The same for the RIGHT side.
 *                 4. Put the car in the carpark, square to the walls,
 *                    anywhere across, and tap: from both readings it works
 *                    out IR_TO_CENTRE_MM (each IR's lens to the car's centre
 *                    line), which T2_PARK_DIRECT needs. With the new fits, and
 *                    with calib.h's in brackets: use the one that goes with
 *                    the IR values you keep.
 *                 5. Live view: both sides now (calib.h) and with the new
 *                    fits, so a board moved about can be checked against the
 *                    tape. A tap starts over; so does a 2 s hold.
 *               Paste the new values only if the fit's errors are clearly
 *               smaller than calib.h's, mostly at 40-70 cm. IRC_DO_SWEEP 0
 *               goes straight to step 4 with the calib.h values.
 *
 * Nothing here changes how the car drives. As in a real run the brake stays
 * on after each move, and the readings keep running for CAL_SETTLE_MS
 * afterwards so the coast is included. Every result also goes to USART3:
 * "[CS]" straight, "[OD]" odometer, "[T30]" six turns, "[SEQ]" sequence,
 * "[TR]"/"[TRS]" trace, "[IRC]" IR calibration.
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
#include "obstacle_nav.h"
#include "sensors.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CAL_SETTLE_MS   700u   /* keep reading after a move: the coast           */
#define CAL_START_MS    800u   /* after the tap, so your hand is off the car     */
#define CAL_DEG2RAD     (3.14159265f / 180.0f)

extern int calibrated;         /* main.c: set once the gyro bias is locked       */
 extern uint8_t  t2_direct_used;
 extern float    t2_direct_side_cm;
 extern uint16_t t2_direct_n;


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

/* -------------------------------------------------------------- CAL_TASK2 */

static uint8_t t2l_runs = 0u;            /* finished runs since power-up        */

/* SW1 as STOP during a run, from the 100 Hz interrupt (control_tick_hook),
   so a press is seen even while the car sits in a HAL_Delay() between moves.
   Down on T2L_STOP_TICKS ticks in a row (10-20 ms), so a glitch on the line
   cannot stop a good run. */
#define T2L_STOP_TICKS  2u
static volatile uint8_t t2l_low_ticks = 0u;   /* ticks in a row SW1 was down */
static volatile uint8_t t2l_sw1       = 0u;   /* this run was stopped by SW1 */

static void t2l_sw1_tick(void)                /* interrupt context */
{
    if (HAL_GPIO_ReadPin(BTN_USER_GPIO_Port, BTN_USER_Pin) != GPIO_PIN_RESET) {
        t2l_low_ticks = 0u;
        return;
    }
    if (t2l_low_ticks < T2L_STOP_TICKS) t2l_low_ticks++;
    if (t2l_low_ticks >= T2L_STOP_TICKS && !t2l_sw1) {
        t2l_sw1 = 1u;
        motion_stop();
    }
}

/* HAL_Delay that ends early once stopped, and keeps the UART answered. */
static void t2l_wait_ms(uint32_t ms)
{
    const uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms && !motion_abort_requested()) {
        command_poll();
        HAL_Delay(2);
    }
}

/* Returns once SW1 has been up for 60 ms, the same release main.c waits for,
   so the press that stopped a run is not then taken as a tap to start one. */
static void t2l_wait_release(void)
{
    uint32_t high_since = HAL_GetTick();
    while (HAL_GetTick() - high_since < 60u) {
        if (HAL_GPIO_ReadPin(BTN_USER_GPIO_Port, BTN_USER_Pin) == GPIO_PIN_RESET) {
            high_since = HAL_GetTick();
        }
        HAL_Delay(2);
    }
}

static const char *t2l_name(uint8_t arrow)
{
    if (arrow == T2_ARROW_LEFT)  return "LEFT";
    if (arrow == T2_ARROW_RIGHT) return "RIGHT";
    return "?";
}

static char t2l_letter(uint8_t arrow)
{
    return (arrow == T2_ARROW_LEFT) ? 'L' : (arrow == T2_ARROW_RIGHT) ? 'R' : '?';
}

void cal_task2_show_idle(uint8_t arrow_obs1, uint8_t arrow_obs2)
{
    OLED_Clear();
    cal_line(0,  "TASK 2, NO PI");
    cal_line(10, "obs1 arrow %s", t2l_name(arrow_obs1));
    cal_line(20, "obs2 arrow %s", t2l_name(arrow_obs2));
    cal_line(30, "SW1 = run");
    cal_line(40, "SW1 again = STOP");
    cal_line(50, "CAR WILL MOVE");
    OLED_Refresh_Gram();
}

void cal_task2_step(uint8_t arrow_obs1, uint8_t arrow_obs2)
{
    char u[112];
    const unsigned run = (unsigned)t2l_runs + 1u;
    const char a1 = t2l_letter(arrow_obs1), a2 = t2l_letter(arrow_obs2);

    /* From here on SW1 means STOP. */
    motion_abort_clear();
    t2l_low_ticks = 0u;
    t2l_sw1       = 0u;
    control_tick_hook = t2l_sw1_tick;

    OLED_Clear();
    cal_line(0,  "TASK 2 run %u", run);
    cal_line(10, "obs1 %c, obs2 %c", a1, a2);
    cal_line(30, "hands off...");
    cal_line(50, "SW1 = STOP");
    OLED_Refresh_Gram();
    t2l_wait_ms(CAL_START_MS);

    /* The Pi zeroes the gyro hands-off before every real run (GC); do the
       same, so the run is calibrated under the same conditions. */
    uint8_t zero_ok = 0u;
    if (!motion_abort_requested()) zero_ok = command_gyro_zero();

    uint32_t ms = 0u;
    uint8_t  ran = 0u;
    if (zero_ok && !motion_abort_requested()) {
        OLED_Clear();
        cal_line(0,  "TASK 2 run %u", run);
        cal_line(10, "obs1 %c, obs2 %c", a1, a2);
        cal_line(20, "RUNNING");
        cal_line(50, "SW1 = STOP");
        OLED_Refresh_Gram();
        snprintf(u, sizeof u, "[T2L] run %u start obs1=%s obs2=%s\r\n",
                 run, t2l_name(arrow_obs1), t2l_name(arrow_obs2));
        command_send(u);

        const uint32_t t0 = HAL_GetTick();
        command_task2_local(arrow_obs1, arrow_obs2);
        ms  = HAL_GetTick() - t0;
        ran = 1u;
    }

    control_tick_hook = 0;
    const uint8_t stopped = motion_abort_requested();

    if (stopped) {
        OLED_Clear();
        cal_line(0,  "TASK 2 run %u", run);
        cal_line(10, "STOPPED%s", t2l_sw1 ? " (SW1)" : "");
        if (ran) cal_line(20, "after %.1f s", (double)ms * 0.001);
        cal_line(50, "SW1 = run again");
        OLED_Refresh_Gram();
        snprintf(u, sizeof u, "[T2L] run %u stopped%s after %lu ms\r\n",
                 run, t2l_sw1 ? " by SW1" : "", (unsigned long)ms);
        command_send(u);
    } else if (!ran) {
        /* The gyro zero refused (the car moved): its screen says why. Nothing
           drove. */
        cal_line(50, "SW1 = try again");
        OLED_Refresh_Gram();
        snprintf(u, sizeof u, "[T2L] run %u not started: gyro zero failed\r\n", run);
        command_send(u);
    } else {
        t2l_runs++;
        OLED_Clear();
        cal_line(0,  "TASK 2 run %u", run);
        cal_line(10, "DONE %.2f s", (double)ms * 0.001);
        cal_line(20, "direct=%u s=%.1fcm n=%u",
                 (unsigned)t2_direct_used, (double)t2_direct_side_cm,
                 (unsigned)t2_direct_n);
        cal_line(30, "obs1 %c, obs2 %c", a1, a2);
        cal_line(50, "SW1 = run again");
        OLED_Refresh_Gram();
        snprintf(u, sizeof u, "[T2L] run %u done in %lu ms (no scan time) obs1=%s obs2=%s\r\n",
                 run, (unsigned long)ms, t2l_name(arrow_obs1), t2l_name(arrow_obs2));
        command_send(u);
    }

    t2l_wait_release();
    motion_abort_clear();
}

/* ----------------------------------------------------------------- CAL_IR */

/* Distances each side is read at, cm from the IR's lens to the board,
   nearest first. Edit freely (3 to IRC_MAX_PTS of them). 40-70 is where
   T2_PARK_DIRECT reads obstacle 1; the near ones are the parking checks. */
static const float IRC_POINTS_CM[] = { 10.0f, 15.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f };
#define IRC_NPTS      ((uint8_t)(sizeof IRC_POINTS_CM / sizeof IRC_POINTS_CM[0]))
#define IRC_MAX_PTS   16u
#define IRC_SAMPLES   100u     /* voltages averaged per reading, 10 ms apart: 1 s */
#define IRC_GAP_CM    60.0f    /* carpark, inside of one wall to the other        */
#define IRC_K_MIN     (-5.0f)  /* range searched for the fit's k, cm              */
#define IRC_K_MAX     10.0f
/* 1 = both sweeps, then the carpark. 0 = straight to the carpark step and the
   live readings, with the calib.h model only (to measure IR_TO_CENTRE_MM or
   check the IRs without fitting them again). */
#define IRC_DO_SWEEP  1

_Static_assert(sizeof IRC_POINTS_CM / sizeof IRC_POINTS_CM[0] >= 3u &&
               sizeof IRC_POINTS_CM / sizeof IRC_POINTS_CM[0] <= IRC_MAX_PTS,
               "IRC_POINTS_CM needs 3 to IRC_MAX_PTS distances");

typedef struct {
    float   m, b, k;            /* V = m / (d + k) + b                           */
    float   rms, max_err, max_at;
    uint8_t ok;
} irc_fit_t;

typedef enum { IRC_SWEEP = 0, IRC_GAP, IRC_GAP_SHOWN, IRC_LIVE } irc_phase_t;

static irc_phase_t irc_phase = IRC_SWEEP;
static uint8_t     irc_side  = IR_SIDE_LEFT;       /* the side being swept      */
static uint8_t     irc_i     = 0u;                 /* its next point            */
static float       irc_v[2][IRC_MAX_PTS];          /* volts, [side][point]      */
static irc_fit_t   irc_fit[2];                     /* [IR_SIDE_RIGHT/_LEFT]     */
static uint32_t    irc_last_draw = 0u, irc_last_log = 0u;

static char irc_letter(uint8_t side) { return (side == IR_SIDE_LEFT) ? 'L' : 'R'; }
static const char *irc_name(uint8_t side) { return (side == IR_SIDE_LEFT) ? "LEFT" : "RIGHT"; }

/* d = m / (V - b) - k. clamp = 1: limited exactly as sensors.c does it, so
   what the car would read; 0: the model itself, for the fit. */
static float irc_model(float m, float b, float k, float v, uint8_t clamp)
{
    const float den = v - b;
    if (den < 0.01f) return clamp ? IR_MAX_CM : 1000.0f;
    float d = m / den - k;
    if (clamp && d > IR_MAX_CM) d = IR_MAX_CM;
    if (clamp && d < IR_MIN_CM) d = IR_MIN_CM;
    return d;
}

static float irc_model_cm(float m, float b, float k, float v) { return irc_model(m, b, k, v, 1u); }

/* What the car reads now, with the calib.h values. */
static float irc_now_cm(uint8_t side, float v)
{
    if (side == IR_SIDE_LEFT) return irc_model_cm(IR_LEFT_M, IR_LEFT_B, IR_LEFT_K, v);
    return irc_model_cm(IR_RIGHT_M, IR_RIGHT_B, IR_RIGHT_K, v);
}

/* What it would read with this side's new fit. */
static float irc_fit_cm(uint8_t side, float v)
{
    const irc_fit_t *f = &irc_fit[side];
    return irc_model_cm(f->m, f->b, f->k, v);
}

/* Mean and spread of both sensors' voltages over IRC_SAMPLES readings. */
static void irc_measure(float mean[2], float sd[2])
{
    float sum[2] = { 0.0f, 0.0f }, sq[2] = { 0.0f, 0.0f };
    for (uint16_t i = 0u; i < IRC_SAMPLES; i++) {
        for (uint8_t s = 0u; s < 2u; s++) {
            const float v = ir_voltage(s);
            sum[s] += v;
            sq[s]  += v * v;
        }
        HAL_Delay(10);
    }
    for (uint8_t s = 0u; s < 2u; s++) {
        mean[s] = sum[s] / (float)IRC_SAMPLES;
        const float var = sq[s] / (float)IRC_SAMPLES - mean[s] * mean[s];
        sd[s] = (var > 0.0f) ? sqrtf(var) : 0.0f;
    }
}

/* For one k: m and b from a straight-line fit of V against 1/(d + k), each
   point weighted by (d + k)^4. A voltage error at distance d is a distance
   error (d + k)^2 / m times as big, so the weights make the fit about the
   distance errors, not the voltage ones (far points would barely count). */
static uint8_t irc_fit_mb(const float *v, float k, float *m, float *b)
{
    double sw = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (uint8_t i = 0u; i < IRC_NPTS; i++) {
        const double dk = (double)IRC_POINTS_CM[i] + (double)k;
        if (dk < 1.0) return 0u;
        const double x = 1.0 / dk, w = dk * dk * dk * dk;
        sw += w; sx += w * x; sy += w * v[i]; sxx += w * x * x; sxy += w * x * v[i];
    }
    const double den = sw * sxx - sx * sx;
    if (den <= 1e-9 * sw * sxx) return 0u;
    const double mm = (sw * sxy - sx * sy) / den;
    if (mm <= 0.0) return 0u;
    *m = (float)mm;
    *b = (float)((sy - mm * sx) / sw);
    return 1u;
}

/* RMS and worst distance error of a model over one side's points (clamp as
   in irc_model()). */
static float irc_errors(const float *v, float m, float b, float k, uint8_t clamp,
                        float *max_err, float *max_at)
{
    float sq = 0.0f;
    *max_err = 0.0f;
    *max_at  = IRC_POINTS_CM[0];
    for (uint8_t i = 0u; i < IRC_NPTS; i++) {
        const float e = irc_model(m, b, k, v[i], clamp) - IRC_POINTS_CM[i];
        sq += e * e;
        if (fabsf(e) > fabsf(*max_err)) { *max_err = e; *max_at = IRC_POINTS_CM[i]; }
    }
    return sqrtf(sq / (float)IRC_NPTS);
}

/* The k with the smallest RMS distance error: every 0.5 cm over
   IRC_K_MIN..IRC_K_MAX, then every 0.02 cm around the best of those. Chosen
   on the model's own errors (the clamp would let the 10 cm point read 8 for
   free); the errors kept for the report are what the car would read. */
static void irc_fit_side(uint8_t side)
{
    irc_fit_t best = { 0 };
    best.rms = 1e9f;
    for (uint8_t pass = 0u; pass < 2u; pass++) {
        if (pass && !best.ok) break;
        const float lo   = pass ? best.k - 0.5f : IRC_K_MIN;
        const float step = pass ? 0.02f : 0.5f;
        const int   n    = pass ? 50 : (int)((IRC_K_MAX - IRC_K_MIN) / 0.5f + 0.5f);
        for (int j = 0; j <= n; j++) {
            const float k = lo + (float)j * step;
            float m, b, me, ma;
            if (!irc_fit_mb(irc_v[side], k, &m, &b)) continue;
            const float r = irc_errors(irc_v[side], m, b, k, 0u, &me, &ma);
            if (r < best.rms) {
                best = (irc_fit_t){ .m = m, .b = b, .k = k, .rms = r, .ok = 1u };
            }
        }
    }
    if (best.ok) best.rms = irc_errors(irc_v[side], best.m, best.b, best.k, 1u, &best.max_err, &best.max_at);
    irc_fit[side] = best;
}

static void irc_prompt_point(uint8_t y)
{
    cal_line(y,      "next %c %.0f cm", irc_letter(irc_side), (double)IRC_POINTS_CM[irc_i]);
    cal_line(y + 10, "board, then tap");
}

static void irc_prompt_gap(uint8_t y)
{
    cal_line(y,      "next: carpark");
    cal_line(y + 10, "square, then tap");
}

void cal_ir_show_idle(void)
{
    irc_phase = IRC_DO_SWEEP ? IRC_SWEEP : IRC_GAP;
    irc_side  = IR_SIDE_LEFT;
    irc_i     = 0u;
    irc_fit[0].ok = irc_fit[1].ok = 0u;

    OLED_Clear();
    if (irc_phase == IRC_SWEEP) {
        cal_line(0,  "IR CAL  no drive");
        cal_line(10, "flat board, side");
        cal_line(20, "parallel, from");
        cal_line(30, "the IR's lens:");
        irc_prompt_point(40);
    } else {
        cal_line(0,  "IR CAL  no drive");
        cal_line(10, "car square in");
        cal_line(20, "the carpark,");
        cal_line(30, "walls %.0f cm", (double)IRC_GAP_CM);
        cal_line(50, "tap = read");
    }
    OLED_Refresh_Gram();
    command_send("[IRC] start\r\n");
}

static void irc_sweep_tap(void)
{
    char u[200];
    float mean[2], sd[2];
    const uint8_t s = irc_side, i = irc_i;
    const float d = IRC_POINTS_CM[i];

    OLED_Clear();
    cal_line(0,  "%c %.0f cm", irc_letter(s), (double)d);
    cal_line(10, "reading 1 s...");
    OLED_Refresh_Gram();
    irc_measure(mean, sd);
    irc_v[s][i] = mean[s];

    const float now = irc_now_cm(s, mean[s]);
    snprintf(u, sizeof u, "[IRC] %s %.0fcm: %.4f V (sd %.1f mV), reads %.1f cm now (%+.1f)\r\n",
             irc_name(s), (double)d, (double)mean[s], (double)(sd[s] * 1000.0f),
             (double)now, (double)(now - d));
    command_send(u);

    irc_i++;
    if (irc_i < IRC_NPTS) {
        OLED_Clear();
        cal_line(0,  "%c %.0f cm  %u/%u", irc_letter(s), (double)d, (unsigned)irc_i, (unsigned)IRC_NPTS);
        cal_line(10, "%.3f V sd %.0fmV", (double)mean[s], (double)(sd[s] * 1000.0f));
        cal_line(20, "reads %.1f now", (double)now);
        irc_prompt_point(40);
        OLED_Refresh_Gram();
        return;
    }

    /* That side is done: fit it, and compare with what calib.h gives now. */
    irc_fit_side(s);
    const irc_fit_t *f = &irc_fit[s];
    float now_max, now_at;
    const float m_now = (s == IR_SIDE_LEFT) ? IR_LEFT_M : IR_RIGHT_M;
    const float b_now = (s == IR_SIDE_LEFT) ? IR_LEFT_B : IR_RIGHT_B;
    const float k_now = (s == IR_SIDE_LEFT) ? IR_LEFT_K : IR_RIGHT_K;
    const float now_rms = irc_errors(irc_v[s], m_now, b_now, k_now, 1u, &now_max, &now_at);

    command_send("[IRC]   cm     volts   now_cm   err  fit_cm   err\r\n");
    for (uint8_t j = 0u; j < IRC_NPTS; j++) {
        const float a = irc_now_cm(s, irc_v[s][j]);
        const float c = f->ok ? irc_fit_cm(s, irc_v[s][j]) : 0.0f;
        snprintf(u, sizeof u, "[IRC] %5.1f  %7.4f  %6.1f %+5.1f  %6.1f %+5.1f\r\n",
                 (double)IRC_POINTS_CM[j], (double)irc_v[s][j], (double)a,
                 (double)(a - IRC_POINTS_CM[j]), (double)c, (double)(c - IRC_POINTS_CM[j]));
        command_send(u);
    }
    snprintf(u, sizeof u, "[IRC] %s calib.h now: rms %.2f cm, worst %+.1f cm at %.0f cm\r\n",
             irc_name(s), (double)now_rms, (double)now_max, (double)now_at);
    command_send(u);

    if (f->ok) {
        snprintf(u, sizeof u, "[IRC] %s fit: m %.3f b %.4f k %.2f, rms %.2f cm, worst %+.1f cm at %.0f cm\r\n",
                 irc_name(s), (double)f->m, (double)f->b, (double)f->k, (double)f->rms,
                 (double)f->max_err, (double)f->max_at);
        command_send(u);
        char nm[3][16];
        snprintf(nm[0], sizeof nm[0], "IR_%s_M", irc_name(s));
        snprintf(nm[1], sizeof nm[1], "IR_%s_B", irc_name(s));
        snprintf(nm[2], sizeof nm[2], "IR_%s_K", irc_name(s));
        snprintf(u, sizeof u, "[IRC] for calib.h:\r\n#define %-20s%.3ff\r\n#define %-20s%.4ff\r\n"
                              "#define %-20s%.2ff\r\n",
                 nm[0], (double)f->m, nm[1], (double)f->b, nm[2], (double)f->k);
        command_send(u);
    } else {
        snprintf(u, sizeof u, "[IRC] %s: no fit (do the readings fall as the board moves away?)\r\n",
                 irc_name(s));
        command_send(u);
    }

    /* Fit, its worst point, and calib.h's worst point for comparison. */
    OLED_Clear();
    if (f->ok) {
        cal_line(0,  "%s fit rms%.1f", irc_name(s), (double)f->rms);
        cal_line(10, "m%.2f b%.3f", (double)f->m, (double)f->b);
        cal_line(20, "k%+.2f w%+.1f@%.0f", (double)f->k, (double)f->max_err, (double)f->max_at);
    } else {
        cal_line(0,  "%s: NO FIT", irc_name(s));
        cal_line(10, "see USART3");
    }
    cal_line(30, "now w%+.1f@%.0f", (double)now_max, (double)now_at);

    if (s == IR_SIDE_LEFT) {
        irc_side = IR_SIDE_RIGHT;
        irc_i    = 0u;
        irc_prompt_point(40);
    } else {
        irc_phase = IRC_GAP;
        irc_prompt_gap(40);
    }
    OLED_Refresh_Gram();
}

/* The carpark: both IRs at once. Whatever the car's position across, as long
   as it is square to the walls, left + right + 2 x IR_TO_CENTRE_MM is the
   gap between them. With the new fits when there are some, else calib.h's. */
static void irc_gap_tap(void)
{
    char u[200];
    float mean[2], sd[2];

    OLED_Clear();
    cal_line(0,  "CARPARK");
    cal_line(10, "reading 1 s...");
    OLED_Refresh_Gram();
    irc_measure(mean, sd);

    const uint8_t L = IR_SIDE_LEFT, R = IR_SIDE_RIGHT;
    const float nl = irc_now_cm(L, mean[L]), nr = irc_now_cm(R, mean[R]);
    const float c_now = (IRC_GAP_CM - nl - nr) * 5.0f;          /* cm -> mm, halved */
    const uint8_t have_fit = irc_fit[L].ok && irc_fit[R].ok;
    const float fl = have_fit ? irc_fit_cm(L, mean[L]) : nl;
    const float fr = have_fit ? irc_fit_cm(R, mean[R]) : nr;
    const float c_fit = (IRC_GAP_CM - fl - fr) * 5.0f;

    snprintf(u, sizeof u, "[IRC] carpark %.0f cm: L %.4f V R %.4f V; calib.h now L %.1f R %.1f cm"
                          " -> IR_TO_CENTRE_MM %.0f\r\n",
             (double)IRC_GAP_CM, (double)mean[L], (double)mean[R], (double)nl, (double)nr, (double)c_now);
    command_send(u);
    if (have_fit) {
        snprintf(u, sizeof u, "[IRC] carpark with the new fits: L %.1f R %.1f cm -> IR_TO_CENTRE_MM %.0f\r\n",
                 (double)fl, (double)fr, (double)c_fit);
        command_send(u);
    }
    snprintf(u, sizeof u, "[IRC] for calib.h (%s):\r\n#define %-20s%.0f\r\n",
             have_fit ? "with the new IR_*_M/_B/_K" : "with the IR values in calib.h now",
             "IR_TO_CENTRE_MM", (double)(have_fit ? c_fit : c_now));
    command_send(u);
    /* Both IRs see their wall from 10 to ~25 cm in a 60 cm carpark; a reading
       at either end of the range means one of them is not looking at one. */
    const uint8_t odd = (fl <= IR_MIN_CM + 0.5f || fr <= IR_MIN_CM + 0.5f ||
                         fl > 0.6f * IRC_GAP_CM || fr > 0.6f * IRC_GAP_CM ||
                         c_fit < 30.0f || c_fit > 200.0f);
    if (odd) command_send("[IRC] check: is the car between the walls, square to them?\r\n");

    OLED_Clear();
    cal_line(0,  "CARPARK %.0f cm", (double)IRC_GAP_CM);
    cal_line(10, "L %.1f  R %.1f", (double)fl, (double)fr);
    cal_line(20, "centre %.0f mm", (double)c_fit);
    if (have_fit) cal_line(30, "(calib.h: %.0f)", (double)c_now);
    if (odd) cal_line(40, "CHECK: square?");
    else     cal_line(40, "(%d in calib.h)", IR_TO_CENTRE_MM);
    cal_line(50, "tap = live view");
    OLED_Refresh_Gram();

    irc_phase = IRC_GAP_SHOWN;
}

void cal_ir_step(void)
{
    switch (irc_phase) {
        case IRC_SWEEP: irc_sweep_tap(); break;
        case IRC_GAP:   irc_gap_tap();   break;
        case IRC_GAP_SHOWN:                          /* on to the live view */
            irc_phase = IRC_LIVE;
            irc_last_draw = HAL_GetTick() - 1000u;
            OLED_Clear();
            break;
        default:        cal_ir_show_idle(); break;   /* live view: start over */
    }
}

/* Live view: what each side reads now (calib.h, as Task 2 sees it) and with
   its new fit, every 200 ms. Move a board about to check them. */
void cal_ir_poll(void)
{
    if (irc_phase != IRC_LIVE) return;
    const uint32_t now = HAL_GetTick();
    if (now - irc_last_draw < 200u) return;
    irc_last_draw = now;

    float v[2] = { 0.0f, 0.0f };
    for (uint8_t i = 0u; i < 8u; i++) {             /* 8 ms of ADC noise out */
        v[0] += ir_voltage(0u);
        v[1] += ir_voltage(1u);
        HAL_Delay(1);
    }
    v[0] *= 0.125f;
    v[1] *= 0.125f;

    const uint8_t L = IR_SIDE_LEFT, R = IR_SIDE_RIGHT;
    cal_line(0,  "IR LIVE tap=redo");
    cal_line(10, "L now %5.1f", (double)ir_filtered_cm(L));
    cal_line(20, "R now %5.1f", (double)ir_filtered_cm(R));
    if (irc_fit[L].ok) cal_line(30, "L fit %5.1f", (double)irc_fit_cm(L, v[L]));
    else               cal_line(30, " ");
    if (irc_fit[R].ok) cal_line(40, "R fit %5.1f", (double)irc_fit_cm(R, v[R]));
    else               cal_line(40, " ");
    cal_line(50, "%.3fV %.3fV", (double)v[L], (double)v[R]);
    OLED_Refresh_Gram();

    if (now - irc_last_log >= 1000u) {
        char u[128];
        irc_last_log = now;
        snprintf(u, sizeof u, "[IRC] live L %.4f V now %.1f fit %.1f | R %.4f V now %.1f fit %.1f\r\n",
                 (double)v[L], (double)irc_now_cm(L, v[L]), (double)(irc_fit[L].ok ? irc_fit_cm(L, v[L]) : 0.0f),
                 (double)v[R], (double)irc_now_cm(R, v[R]), (double)(irc_fit[R].ok ? irc_fit_cm(R, v[R]) : 0.0f));
        command_send(u);
    }
}
