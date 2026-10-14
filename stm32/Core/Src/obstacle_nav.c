#include "obstacle_nav.h"
#include "control.h"
#include "calib.h"
#include "oled.h"
#include "command.h"
#include "servo.h"
#include "usart.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define STANDOFF_DIST_MM    150
#define PILLAR_HALF_WIDTH   50


/* ------------------------------------------------------------------------- */
/* The Box Bypass Maneuver                                                */
/* ------------------------------------------------------------------------- */
static void bypass_unplanned_obstacle(void)
{
    command_send(">> [AVOID] Executing Box Bypass...\r\n");

    /* 1. Back up 15cm to give steering clearance */
    move_straight_mm(-150);
    HAL_Delay(200);

    /* 2. K-Turn Left to face away from obstacle */
    move_kturn_90(1);
    HAL_Delay(200);

    /* 3. Drive 25cm laterally to clear the 10x10 obstacle */
    move_straight_mm(250);
    HAL_Delay(200);

    /* 4. K-Turn Right to face original destination */
    move_kturn_90(0);
    HAL_Delay(200);

    /* 5. Drive 35cm forward (Drives completely past the obstacle) */
    move_straight_mm(350);
    HAL_Delay(200);

    /* 6. K-Turn Right to point back toward original line */
    move_kturn_90(0);
    HAL_Delay(200);

    /* 7. Drive 25cm laterally to return to original line */
    move_straight_mm(250);
    HAL_Delay(200);

    /* 8. K-Turn Left to face destination again */
    move_kturn_90(1);
    HAL_Delay(200);

    command_send(">> [AVOID] Bypass complete.\r\n");
}

/* ------------------------------------------------------------------------- */
/* Smart Movement Wrapper                                                 */
/* ------------------------------------------------------------------------- */
static void safe_move_straight_mm(int32_t distance_mm)
{
    while (distance_mm > 0)
    {
        uint8_t reached = move_straight_mm(distance_mm);

        if (reached) {
            distance_mm = 0;
        }
        else {
            /* Obstacle detected! */
            bypass_unplanned_obstacle();

            /* The bypass results in exactly 200mm of net forward progress */
            distance_mm -= 200;

            if (distance_mm < 0) {
                move_straight_mm(distance_mm); /* Fix overshoot if close to target */
                distance_mm = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Simulated Camera Scan                                                  */
/* ------------------------------------------------------------------------- */
static uint8_t inspect_current_face(uint8_t face_num)
{
    char buf[64];
    snprintf(buf, sizeof(buf), ">> [CAM] Scanning Face %u...\r\n", face_num);
    command_send(buf);

    OLED_Clear();
    OLED_ShowString(0, 0, (const uint8_t *)"SCANNING...");
    OLED_Refresh_Gram();

    /* 1. Reset the flag before we start waiting */
    image_found = 0;

    uint32_t start_time = HAL_GetTick();
    const uint32_t TIMEOUT_MS = 3000; /* Wait up to 3 seconds */

    /* 2. Non-blocking timeout loop */
    while ((HAL_GetTick() - start_time) < TIMEOUT_MS)
    {
        /* CRITICAL: We must poll the UART buffer while waiting.
         * This allows dispatch() to run and change image_found to 1! */
        command_poll();

        if (image_found > 0)
        {
            snprintf(buf, sizeof(buf), "Found ID: %u   ", image_found);
            OLED_ShowString(0, 20, (const uint8_t *)buf);
            OLED_Refresh_Gram();

            command_send(">> Target Matched!\r\n");

            HAL_Delay(1000); /* Pause so you can read OLED before driving away */
            return 1; /* Success */
        }

        HAL_Delay(5); /* Yield to prevent CPU lockup */
    }

    /* 3. Timeout reached */
    OLED_ShowString(0, 20, (const uint8_t *)"NO TARGET     ");
    OLED_Refresh_Gram();
    command_send(">> Scan Timeout. No image detected.\r\n");

    // Reset Image
    image_found = 0;

    return 0; /* Target not found, continue to next face */
}

/* ------------------------------------------------------------------------- */
/* Circum-Navigation (Back-Out and Box Strategy)                          */
/* ------------------------------------------------------------------------- */
//static void transition_to_next_face(void)
//{
//    /* 1. Reverse to gain a 350mm safe orbit clearance from the pillar center */
//    /* Math: 150mm (current standoff) + 50mm (half-width) + 150mm (reverse) = 350mm */
//    move_straight_mm(-150);
//    HAL_Delay(250);
//
//    /* 2. Rotate 90 Deg left to face parallel to the current face */
//    move_kturn_90(1);
//    HAL_Delay(250);
//
//    /* 3. Drive straight to the corner of the 350mm orbit box */
//    safe_move_straight_mm(350);
//    HAL_Delay(250);
//
//    /* 4. Rotate 90 Deg right to face parallel to the NEXT face */
//    move_kturn_90(0);
//    HAL_Delay(250);
//
//    /* 5. Drive straight to align with the center of the next face */
//    safe_move_straight_mm(350);
//    HAL_Delay(250);
//
//    /* 6. Rotate 90 Deg right to turn INWARD and point the camera at the face */
//    move_kturn_90(0);
//    HAL_Delay(250);
//
//    /* 7. Drive forward to re-establish the exact 15cm focal standoff */
//    safe_move_straight_mm(150);
//    HAL_Delay(250);
//}

static void transition_to_next_face(void)
{
    command_send(">> Transitioning: S-Curve & Reverse maneuver\r\n");

    move_straight_mm(-50);
	HAL_Delay(100);

    /* 1. S-Curve Right to change lanes (Clears the pillar's width) */
    move_turn_deg(0, 1, 45); /* Steer Right, Forward */
    HAL_Delay(100);
    move_turn_deg(1, 1, 45); /* Steer Left, Forward (Straightens out) */
    HAL_Delay(150);

    /* 2. Drive Forward (Overshoot the center of the next face) */
    /* You must overshoot by roughly the radius of your reverse turn (e.g., 250mm) */
    safe_move_straight_mm(400);
    HAL_Delay(100);

    /* 3. The 90-Degree Reverse Arc */
    /* Steer Right (0), Drive Reverse (0). Nose swings 90 degrees Left! */
    uint8_t safe = move_turn_deg(0, 0, 90);
    //move_pivot_deg(1,90);
    HAL_Delay(100);

    if (!safe) {
        command_send("[WARN] Reverse arc blocked!\r\n");
    }

    /* 4. Optional: Small straight adjustment to lock in the exact 15cm standoff */
    /* If the reverse arc left you at 20cm away, drive 5cm forward to correct */
    safe_move_straight_mm(90);
    HAL_Delay(100);
}

/* ------------------------------------------------------------------------- */
/* Main Navigation Entry Point                                            */
/* ------------------------------------------------------------------------- */
void navigate_and_inspect_obstacle(int32_t target_x_mm, int32_t target_y_mm)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "\r\n--- Navigating to Target ---\r\n");
    command_send(buf);

    /* Phase 1: Approach the first face */
    int32_t approach_y = target_y_mm - STANDOFF_DIST_MM - PILLAR_HALF_WIDTH;
    if (approach_y > 0) {
        safe_move_straight_mm(approach_y);
        HAL_Delay(250);
    }

    /* Phase 2: Inspect all 4 faces */
    for (uint8_t face = 1; face <= 4; face++)
    {
        uint8_t target_found = inspect_current_face(face);
        if (target_found) {
            command_send(">> Target matched! Mission complete.\r\n");
            break;
        }

        if (face < 4) {
            transition_to_next_face();
        }
    }
}

/* ========================================================================= */
/* Task 2 -- fastest car                                                      */
/* ========================================================================= */
/*
 * Kush (09-Oct): what changed from the version that ran on 09-Oct, and why.
 *
 * Sensing. Every phase that used to creep -- stop, read the side IR, move a
 * fixed 50/100/200 mm, repeat -- now drives straight through while the 100 Hz
 * tick watches the IR (move_straight_watch() in control.h) and stops a set
 * distance past the edge. A creep left the car anywhere from 0 to one whole
 * step past the edge. Now it goes at least *_AFTER_MM past (half the old
 * step, i.e. the old average), plus whatever it needs to slow from that
 * phase's T2_SENSE_* speed, and stops at the same place every run. Simulated
 * against the old step ranges: front face ~120 mm past the edge (old 0-200),
 * end ~95 (0-100), back face ~120 (0-200), obstacle 1 heading home ~65
 * (0-50, which only moves the car a little further from obstacle 1 before
 * it turns behind it; ~90 since speed step 4, see T2_SENSE_OBS1), obstacle 1
 * from behind lined up at ~27 (0-50, average 25) by the back-off below. The
 * IR is a 5-tick median
 * (sensors.c), not one raw sample, and the ultrasonic is a median of
 * background pings, not one ping.
 *
 * Distances. Dead reckoning adds what the encoders say each move covered,
 * not the distance asked for.
 *
 * Speed. task_2() runs on MOTION_PROFILE_TASK2 (calib.h / control.h): the
 * straights it was tested with before 0c37bfe (cruise 60, crawl 15, the
 * original brake point, the same as Task 1). Back to the Task 1 profile on
 * return. 0c37bfe's 80 / 50 and the faster straights tried on 09-Oct both
 * left the car much closer to obstacle 2 after the obstacle 1 dodge.
 * Speed step 5 (11-Oct): cruise 80, but with the crawl kept at 15 and the
 * brake point worked out from the measured wheel speed (calib.h), so each
 * straight stops from the same crawl as before. A dodge that still ends
 * closer to obstacle 2 is now backed off to the standoff (see below).
 *
 * Turns. Exactly as in the version that ran on 09-Oct: every turn at
 * SPEED_TURN (since speed step 6, 11-Oct, faster in its middle when
 * TURN_FAST in calib.h is 1; the end of every turn is unchanged), the same
 * servo_us(SERVO_CENTRE); HAL_Delay(150) pauses,
 * CREEP 3's forward + reverse pivot of the tilt before a plain 90 for TURN 4
 * (removed 11-Oct: it never turned the car, see CREEP 3; TURN 4 is now
 * 90 + the tilt instead),
 * and the parking turn at 90 + tilt (more when the car drifted away from
 * obstacle 1). A faster 90 degree turn, no pauses, the tilt folded into
 * TURN 4 and the opposite parking-turn sign were tried on 09-Oct and
 * reverted the same day. Speed step 1 (11-Oct) dropped only the plain
 * HAL_Delay(150) in task_2_turn() / task_2_straighten() and the 100 ms after
 * the first dodge turn: the car is standing still then and nothing moves
 * during them (the steering pauses stay). Speed step 2 (11-Oct) dropped the
 * four steering pauses that came right before a straight (PHASE 3's 100 mm,
 * the consecutive reverse, CREEP 1, CREEP 2), since every straight centres
 * the steering and waits 150 ms itself; the ones before turns stay. Only the
 * tilt's measurement is new:
 *  - Both tilts came from two single raw IR readings. Now a least-squares
 *    line through every sample taken along the face, ends trimmed, cut to
 *    whole degrees toward zero as before.
 *
 * Without the Pi. task_2_with_arrows() runs exactly this with both arrows
 * given, for calibration runs started from SW1 (CAL_TASK2 in main.c).
 *
 * Obstacle 2 standoff (11-Oct). If the dodge leaves the car closer than
 * 30 cm to obstacle 2, however close, it now backs off to 30 cm before
 * reading the arrow, and checks the distance again. The original only moved
 * on a reading over 5 cm, so at the 60 cm minimum gap (nose a few cm from
 * obstacle 2) it stayed put and went round obstacle 2 from there, into it.
 * The way back now drives blind for the distance measured to obstacle 2
 * right after the dodge before it looks for obstacle 1 (FIND OBSTACLE 1):
 * the old figure added the back-off in, and then missed obstacle 1.
 *
 * Fixed on the way:
 *  - SCAN went out twice per stop (both sides of the 09-Oct merge added one).
 *    The Pi answered both: the second capture ran while the car was already
 *    dodging, held the next SCAN up by as much as 2.5 s, and its late IM
 *    could land inside obstacle 2's scan window.
 *  - dodge_left / dodge_left_2 were uninitialised when no arrow was read.
 *    They fall back to left, the default before 09-Oct.
 *  - A failed first scan moved the car 10 cm closer and left it there for
 *    the dodge / wrap, which were set up for the normal standoff. It backs
 *    out the 10 cm again now (obstacle 2 used to, but only after every retry
 *    had failed).
 *  - Parking ended with motion_stop(), which latches the abort, so START2
 *    never sent its DONE and the tablet's Task 2 clock ran on to 3:00.
 */

/* --- Approaches ---------------------------------------------------------- */
#define T2_STANDOFF_OBS1_MM     300     /* front ultrasonic to obstacle 1 to read it */
#define T2_STANDOFF_OBS2_MM     300     /* and to obstacle 2                         */
/* No echo at all: obstacle 1 is at least 600 mm out, so 300 still leaves 300. */
#define T2_NO_ECHO_OBS1_MM      300
/* Kush (11-Oct): obstacle 2 closer than its standoff after the dodge (at the
   60 cm minimum gap the dodge leaves the nose a few cm from it): back off to
   the standoff, see the OBSTACLE 2 STANDOFF block in task_2_run().
   No echo right after the dodge = too close for the HC-SR04 to time (it
   needs ~2 cm): back off this far without one, then measure again. */
#define T2_OBS2_BLIND_BACK_MM   150
/* After backing off, measure again and correct once more if the car is
   further than this from the standoff (a reading a few cm from the face can
   be off by a few cm). */
#define T2_OBS2_RECHECK_MM      25
/* The approach to obstacle 2 only acts on a reading under this, as the original
   did: anything further is taken as a miss and the car stays where the dodge
   left it. Raised to 250 on 09-Oct and put back the same day: a far echo after
   the hook back then drove the car forward into obstacle 2. */
#define T2_US_MAX_TRUST_CM      150.0f

/* --- Sensing phases ------------------------------------------------------ */
/* Commanded speeds while a phase is armed (units of SPEED_STRAIGHT: the wheels
   do ~45 % of it). Lower means stopping closer to *_AFTER_MM past the edge.
   From the sense speed the car needs about 50 -> 95 mm, 45 -> 80 mm,
   35 -> 55 mm, 30 -> 45 mm of room to slow to its crawl (simulated), plus
   ~10-20 mm for the IR to confirm the edge. See the header above for where
   each phase ends up against the old creep.
   Kush (11-Oct, speed step 4): OBS1 30 -> 42 and PARK 35 -> 49 (x1.4).
   Simulated over 36 layouts: ~1.1 s a run, same clearances, same obstacle 1
   fit (n ~19, tilt within 1 deg), parked within a few mm of before. The
   obstacle 1 search now ends ~90 mm past the edge instead of ~65 (the turn
   behind obstacle 1 is that much further on, accum_forward counts it), and
   the search from behind ~65 mm past instead of ~40, which extra_back
   reverses as before (now ~55 mm instead of ~25).
   FACE and SIDE stay: a watch cannot run faster than the cruise (60 then,
   80 since speed step 5; see MODE_STRAIGHT in control.c), and FACE at 60 saved ~0.06 s simulated while
   ending the back face ~35 mm further out (from 60 the car needs ~135 mm to
   slow), which swings TURN 4 that much closer to the side wall. SIDE made
   no difference: with the end only 10 cm deep, CREEP 2 never moved. */
#define T2_SENSE_FACE           50.0f   /* along obstacle 2's front and back faces */
#define T2_SENSE_SIDE           45.0f   /* along its 10 cm end                     */
#define T2_SENSE_OBS1           42.0f   /* finding obstacle 1 on the way back (was 30) */
#define T2_SENSE_PARK           49.0f   /* last ultrasonic approach in the carpark (was 35) */

/* CREEP 1: along obstacle 2's front face until the inner IR loses it. */
#define T2_FRONT_CLEAR_CM       35.0f
#define T2_FRONT_AFTER_MM       100     /* old: 200 mm steps */
#define T2_FRONT_MAX_MM         800
/* CREEP 2: along its 10 cm end. */
#define T2_SIDE_CLEAR_CM        45.0f
#define T2_SIDE_AFTER_MM        50      /* old: 100 mm steps */
#define T2_SIDE_MAX_MM          300
/* CREEP 3: along its back face. It must see the face, then lose it. */
#define T2_BACK_CM              50.0f
#define T2_BACK_AFTER_MM        100     /* old: 200 mm steps */
#define T2_BACK_MAX_MM          1200
/* Return: obstacle 1 seen then lost, heading home and then across behind it. */
#define T2_OBS1_CM              55.0f
#define T2_OBS1_AFTER_MM        25      /* old: 50 mm steps */
#define T2_OBS1_MAX_MM          500
/* Kush (11-Oct): heading home the car passes obstacle 1 at the side, its IR
   ~45 cm from it with a 60 cm wide obstacle 2, further with a wider one or
   if the car has turned a few degrees away; at 55 cm it could miss it. So
   that search counts anything within T2_OBS1_FAR_CM as obstacle 1, but
   never anything within T2_OBS1_MARGIN_CM of what the IR reads just before
   the search starts (open space, or a wall on the far side of the course),
   and never less than T2_OBS1_CM. With the rules' layout the next thing that
   side IR could see is the wall beyond the far end of obstacle 2, about 1 m
   or more away (its 50 cm clearance + obstacle 2 + the car's own offset).
   Kush (14-Oct): with the IRs calibrated by CAL_IR and their lenses 5.5 cm
   from the centre line (4 cm further in than the simulator had them),
   obstacle 1 reads ~4 cm further away than it did there: up to ~73 cm with
   a 1 m wide obstacle 2, past the old 70. So T2_OBS1_FAR_CM 70 -> 75. To
   reach it, an open-space reading (80, the cap) only loses
   T2_OBS1_OPEN_MARGIN_CM; anything nearer (a wall) still loses 10. Past
   70 cm the curve is flat and noise is worth a few cm, so the search also
   ignores anything seen over less than T2_OBS1_MIN_SEEN_MM of driving
   before it is lost: obstacle 1's side is 10 cm long, a noise flicker a few
   mm. An obstacle 2 wider than ~1.05 m still puts obstacle 1 out of reach. */
#define T2_OBS1_FAR_CM          75.0f
#define T2_OBS1_MARGIN_CM       10.0f
#define T2_OBS1_OPEN_MARGIN_CM  5.0f
#define T2_OBS1_MIN_SEEN_MM     30
/* Kush (12-Oct): the S-shift on the way home, before the search for
   obstacle 1 (two 30 deg turns inward, undone by a reverse after the turn
   behind obstacle 1). 1 = every run, as before. 0 = skipped when, just
   before it, the inner IR reads open space to its full range (the search
   can then see the ~85 mm further it needs without the shift) and the
   outer IR has the side wall at least T2_RETURN_WALL_CM away (the car then
   stays that much further out until it turns behind obstacle 1). 40 cm,
   not less: under 10 cm the Sharp's curve folds back, and a wall a few cm
   away can read as 20-30 cm. See RETURNING TO CARPARK. */
#ifndef T2_RETURN_S_SHIFT
#define T2_RETURN_S_SHIFT       0
#endif
#define T2_RETURN_WALL_CM       40.0f
/* Kush (14-Oct): park straight off the pass by obstacle 1 on the way home.
   1 = the inner IR's distance to obstacle 1's side face as the car drives
   past it (the median of what the watch logged along it) gives how far out
   the car is. It turns in behind obstacle 1 as before, drives whatever that
   leaves over the two turns (R_in + R_out, ~63 cm, so often nothing) and
   turns out straight into the carpark. 0 = the search along obstacle 1's
   back face, its tilt fit and the reverse after it, as before. Simulated
   over 200 layouts: ~4.5 s quicker, same clearances, parked closer to the
   centre line (3 cm off on average against 4.5). It needs the side IRs
   right at 40-70 cm and IR_TO_CENTRE_MM (calib.h); CAL_IR measures both.
   If the pass did not see obstacle 1 it searches as before. */
#ifndef T2_PARK_DIRECT
#define T2_PARK_DIRECT          0
uint8_t  t2_direct_used = 0u;
float    t2_direct_side_cm = 0.0f;
uint16_t t2_direct_n = 0u;
#endif
#define T2_DIRECT_MIN_N         6u      /* IR samples along obstacle 1's side, at least */
#define T2_DIRECT_MIN_MM        15      /* a middle straight shorter than this is skipped */
/* Behind obstacle 1, where the car stops sets where it lines up with the
   carpark. The old creep stopped 0-50 mm past the edge (25 on average),
   judged at a standstill. The watch confirms the edge a little late (sensor
   update + median + confirm: about T2_IR_LAG_TICKS ticks of travel) and then
   needs room to slow down, so whatever it drives past "edge + 25 mm" is added
   to the back-off before the parking turn. */
#define T2_OBS1_STOP_PAST_EDGE_MM   25
#define T2_IR_LAG_TICKS         6.0f
#define T2_OBS1_EXTRA_MAX_MM    150

/* --- Tilt from the IR line fits ------------------------------------------ */
#define T2_FIT_TRIM             0.2f    /* drop the first and last 20 % (face edges) */
#define T2_FIT_MIN_N            8u
#define T2_FIT_MIN_SPAN_BACK_MM 150.0f  /* obstacle 2's back face, >= 30 cm long    */
#define T2_FIT_MIN_SPAN_OBS1_MM 60.0f   /* obstacle 1, 10 cm wide                    */
#define T2_TILT_MAX_DEG         5       /* the old clamp                             */

/* --- Parking -------------------------------------------------------------- */
/* Stop with the front ultrasonic this far from the back wall (the old 30 mm
   creep stopped anywhere from 7 to 10 cm). One move, never more than
   T2_PARK_MAX_MM, T2_PARK_NO_ECHO_MM if there is no echo at all, and the
   ultrasonic stops it early below T2_PARK_GUARD_CM. */
#define T2_PARK_STOP_CM         9.0f
#define T2_PARK_GUARD_CM        20.0f
#define T2_PARK_MAX_MM          400
#define T2_PARK_NO_ECHO_MM      150

/* The arrow the last task_2_image_rec() call got (38 / 39), so a late IM
   cannot change it under us between the scan and the decision. */
static uint8_t t2_arrow_id = 0u;

/* Kush (09-Oct): arrows given by task_2_with_arrows() for obstacles 1 and 2
   (T2_ARROW_LEFT / T2_ARROW_RIGHT), used instead of scanning. 0 = scan, which
   is what START2 always does. */
static uint8_t t2_given_arrow[2] = { 0u, 0u };

/* One line per sensing phase on USART3: where the condition was confirmed,
   how far the car went in total, and its speed then. For tuning *_AFTER_MM. */
static void t2_log_watch(const char *name, const watch_t *w)
{
    char u[112];
    snprintf(u, sizeof u, "[T2] %s trig=%u seen=%u at=%ldmm drove=%ldmm v=%.2fmm/tick%s\r\n",
             name, (unsigned)w->triggered, (unsigned)w->seen, (long)w->trigger_mm,
             (long)w->driven_mm, (double)w->trigger_speed_mm_tick,
             w->blips ? " (ignored a flicker)" : "");
    command_send(u);
}

/* How far the search for obstacle 1 may look, from what the inner IR reads
   standing still beforehand: T2_OBS1_OPEN_MARGIN_CM short of an open-space
   reading (the cap), T2_OBS1_MARGIN_CM short of anything else, never more
   than T2_OBS1_FAR_CM or less than T2_OBS1_CM. */
static float t2_obs1_range_cm(float ir_open)
{
    const float margin = (ir_open >= IR_MAX_CM - 0.5f) ? T2_OBS1_OPEN_MARGIN_CM : T2_OBS1_MARGIN_CM;
    float r = ir_open - margin;
    if (r > T2_OBS1_FAR_CM) r = T2_OBS1_FAR_CM;
    if (r < T2_OBS1_CM)     r = T2_OBS1_CM;
    return r;
}

static void t2_status(const char *s)
{
    OLED_ShowString(0, 40, (const uint8_t *)s);
    OLED_Refresh_Gram();
}

/* Front distance for an approach: three quiet pings (sensors.h), each taken
   the way the original took its one reading, and their median (the nearer of
   two if one got no echo, the one if only one did). -1 if none came back.
   About 0.2-0.35 s (0.35-0.5 s before the quiet gap went from 100 to 60 ms
   on 11-Oct, sensors.c). The background pings every 40 ms that this used to read
   could miss obstacle 1's weak echo and report what was behind it, and the
   car then drove too far. */
static float t2_front_cm(void)
{
    float r[3];
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < 3u; i++) {
        if (motion_abort_requested()) return -1.0f;
        const float d = ultrasonic_quiet_ping_cm();
        if (d > 0.0f) r[n++] = d;
    }
    if (n == 0u) return -1.0f;
    if (n == 1u) return r[0];
    if (n == 2u) return (r[0] < r[1]) ? r[0] : r[1];
    float a = r[0], b = r[1], c = r[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) b = c;
    return (a > b) ? a : b;
}

/* Tilt from the face the last watch logged: whole degrees, clamped to
   +-T2_TILT_MAX_DEG, + = the face got further away along the run. 0 if the
   fit did not have enough data. Whole degrees cut toward zero, as the
   original creep code did, so a tilt under 1 degree is no correction. */
static int t2_face_tilt_deg(float min_span_mm, const char *name)
{
    float slope = 0.0f, span = 0.0f;
    uint16_t n = 0u;
    int tilt = 0;
    uint8_t ok = watch_face_fit(T2_FIT_TRIM, T2_FIT_MIN_N, min_span_mm, &slope, &n, &span);
    if (ok) {
        /* slope is cm of IR per mm driven: x10 for mm per mm */
        float deg = atanf(slope * 10.0f) * (180.0f / 3.14159f);
        tilt = (int)deg;
        if (tilt >  T2_TILT_MAX_DEG) tilt =  T2_TILT_MAX_DEG;
        if (tilt < -T2_TILT_MAX_DEG) tilt = -T2_TILT_MAX_DEG;
    }
    char u[96];
    snprintf(u, sizeof u, "[T2] %s fit=%u n=%u span=%.0fmm slope=%.4f tilt=%d\r\n",
             name, (unsigned)ok, (unsigned)n, (double)span, (double)slope, tilt);
    command_send(u);
    return tilt;
}

/**
 * @brief Task 2 Image rec helper
 */
int task_2_image_rec(void){
	char buf[64];
	/* Kush (09-Oct): after a STOP the rest of the run still comes through
	 * here, up to 4 times per arrow, and each call used to send SCAN before
	 * noticing the STOP. The Pi answers every SCAN, so up to 8 IMs could
	 * still be on their way when the next run started, and a late one landing
	 * in its first scan window would be taken as that run's arrow. */
	if (motion_abort_requested()) return 0;
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"SCANNING...");
	OLED_Refresh_Gram();
	/* Reset the flag before we start waiting */
	image_found = 0;
	t2_arrow_id = 0u;

	/* Denzel: tell the Pi the window is open. Without this nothing on the
	 * Pi knows the car has stopped in front of an arrow, so no IM038/IM039
	 * ever arrives and the dodge always falls back to the guess -- which the
	 * briefing counts as an invalid run whenever the guess is wrong. The Pi
	 * answers IM038 (right) or IM039 (left), picked up by the command_poll()
	 * in the loop below.
	 * Kush (09-Oct): once. The merge had left a second SCAN here, and the Pi
	 * answers every SCAN it gets. */
	command_send("SCAN\r\n");

	uint32_t start_time = HAL_GetTick();
	const uint32_t TIMEOUT_MS = 3000; /* Wait up to 3 seconds */
	while ((HAL_GetTick() - start_time) < TIMEOUT_MS)
	{
		/* CRITICAL: We must poll the UART buffer while waiting.
		 * This allows dispatch() to run and change image_found to 1! */
		command_poll();
		if (motion_abort_requested()) return 0;   /* STOP: don't sit out the 3 s */

		uint8_t id = image_found;
		if (id == 38 || id == 39)
		{
			t2_arrow_id = id;
			snprintf(buf, sizeof(buf), "Found ID: %u   ", id);
			OLED_ShowString(0, 20, (const uint8_t *)buf);
			OLED_Refresh_Gram();
			return 1;
		}

		HAL_Delay(5); /* Yield to prevent CPU lockup */
	}
	return 0;
}

/**
 * @brief Task 2 turn helper (Upgraded with dynamic global orientation math)
 *
 * @param left           1 for a left turn, 0 for a right turn.
 * @param forward        1 for driving forward, 0 for driving backward.
 * @param degrees        Turn degree (how much we are turning)
 * @param current_angle  The car's current heading relative to 0 (magnitude in degrees)
 * @param accum_forward  Pointer to the forward/backward displacement tracker.
 * @param accum_left     Pointer to the left/right displacement tracker.
 * @return int           1 on success.
 */
int task_2_turn(int left, int forward, int degrees, int current_angle, int32_t* accum_forward, int32_t* accum_left) {
    /* Convert angles to radians for true arc projection */
    float rad_current = (float)current_angle * (3.14159f / 180.0f);
    float rad_next    = (float)(current_angle + degrees) * (3.14159f / 180.0f);

    int32_t R = 0;
    if (forward) {
        R = left ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
    } else {
        R = left ? TURN_RADIUS_BL_MM : TURN_RADIUS_BR_MM;
    }

    /*
     * True Kinematics of a turning car from a non-zero starting angle:
     * We project the arc's geometry onto the global grid using the difference
     * between the starting heading and the ending heading.
     */
    int32_t d_forward = (int32_t)(R * (sinf(rad_next) - sinf(rad_current)));
    int32_t d_left    = (int32_t)(R * (cosf(rad_current) - cosf(rad_next)));

    /* Apply directional signs */
    if (!forward) d_forward = -d_forward;
    if (!left)    d_left    = -d_left;

    *accum_forward += d_forward;
    *accum_left    += d_left;

    /* Execute the physical turn. Kush (11-Oct): no HAL_Delay(150) first any
       more (speed step 1). The car is already standing still here (every move
       ends with 100 ms of settle) and move_turn_deg() sets the steering and
       waits SERVO_SETTLE_MS itself, so the wait changed nothing but the time. */
    move_turn_deg(left, forward, degrees);
    return 1;
}

/**
 * @brief Mathematically corrects for straightening out of an S-curve.
 *
 * @param original_dodge_left The direction of the FIRST turn in the S-curve.
 * @param degrees The angle being straightened.
 * @param accum_forward Pointer to forward tracker.
 * @param accum_left Pointer to left tracker.
 */
void task_2_straighten(int original_dodge_left, int degrees, int32_t* accum_forward, int32_t* accum_left) {
    /* 1. We physically turn opposite to the original dodge to straighten out.
       Kush (11-Oct): no HAL_Delay(150) first any more, as in task_2_turn(). */
    int turn_left = !original_dodge_left;
    move_turn_deg(turn_left, 1, degrees);

    /* 2. Calculate the displacement footprint */
    float rad = (float)degrees * (3.14159f / 180.0f);
    int32_t R = turn_left ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

    int32_t d_forward = (int32_t)(R * sinf(rad));
    int32_t d_left    = (int32_t)(R * (1.0f - cosf(rad)));

    *accum_forward += d_forward;

    /* 3. THE CRITICAL FIX:
     * Even though we are steering inward, the car continues to sweep
     * outward in global space until the heading hits 0!
     * We MUST keep adding to the original dodge direction. */
    if (original_dodge_left) {
        *accum_left += d_left;
    } else {
        *accum_left -= d_left;
    }
}

/* The arrow in front of obstacle 1 or 2, with the same fallback ladder as
 * before:
 *   scan; 10 cm closer and scan; tilt left (reverse-right 15) and scan;
 *   square up; tilt right (reverse-left 15) and scan; square up.
 * Then backs out the 10 cm, so what follows starts from the standoff it was
 * set up for. Returns 39 (left), 38 (right) or 0 (no arrow read).
 * Kush (09-Oct): if task_2_with_arrows() gave this obstacle's arrow, that is
 * the answer at once: no SCAN, no wait, no retries. */
static uint8_t t2_read_arrow(uint8_t obstacle, int32_t *af, int32_t *al)
{
    const uint8_t given = t2_given_arrow[(obstacle == 2u) ? 1 : 0];
    if (given == T2_ARROW_LEFT || given == T2_ARROW_RIGHT) {
        char u[48];
        t2_arrow_id = given;
        snprintf(u, sizeof u, "[T2] obs%u arrow %s (given, no scan)\r\n",
                 (unsigned)obstacle, (given == T2_ARROW_LEFT) ? "LEFT" : "RIGHT");
        command_send(u);
        return given;
    }

    if (task_2_image_rec()) return t2_arrow_id;

    /* RETRY 1: 10 cm closer */
    int32_t closer = move_straight_measured(100);
    *af += closer;
    uint8_t id = task_2_image_rec() ? t2_arrow_id : 0u;

    if (id == 0u) {
        /* RETRY 2: tilt left. Steer Right (0) in Reverse (0) sweeps the
           tail right and the nose left. */
        task_2_turn(0, 0, 15, 0, af, al);
        if (task_2_image_rec()) id = t2_arrow_id;
        task_2_straighten(1, 15, af, al);    /* square again */
    }
    if (id == 0u) {
        /* RETRY 3: tilt right. Steer Left (1) in Reverse (0). */
        task_2_turn(1, 0, 15, 0, af, al);
        if (task_2_image_rec()) id = t2_arrow_id;
        task_2_straighten(0, 15, af, al);    /* square again */
    }

    *af += move_straight_measured(-closer);
    return id;
}

/**
 * @brief Function for Task 2 Fastest car task
 */
static void task_2_run(void)
{
	int32_t accum_forward = 0;
	int32_t accum_left = 0;
	char u[96];
	watch_t w;

	command_send("[T2] start\r\n");

	/* ======================================================================
	 * OBSTACLE 1 (10x10cm) - The "Reverse Corridor" Dodge
	 * ====================================================================== */
	/* Stop 30cm (300mm) away to read the image. */
	float us1 = t2_front_cm();
	int32_t dist_forward = (us1 > 0.0f) ? (int32_t)(us1 * 10.0f) - T2_STANDOFF_OBS1_MM
	                                    : T2_NO_ECHO_OBS1_MM;
	snprintf(u, sizeof u, "[T2] obs1 us=%.1fcm drive=%ldmm\r\n", (double)us1, (long)dist_forward);
	command_send(u);
	if (dist_forward > 0) {
		accum_forward += move_straight_measured(dist_forward);
	}

	uint8_t id1 = t2_read_arrow(1u, &accum_forward, &accum_left);
	int dodge_left = (id1 == 38) ? 0 : 1;    /* 39 = left; nothing read = left */

	OLED_Clear();
	OLED_ShowString(0, 20, dodge_left ? (const uint8_t *)"Dodging Left" : (const uint8_t *)"Dodging Right");
	OLED_Refresh_Gram();

	/* ---------------------------------------------------------
	 * PHASE 1 + 2: Outward S-curve into the side lane
	 * --------------------------------------------------------- */
	task_2_turn(dodge_left, 1, 45, 0, &accum_forward, &accum_left);
	/* Kush (11-Oct): the HAL_Delay(100) that was here went with speed step 1
	   (see task_2_turn): the car is standing still and the next turn waits
	   for its steering itself. */
	task_2_straighten(dodge_left, 45, &accum_forward, &accum_left);

	/* ---------------------------------------------------------
	 * PHASE 3: Forward Clear
	 * --------------------------------------------------------- */
	t2_status("Clearing!       ");
	/* MECHANICS FIX 1 (let the servo return to centre first, so the move
	 * does not start on an arc) is still met: the turn that just ended
	 * centred the steering when it stopped and waited 100 ms, and
	 * move_straight_mm() centres it again and waits 150 ms before driving.
	 * Kush (11-Oct, speed step 2): the servo_us(SERVO_CENTRE);
	 * HAL_Delay(150) that was here only added 150 ms to that. */
	accum_forward += move_straight_measured(100);

	/* ---------------------------------------------------------
	 * PHASE 4: Tightly Hook back to the Center Line
	 * --------------------------------------------------------- */
	t2_status("Centering!      ");
	int turn_inward = !dodge_left;
	task_2_turn(turn_inward, 1, 45, 0, &accum_forward, &accum_left);
	servo_us(SERVO_CENTRE);
	HAL_Delay(150);
	task_2_straighten(turn_inward, 45, &accum_forward, &accum_left);

	/* ---------------------------------------------------------
	 * OBSTACLE 2 STANDOFF: sit 30 cm (T2_STANDOFF_OBS2_MM) from it to read
	 * the arrow.
	 * Kush (11-Oct): the dodge can leave the car closer than that (a few cm
	 * at the 60 cm minimum gap), and the original only moved on a reading
	 * over 5 cm, so there it stayed put and turned into obstacle 2. Now:
	 *  - closer than 30 cm, however close: back off to 30 cm;
	 *  - no echo at all: too close for the sensor to time, so back off
	 *    T2_OBS2_BLIND_BACK_MM and measure again;
	 *  - after backing off, measure once more and correct once if the car is
	 *    still more than T2_OBS2_RECHECK_MM out (a reading that close to the
	 *    face can be a few cm off);
	 *  - further than 30 cm (and under T2_US_MAX_TRUST_CM): forward to 30 cm,
	 *    as before.
	 * d_after_dodge, the distance to obstacle 2 right after the dodge, also
	 * sets how far the way back drives before it looks for obstacle 1 (FIND
	 * OBSTACLE 1).
	 * --------------------------------------------------------- */
	int32_t backed        = 0;       /* mm backed off here (net, measured)       */
	int32_t d_after_dodge = 0;       /* mm to obstacle 2 right after the dodge   */
	const float us2_first = t2_front_cm();
	float us2 = us2_first;
	if (us2 > 0.0f) {
		d_after_dodge = (int32_t)(us2 * 10.0f);
	} else {
		const int32_t d = move_straight_measured(-T2_OBS2_BLIND_BACK_MM);
		accum_forward += d;
		backed        -= d;
		us2 = t2_front_cm();
		if (us2 > 0.0f) {
			d_after_dodge = (int32_t)(us2 * 10.0f) - backed;
			if (d_after_dodge < 0) d_after_dodge = 0;
		}
	}
	int32_t dist_to_target = (us2 > 0.0f) ? (int32_t)(us2 * 10.0f) - T2_STANDOFF_OBS2_MM : 0;
	float us2_check = -1.0f;
	if (us2 > 0.0f && dist_to_target < 0) {
		/* Closer than the standoff: back off to it, then check. */
		int32_t d = move_straight_measured(dist_to_target);
		accum_forward += d;
		backed        -= d;
		us2_check = t2_front_cm();
		if (us2_check > 0.0f && us2_check < T2_US_MAX_TRUST_CM) {
			const int32_t err = (int32_t)(us2_check * 10.0f) - T2_STANDOFF_OBS2_MM;
			if (err < -T2_OBS2_RECHECK_MM || err > T2_OBS2_RECHECK_MM) {
				d = move_straight_measured(err);
				accum_forward += d;
				backed        -= d;
			}
		}
	} else if (dist_to_target > 0 && us2 < T2_US_MAX_TRUST_CM) {
		/* Further than the standoff: forward to it, as before. */
		accum_forward += move_straight_measured(dist_to_target);
	}
	snprintf(u, sizeof u, "[T2] obs2 us=%.1fcm then=%.1fcm check=%.1fcm backed=%ldmm\r\n",
	         (double)us2_first, (double)us2, (double)us2_check, (long)backed);
	command_send(u);

	/* ======================================================================
	 * OBSTACLE 2: The Perfect Orthogonal Box-Wrap
	 * ====================================================================== */
	uint8_t id2 = t2_read_arrow(2u, &accum_forward, &accum_left);
	int dodge_left_2 = (id2 == 38) ? 0 : 1;

	int turn_out = dodge_left_2 ? 1 : 0;
	int turn_in  = !dodge_left_2;

	int32_t R_out = turn_out ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
	int32_t R_in  = turn_in  ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

	/* The IR on the side facing the obstacle */
	uint8_t inner = dodge_left_2 ? IR_SIDE_RIGHT : IR_SIDE_LEFT;

	OLED_Clear();
	OLED_ShowString(0, 20, dodge_left_2 ? (const uint8_t *)"Obs 2: Left" : (const uint8_t *)"Obs 2: Right");
	OLED_Refresh_Gram();

	/* ---------------------------------------------------------
	 * TURN 1: 90 Degrees Outward (North -> West/East)
	 * --------------------------------------------------------- */
	t2_status("Turn 1!         ");
	/* Kush (12-Oct, T2_FEWER_PAUSES in calib.h): no pause here. The arrow
	   read always ends on a straight (or standing still after the scans), so
	   the wheels are centred already; the turn waits for its lock itself. */
	servo_us(SERVO_CENTRE);
	if (!T2_FEWER_PAUSES) HAL_Delay(150);
	move_turn_deg(turn_out, 1, 90);
	accum_forward += R_out;
	accum_left    += dodge_left_2 ? R_out : -R_out;

	// Reverse slightly if consecutive
	/* Kush (11-Oct, speed step 2): no servo_us(SERVO_CENTRE); HAL_Delay(150)
	   before this move, CREEP 1 or CREEP 2 any more. The turn before each
	   centred the steering when it stopped and waited 100 ms, and every
	   straight move centres it again (with the reverse trim when backing)
	   and waits 150 ms before it drives, so the servo still gets 250 ms to
	   come back from full lock (a turn gives it 250 ms to get there). The
	   IR checks before CREEP 1 and 2 now come 100 ms after the turn stopped
	   instead of 250 ms; the IR median covers only the last 50 ms. */
	if (dodge_left == dodge_left_2){
		int32_t d = move_straight_measured(-100);
		accum_left += dodge_left_2 ? d : -d;
	}

	/* ---------------------------------------------------------
	 * CREEP 1: The Front Face -- now one move, until the inner IR loses it
	 * --------------------------------------------------------- */
	t2_status("Front Face!     ");
	if (ir_filtered_cm(inner) < T2_FRONT_CLEAR_CM) {   /* already clear: no move, as before */
		w = (watch_t){ .kind = WATCH_IR_ABOVE, .ir_side = inner, .thresh_cm = T2_FRONT_CLEAR_CM,
		               .after_mm = T2_FRONT_AFTER_MM, .sense_speed = T2_SENSE_FACE };
		move_straight_watch(T2_FRONT_MAX_MM, &w);
		/* Facing West/East: pure lateral movement */
		accum_left += dodge_left_2 ? w.driven_mm : -w.driven_mm;
		t2_log_watch("front", &w);
	}

	/* ---------------------------------------------------------
	 * TURN 2: 90 Degrees Inward (West/East -> North)
	 * --------------------------------------------------------- */
	t2_status("Turn 2!         ");
	servo_us(SERVO_CENTRE);
	HAL_Delay(150);
	move_turn_deg(turn_in, 1, 90);
	accum_forward += R_in;
	accum_left    += dodge_left_2 ? R_in : -R_in;

	/* ---------------------------------------------------------
	 * CREEP 2: The Side Face
	 * --------------------------------------------------------- */
	t2_status("Side Face!      ");
	if (ir_filtered_cm(inner) < T2_SIDE_CLEAR_CM) {   /* speed step 2: see the reverse above */
		w = (watch_t){ .kind = WATCH_IR_ABOVE, .ir_side = inner, .thresh_cm = T2_SIDE_CLEAR_CM,
		               .after_mm = T2_SIDE_AFTER_MM, .sense_speed = T2_SENSE_SIDE };
		move_straight_watch(T2_SIDE_MAX_MM, &w);
		accum_forward += w.driven_mm;
		t2_log_watch("side", &w);
	}

	/* ---------------------------------------------------------
	 * TURN 3: 90 Degrees Inward (North -> East/West)
	 * --------------------------------------------------------- */
	t2_status("Turn 3!         ");
	/* Kush (12-Oct, T2_FEWER_PAUSES): no pause here either: TURN 2 used the
	   same lock (or CREEP 2 left the wheels centred). */
	servo_us(SERVO_CENTRE);
	if (!T2_FEWER_PAUSES) HAL_Delay(150);
	move_turn_deg(turn_in, 1, 90);
	accum_forward += R_in;
	accum_left    += dodge_left_2 ? -R_in : R_in;

	/* ---------------------------------------------------------
	 * CREEP 3: The Back Face & Dynamic Tilt Correction
	 * Must see obstacle 2 first (it can be long), then drive until it ends.
	 * --------------------------------------------------------- */
	t2_status("Back Face!      ");
	w = (watch_t){ .kind = WATCH_IR_PASS, .ir_side = inner, .thresh_cm = T2_BACK_CM,
	               .after_mm = T2_BACK_AFTER_MM, .sense_speed = T2_SENSE_FACE, .log_face = 1u };
	move_straight_watch(T2_BACK_MAX_MM, &w);
	t2_log_watch("back", &w);

	/* TILT: + means the face got further away along the run (drifting away) */
	int tilt_back = t2_face_tilt_deg(T2_FIT_MIN_SPAN_BACK_MM, "back");
	{
		float th = (float)abs(tilt_back) * (3.14159f / 180.0f);
		int32_t true_lat = (int32_t)((float)w.driven_mm * cosf(th));
		int32_t true_fwd = (int32_t)((float)w.driven_mm * sinf(th));
		accum_left    += dodge_left_2 ? -true_lat : true_lat;
		accum_forward += (tilt_back > 0) ? true_fwd : -true_fwd;
	}
	/* Kush (11-Oct): the "Fixing Tilt!" pivot that was here is gone: a forward
	   turn and a reverse turn of |tilt_back| degrees, opposite locks. It did
	   not turn the car. move_turn_deg() stops any turn of 45 deg or less once
	   the angle left is within its 3.3-4.7 deg braking lead, so a 1-3 deg
	   turn stopped on its first tick (the wheels went to lock and back, that
	   was all) and a 4-5 deg one turned 0.3-1.7 deg. Had both turns run, they
	   would have doubled the correction: reversing on the opposite lock keeps
	   rotating the same way. tilt_back is still measured and logged, the
	   dead reckoning above still uses it, and TURN 4 now corrects it. */

	/* ---------------------------------------------------------
	 * TURN 4: 90 Degrees Inward (East/West -> South)
	 * Kush (11-Oct): 90 + the back-face tilt, the same as the parking turn
	 * does with obstacle 1's. + = the car was heading away from the face, so
	 * it turns that much further to come home straight; - = it turns less.
	 * As part of a 90 the extra degrees really happen (the 2.5 deg braking
	 * lead of a big turn, not the 3.3-4.7 deg one that stopped the pivot).
	 * --------------------------------------------------------- */
	t2_status("Turn 4!         ");
	const int turn4_deg = 90 + tilt_back;   /* tilt_back is clamped to +-5 */
	snprintf(u, sizeof u, "[T2] turn4=%d (back tilt %d)\r\n", turn4_deg, tilt_back);
	command_send(u);
	move_turn_deg(turn_in, 1, turn4_deg);

	accum_forward -= R_in;
	accum_left    += dodge_left_2 ? -R_in : R_in;

	image_found = 0;

	/* ======================================================================
	 * RETURNING TO CARPARK
	 * ====================================================================== */
	/* The S-shift's size: s_fwd along the track, s_lat inward. */
	float rad_30 = 30.0f * (3.14159f / 180.0f);
	int32_t R_inward  = turn_in ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
	int32_t R_outward = (!turn_in) ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

	int32_t s_fwd = (int32_t)((R_inward + R_outward) * sinf(rad_30));
	int32_t s_lat = (int32_t)((R_inward + R_outward) * (1.0f - cosf(rad_30)));

	/* Drive some distance down the track (South) */
	accum_forward -= move_straight_measured(200);

	/* ---------------------------------------------------------
	 * THE INWARD S-SHIFT, only when the IR needs it.
	 * Kush (12-Oct): the shift moves the car s_lat (~85 mm) inward, and the
	 * reverse of s_lat after the turn behind obstacle 1 moves it back out
	 * again, so the search behind obstacle 1 starts at the same place with
	 * or without it. What it does buy is range: the IR passes obstacle 1
	 * ~37 cm away after it, ~45 cm without (60 cm obstacle 2, more if the car
	 * is heading a little outward). Without it the search therefore needs
	 * the full T2_OBS1_FAR_CM, which it only gets when the IR reads open
	 * space to the end of its range here. Simulated over 200 layouts with
	 * open space reading 80 / 70 / 65 cm: with the shift obstacle 1 was found
	 * 200 / 200 / 200 times, without it 200 / 169 / 100. It also keeps the
	 * car s_lat further from the side wall all the way to obstacle 1; with
	 * the walls at the rules' 50 cm minimum the return line is only a few cm
	 * from the wall without it (after a left arrow at obstacle 2). So it is
	 * skipped, saving ~2.6 s (two 30 deg turns and the reverse), only when
	 * the inner IR reads its full range AND the outer IR has the wall at
	 * least T2_RETURN_WALL_CM away; otherwise it runs as before.
	 * T2_RETURN_S_SHIFT 1 runs it every time.
	 * --------------------------------------------------------- */
	float ir_open = ir_filtered_cm(inner);   /* standing still, clear of obstacle 2 */
	const float ir_wall = ir_filtered_cm((inner == IR_SIDE_LEFT) ? IR_SIDE_RIGHT : IR_SIDE_LEFT);
	const uint8_t s_shift = (T2_RETURN_S_SHIFT ||
	                         (t2_obs1_range_cm(ir_open) < T2_OBS1_FAR_CM) ||
	                         (ir_wall < T2_RETURN_WALL_CM)) ? 1u : 0u;
	int32_t remaining_dist = d_after_dodge;
	if (remaining_dist < 0) remaining_dist = 0;

	if (s_shift) {
		t2_status("S-Shift In!     ");
		move_turn_deg(turn_in, 1, 30);
		move_turn_deg(!turn_in, 1, 30);

		accum_forward -= s_fwd;
		if (dodge_left_2) accum_left += s_lat; else accum_left -= s_lat;
		ir_open = ir_filtered_cm(inner);   /* read again from here, as before */
	} else {
		/* Straight on instead: the shift's forward part joins the blind
		   stretch of the search below, so it starts looking where it did. */
		remaining_dist += s_fwd;
		s_lat = 0;   /* nothing for the reverse after the turn to undo */
	}

	/* ---------------------------------------------------------
	 * The rest of the gap, then FIND OBSTACLE 1 (forward pass) -- one move.
	 * The IR is ignored for the first remaining_dist, exactly the stretch the
	 * old code drove blind before it started creeping.
	 * Kush (11-Oct): remaining_dist is now the distance measured to obstacle 2
	 * right after the dodge. It was dist_between - s_fwd + 200, and
	 * dist_between had the back-off added in, so once the car had backed
	 * off to the 30 cm standoff it came out the same (~430 mm) at every gap.
	 * Simulated, that carried the car past obstacle 1 before it started
	 * looking at gaps of 60-75 cm; the tested version's - 200 instead ran out
	 * of search (T2_OBS1_MAX_MM) beside it at 85-100 cm. The measured
	 * distance tracks the gap: the IR reaches obstacle 1 ~15 cm after the
	 * car starts looking, at every gap, and it found it at 56-100 cm for all
	 * four arrow pairs. Starts looking long before obstacle 1: add to it.
	 * Still beside obstacle 1 when the search ends: subtract.
	 * --------------------------------------------------------- */
	t2_status("Find Obs 1!     ");
	/* How far the IR may look for obstacle 1 (T2_OBS1_FAR_CM): set from what
	   it read above, standing still, before obstacle 1 is beside it. */
	const float obs1_cm = t2_obs1_range_cm(ir_open);
	snprintf(u, sizeof u, "[T2] obs1 search: open %.1fcm, counts under %.1fcm, wall %.1fcm, s-shift %s\r\n",
	         (double)ir_open, (double)obs1_cm, (double)ir_wall, s_shift ? "yes" : "skipped");
	command_send(u);
	w = (watch_t){ .kind = WATCH_IR_PASS, .ir_side = inner, .thresh_cm = obs1_cm,
	               .arm_mm = remaining_dist, .after_mm = T2_OBS1_AFTER_MM,
	               .sense_speed = T2_SENSE_OBS1, .log_face = T2_PARK_DIRECT ? 1u : 0u,
	               .min_seen_mm = T2_OBS1_MIN_SEEN_MM };
	move_straight_watch(remaining_dist + T2_OBS1_MAX_MM, &w);
	accum_forward -= w.driven_mm;
	t2_log_watch("obs1-pass", &w);

	/* Kush (14-Oct, T2_PARK_DIRECT): how far the IR was from obstacle 1's
	   side face as the car went past. */
	float side_cm = 0.0f;
	uint16_t side_n = 0u;
	const uint8_t direct = (T2_PARK_DIRECT && w.triggered &&
	                        watch_face_level(T2_FIT_TRIM, T2_DIRECT_MIN_N, &side_cm, &side_n)) ? 1u : 0u;
	t2_direct_used = direct;
	t2_direct_side_cm = side_cm;
	t2_direct_n = side_n;
	/* Turn into behind obstacle 1 (South -> East/West) */
	move_turn_deg(turn_in, 1, 90);
	accum_forward -= R_in; /* Sweeps South */
	accum_left    += dodge_left_2 ? R_in : -R_in;

	int final_turn_angle = 90;
	if (direct) {
		/* The rear-axle centre's distance from obstacle 1's centre line, less
		   what the two 90s cover, is what is left to drive between them
		   (negative: back). No tilt fit without the search, so the parking
		   turn below is a plain 90: the car comes out at the heading it
		   passed obstacle 1 with, plus whatever the two 90s differ by. */
		t2_status("Park Direct!    ");
		const int32_t lat = PILLAR_HALF_WIDTH + (int32_t)(side_cm * 10.0f) + IR_TO_CENTRE_MM;
		const int32_t mid = lat - (R_in + R_out);
		snprintf(u, sizeof u, "[T2] direct park: obs1 side %.1fcm (n=%u), axle %ldmm out, middle %ldmm\r\n",
		         (double)side_cm, (unsigned)side_n, (long)lat, (long)mid);
		command_send(u);
		if (mid >= T2_DIRECT_MIN_MM || mid <= -T2_DIRECT_MIN_MM) {
			int32_t d = move_straight_measured(mid);
			accum_left += dodge_left_2 ? d : -d;
		}
	} else {
		/* Move back some distance before starting search for obstacle 1.
		 * Kush (12-Oct): it only undid the S-shift, so with T2_RETURN_S_SHIFT 0
		 * (s_lat = 0) there is nothing to do. */
		if (s_lat > 0) {
			int32_t d = move_straight_measured(-s_lat);
			accum_left += dodge_left_2 ? d : -d;
		}

		/* ---------------------------------------------------------
		 * FIND OBS 1 AGAIN & DYNAMIC TILT CORRECTION
		 * --------------------------------------------------------- */
		t2_status("Obs 1 Again!    ");
		w = (watch_t){ .kind = WATCH_IR_PASS, .ir_side = inner, .thresh_cm = T2_OBS1_CM,
		               .after_mm = 0, .sense_speed = T2_SENSE_OBS1, .log_face = 1u };
		move_straight_watch(T2_OBS1_MAX_MM, &w);
		/* Facing lateral. Forward moves Left or Right. */
		accum_left += dodge_left_2 ? w.driven_mm : -w.driven_mm;
		t2_log_watch("obs1-back", &w);

		/* TILT MATH FOR FINAL PARKING TURN */
		int tilt_obs1 = t2_face_tilt_deg(T2_FIT_MIN_SPAN_OBS1_MM, "obs1");
		if (tilt_obs1 != 0) {
			t2_status("Align Final!    ");
			/* If distance increased, we drifted away.
			 * We need to turn MORE to perfectly face South. */
			final_turn_angle = 90 + tilt_obs1;
		}

		/* Reverse by Radius to give space to turn into carpark, plus whatever the
		 * watch drove past where the old creep would have stopped. */
		int32_t extra = 0;
		if (w.triggered) {
			int32_t lag_mm   = (int32_t)(w.trigger_speed_mm_tick * T2_IR_LAG_TICKS);
			int32_t old_stop = w.trigger_mm - lag_mm + T2_OBS1_STOP_PAST_EDGE_MM;
			extra = w.driven_mm - old_stop;
			if (extra < 0) extra = 0;
			if (extra > T2_OBS1_EXTRA_MAX_MM) extra = T2_OBS1_EXTRA_MAX_MM;
		}
		{
			int32_t d = move_straight_measured(-((R_out - PILLAR_HALF_WIDTH) + extra));
			accum_left += dodge_left_2 ? d : -d;
		}
		snprintf(u, sizeof u, "[T2] park turn=%d extra_back=%ldmm\r\n", final_turn_angle, (long)extra);
		command_send(u);
	}

	/* Turn to face the carpark (East/West -> South) using the CORRECTED angle! */
	move_turn_deg(turn_out, 1, final_turn_angle);

	/* We maintain the standard 90-degree R_out footprint for the grid tracker.
	 * A 5-degree shift creates virtually zero lateral error, and the forward
	 * distance is handled by the ultrasonic approach at the end. */
	accum_forward -= R_out;
	accum_left    += dodge_left_2 ? -R_out : R_out;

	/* ======================================================================
	 * FINAL PARKING SEQUENCE (With Tight-Space Adjuster)
	 * ====================================================================== */
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"PARKING...");
	OLED_Refresh_Gram();

	/* Drive the bulk of the distance, leaving a 25cm buffer for precision sensing */
	int32_t final_approach = accum_forward - 250;
	if (final_approach > 0) {
		move_straight_measured(final_approach);
	}

	/* ---------------------------------------------------------
	 * PARKING TIGHT FIT (Sideways Shift)
	 * --------------------------------------------------------- */
	/* Kush (12-Oct, T2_FEWER_PAUSES): no pause before these reads or before
	   the pings below. The car is standing still with its wheels centred,
	   and the move after them centres the steering and waits itself. */
//	servo_us(SERVO_CENTRE);
//	if (!T2_FEWER_PAUSES) HAL_Delay(150);
//
//	float park_ir_left  = ir_filtered_cm(IR_SIDE_LEFT);
//	float park_ir_right = ir_filtered_cm(IR_SIDE_RIGHT);
//
//	if (park_ir_left < 15.0f && park_ir_right >= 20.0f) {
//		t2_status("Shift Right!    ");
//		move_turn_deg(0, 1, 10);
//		move_turn_deg(1, 1, 10);
//	}
//	else if (park_ir_right < 15.0f && park_ir_left >= 20.0f) {
//		t2_status("Shift Left!     ");
//		move_turn_deg(1, 1, 10);
//		move_turn_deg(0, 1, 10);
//	}
//
//	servo_us(SERVO_CENTRE);
//	if (!T2_FEWER_PAUSES) HAL_Delay(150);

	/* Precision ultrasonic stop at the back wall: one move to T2_PARK_STOP_CM
	 * instead of 30 mm creeps, with the ultrasonic as a guard. */
	float usp = t2_front_cm();
	int32_t go = (usp > 0.0f) ? (int32_t)((usp - T2_PARK_STOP_CM) * 10.0f) : T2_PARK_NO_ECHO_MM;
	if (go > T2_PARK_MAX_MM) go = T2_PARK_MAX_MM;
	if (go > 0) {
		w = (watch_t){ .kind = WATCH_US_BELOW, .thresh_cm = T2_PARK_GUARD_CM,
		               .after_mm = 0, .sense_speed = T2_SENSE_PARK };
		move_straight_watch(go, &w);
	}

	/* Kush (12-Oct): PARKED, for main's tablet (Zhenxi's 67e6d5d there): the
	 * rules stop the timing when the car is in the carpark and stopped, which
	 * is now, as the move above has stopped and settled. Sent before the log
	 * line below (it waits ~200 ms for fresh echoes) and before the DONE that
	 * only comes once task_2() returns. The Pi relays it as STM,PARKED and the
	 * tablet stops its clock on it (on DONE only for firmware without it). Not
	 * sent after a STOP: the car is wherever it was stopped, not parked. */
	if (!motion_abort_requested()) command_send("PARKED\r\n");

	snprintf(u, sizeof u, "[T2] park us=%.1fcm drive=%ldmm, now %.1fcm\r\n",
	         (double)usp, (long)go, (double)ultrasonic_read_cm(2u, 200u));
	command_send(u);

	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"PARKED!");
	OLED_Refresh_Gram();
}

/**
 * @brief Function for Task 2 Fastest car task. Runs on the Task 2 speed
 *        profile and puts the previous one back afterwards.
 */
void task_2(void)
{
	const motion_profile_t *before = motion_get_profile();
	motion_set_profile(&MOTION_PROFILE_TASK2);
	task_2_run();
	motion_set_profile(before);
}

/**
 * @brief Task 2 with the arrows given instead of scanned (obstacle_nav.h).
 *        Everything else is task_2() as START2 runs it.
 */
void task_2_with_arrows(uint8_t arrow_obs1, uint8_t arrow_obs2)
{
	t2_given_arrow[0] = arrow_obs1;
	t2_given_arrow[1] = arrow_obs2;
	task_2();
	t2_given_arrow[0] = 0u;
	t2_given_arrow[1] = 0u;
}
