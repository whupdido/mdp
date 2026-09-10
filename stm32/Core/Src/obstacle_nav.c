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

/**
 * @brief Task 2 Image rec helper
 */
int task_2_image_rec(void){
	char buf[64];
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"SCANNING...");
	OLED_Refresh_Gram();
	/* Reset the flag before we start waiting */
	image_found = 0;

	uint32_t start_time = HAL_GetTick();
	const uint32_t TIMEOUT_MS = 3000; /* Wait up to 3 seconds */
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

			//command_send(">> Target Matched!\r\n");

			HAL_Delay(1000); /* Pause so you can read OLED before driving away */
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

    /* Execute the physical turn */
    HAL_Delay(150);
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
    /* 1. We physically turn opposite to the original dodge to straighten out */
    int turn_left = !original_dodge_left;
    HAL_Delay(150);
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

/**
 * @brief Function for Task 2 Fastest car task
 */
void task_2(void) {
	int32_t accum_forward = 0;
	int32_t accum_left = 0;
	char buf[64];

	/* ======================================================================
	 * OBSTACLE 1 (10x10cm) - The "Reverse Corridor" Dodge
	 * ====================================================================== */
	trigger_ultrasonic();
	HAL_Delay(60);

	/* Stop 30cm (300mm) away to read the image. */
	int32_t dist_forward = (ultrasonic_distance_cm * 10) - 300;

	if (dist_forward > 0) {
		move_straight_mm(dist_forward);
		accum_forward += dist_forward;
	}

	int success = task_2_image_rec();

	/* Default to dodging right if image fails */
	int dodge_left = (image_found == 39) ? 1 : 1; // RMB TO CHANGE!!!!

	OLED_Clear();
	OLED_ShowString(0, 20, dodge_left ? (const uint8_t *)"Dodging Left" : (const uint8_t *)"Dodging Right");
	OLED_Refresh_Gram();

	/* ---------------------------------------------------------
	 * PHASE 1: Outward Dodge & IR Scanning
	 * --------------------------------------------------------- */
	int current_angle = 45;
	task_2_turn(dodge_left, 1, current_angle, 0, &accum_forward, &accum_left);
	HAL_Delay(100);
	move_straight_mm(150);
	float rad_45 = 45.0f * (3.14159f / 180.0f);
	accum_forward += (int32_t)(150 * cosf(rad_45));
	accum_left    += (int32_t)(150 * sinf(rad_45)) * (dodge_left ? 1 : -1);
	uint8_t obs1_seen = 0;
	float inner_ir = dodge_left ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();
	if (inner_ir <= 25.0f){
		uint8_t creep_times = 0;
		int32_t diag_driven = 150;
		/* Fast 60mm checks to accurately catch the 10cm block */
		while (diag_driven <= 300) {

			if (!obs1_seen && inner_ir < 45.0f) {
				obs1_seen = 1; /* Found the block! */
			}
			else if (obs1_seen && inner_ir >= 45.0f) {
				/* FALLING EDGE! We just cleared the 10cm block. */
				break;
			}

			move_straight_mm(75);
			diag_driven += 75;
			creep_times += 1;

			accum_forward += (int32_t)(75 * cosf(rad_45));
			accum_left    += (int32_t)(75 * sinf(rad_45)) * (dodge_left ? 1 : -1);
		}
	}

	/* ---------------------------------------------------------
	 * PHASE 2: Straighten Out in the Side Lane
	 * --------------------------------------------------------- */
	task_2_straighten(dodge_left, current_angle, &accum_forward, &accum_left);


	/* ---------------------------------------------------------
	 * PHASE 3: The "Reverse Corridor" Space Saver
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Reversing!   ");
	OLED_Refresh_Gram();

	int32_t reverse_dist = 70;

	/* MECHANICS FIX 1: Let the servo physically return to center
	 * before reversing so we don't swing a Ghost Arc! */
	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	move_straight_mm(-reverse_dist);
	accum_forward -= reverse_dist;


	/* ---------------------------------------------------------
	 * PHASE 4: Tightly Hook to the Center
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Centering!   ");
	OLED_Refresh_Gram();

	int turn_inward = !dodge_left;

	/* 4A. Turn 45 Inward */
	task_2_turn(turn_inward, 1, 45, 0, &accum_forward, &accum_left);

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	/* 4C. Final Outward Turn to Straighten exactly on the Center Line! */
	task_2_straighten(turn_inward, 45, &accum_forward, &accum_left);

	/* Go back to 30cm away from target */
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"Calibrating...");
	OLED_Refresh_Gram();

	trigger_ultrasonic();
	HAL_Delay(60);
	int32_t dist_between = ultrasonic_distance_cm * 10 + 250;

	/* We want to sit exactly 30cm (300mm) away from Obstacle 2 */
	int32_t dist_to_target = (ultrasonic_distance_cm * 10) - 300;

	/* If dist_to_target is positive, we drive forward. If negative, we reverse! */
	if (dist_to_target != 0 && ultrasonic_distance_cm > 5.0f && ultrasonic_distance_cm < 150.0f) {
		move_straight_mm(dist_to_target);
		accum_forward += dist_to_target;
	}
	if (dist_to_target < 0){
		dist_between -= dist_to_target;
	}

	image_found = 0;

	/* ======================================================================
	 * OBSTACLE 2: The Perfect Orthogonal Box-Wrap
	 * ====================================================================== */
	success = task_2_image_rec();

	int dodge_left_2 = (image_found == 39) ? 1 : 1; // RMB TO CHANGE!!!!
	int turn_out = dodge_left_2 ? 1 : 0;
	int turn_in  = !dodge_left_2;

	int32_t R_out = turn_out ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
	int32_t R_in  = turn_in  ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

	OLED_Clear();
	OLED_ShowString(0, 20, dodge_left_2 ? (const uint8_t *)"Obs 2: Left" : (const uint8_t *)"Obs 2: Right");
	OLED_Refresh_Gram();

	/* ---------------------------------------------------------
	 * TURN 1: 90 Degrees Outward (North -> West/East)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Turn 1!      ");
	OLED_Refresh_Gram();

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	move_turn_deg(turn_out, 1, 90);
	accum_forward += R_out;
	accum_left    += dodge_left_2 ? R_out : -R_out;

	// Reverse slightly if consecutive
	if (dodge_left == dodge_left_2){
		servo_us(SERVO_CENTRE);
		HAL_Delay(150);
		move_straight_mm(-100);
		if (dodge_left_2){
			accum_left -= 100;
		}
		else {
			accum_left += 100;
		}
	}

	/* ---------------------------------------------------------
	 * CREEP 1: The Front Face (20cm steps)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Front Face!  ");
	OLED_Refresh_Gram();

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	int32_t creep_driven = 0;
	while (creep_driven < 800) {
		float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir >= 35.0f) {
			break; /* No extra clearance needed! */
		}

		move_straight_mm(200);
		creep_driven += 200;

		/* Facing West/East: pure lateral movement */
		if (dodge_left_2) accum_left += 200; else accum_left -= 200;
	}


	/* ---------------------------------------------------------
	 * TURN 2: 90 Degrees Inward (West/East -> North)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Turn 2!      ");
	OLED_Refresh_Gram();

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	move_turn_deg(turn_in, 1, 90);

	accum_forward += R_in;
	accum_left    += dodge_left_2 ? R_in : -R_in;


	/* ---------------------------------------------------------
	 * CREEP 2: The Side Face (10cm steps)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Side Face!   ");
	OLED_Refresh_Gram();

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	creep_driven = 0;
	while (creep_driven < 300) {
		float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir >= 45.0f) {
			break;
		}

		move_straight_mm(100);
		creep_driven += 100;
		accum_forward += 100;
	}


	/* ---------------------------------------------------------
	 * TURN 3: 90 Degrees Inward (North -> East/West)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Turn 3!      ");
	OLED_Refresh_Gram();

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	move_turn_deg(turn_in, 1, 90);

	accum_forward += R_in;
	accum_left    += dodge_left_2 ? -R_in : R_in;


	/* ---------------------------------------------------------
	 * CREEP 3: The Back Face & Dynamic Tilt Correction
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Back Face!   ");
	OLED_Refresh_Gram();
	uint8_t obs2_seen = 0; //MUST SEE OBS2 FIRST DUE TO BIG LENGTH

	/* Take initial baseline IR reading for tilt detection */
	float ir_start = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();
	float ir_end = ir_start;

	creep_driven = 0;
	while (creep_driven < 1200) {
		float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir >= 50.0f && obs2_seen) {
			break; /* Cleared the back edge */
		}
		if (inner_ir <= 50.0f) obs2_seen = 1;
		ir_end = inner_ir; /* Record last valid measurement on the block */

		move_straight_mm(200);
		creep_driven += 200;
	}

	/* TILT MATH: If distance driven is long enough, calculate and correct chassis tilt */
	if (creep_driven >= 200) {
		float diff_mm = (ir_end - ir_start) * 10.0f;
		float theta_rad = atanf(fabs(diff_mm) / (float)creep_driven);
		int tilt_deg = (int)(theta_rad * (180.0f / 3.14159f));

		/* LIMIT TILT CORRECTION TO MAXIMUM 5 DEGREES */
		if (tilt_deg > 5) {
			tilt_deg = 5;
		}

		if (tilt_deg > 0) {
			OLED_ShowString(0, 40, (const uint8_t *)"Fixing Tilt! ");
			OLED_Refresh_Gram();

			int drifted_away = (diff_mm > 0) ? 1 : 0;
			int pivot_left = dodge_left_2 ? 0 : 1;
			if (!drifted_away) pivot_left = !pivot_left;

			/* Zero-space pivot to snap exactly to East/West */
			move_turn_deg(pivot_left, 1, tilt_deg);
			move_turn_deg(!pivot_left, 0, tilt_deg);

			/* Update true vector footprint */
			int32_t true_lat = (int32_t)((float)creep_driven * cosf(theta_rad));
			int32_t true_fwd = (int32_t)((float)creep_driven * sinf(theta_rad));

			if (dodge_left_2) accum_left -= true_lat; else accum_left += true_lat;
			if (drifted_away) accum_forward += true_fwd; else accum_forward -= true_fwd;
		} else {
			if (dodge_left_2) accum_left -= creep_driven; else accum_left += creep_driven;
		}
	} else {
		/* Failsafe update */
		if (dodge_left_2) accum_left -= creep_driven; else accum_left += creep_driven;
	}


	/* ---------------------------------------------------------
	 * TURN 4: 90 Degrees Inward (East/West -> South)
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"Turn 4!      ");
	OLED_Refresh_Gram();

	move_turn_deg(turn_in, 1, 90);

	accum_forward -= R_in;
	accum_left    += dodge_left_2 ? -R_in : R_in;

	image_found = 0;


	/* ======================================================================
	 * RETURNING TO CARPARK
	 * ====================================================================== */
	/* Drive some distance down the track (South) */
	move_straight_mm(200);
	accum_forward -= 200;

	/* ---------------------------------------------------------
	 * THE INWARD S-SHIFT
	 * --------------------------------------------------------- */
	OLED_ShowString(0, 40, (const uint8_t *)"S-Shift In!  ");
	OLED_Refresh_Gram();

	move_turn_deg(turn_in, 1, 30);
	move_turn_deg(!turn_in, 1, 30);

	float rad_30 = 30.0f * (3.14159f / 180.0f);
	int32_t R_inward  = turn_in ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
	int32_t R_outward = (!turn_in) ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

	int32_t s_fwd = (int32_t)((R_inward + R_outward) * sinf(rad_30));
	int32_t s_lat = (int32_t)((R_inward + R_outward) * (1.0f - cosf(rad_30)));

	accum_forward -= s_fwd;
	if (dodge_left_2) accum_left += s_lat; else accum_left -= s_lat;

	/* Drive the remaining half of the gap */
	int remaining_dist = dist_between - s_fwd - 200;
	if (remaining_dist > 0) {
		move_straight_mm(remaining_dist);
		accum_forward -= remaining_dist;
	}

	/* ---------------------------------------------------------
	 * FIND OBSTACLE 1 (Forward pass)
	 * --------------------------------------------------------- */
	obs1_seen = 0;
	creep_driven = 0;
	while (creep_driven < 500) {
		float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir >= 55.0f && obs1_seen==1) break;
		else if (inner_ir <= 55.0f) obs1_seen = 1;

		move_straight_mm(50);
		creep_driven += 50;
		accum_forward -= 50;
	}

	/* Turn into behind obstacle 1 (South -> East/West) */
	move_turn_deg(turn_in, 1, 90);
	accum_forward -= R_in; /* Sweeps South */
	accum_left    += dodge_left_2 ? R_in : -R_in;

	/* Move back some distance before starting search for obstacle 1 */
	move_straight_mm(-s_lat);
	if (dodge_left_2) accum_left -= s_lat; else accum_left += s_lat;


	/* ---------------------------------------------------------
	 * FIND OBS 1 AGAIN & DYNAMIC TILT CORRECTION
	 * --------------------------------------------------------- */
	obs1_seen = 0;
	creep_driven = 0;
	int32_t obs1_seen_dist = 0;
	float obs1_ir_start = 0.0f;
	float obs1_ir_end = 0.0f;

	while (creep_driven < 500) {
		float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir <= 55.0f) {
			if (!obs1_seen) {
				obs1_seen = 1;
				obs1_ir_start = inner_ir; /* Record exact moment it appears */
			}
			obs1_ir_end = inner_ir;       /* Continually update while visible */
			obs1_seen_dist += 50;
		}
		else if (inner_ir >= 55.0f && obs1_seen == 1) {
			break; /* Cleared the block */
		}

		move_straight_mm(50);
		creep_driven += 50;

		/* Facing lateral. Forward moves Left or Right. */
		if (dodge_left_2) accum_left += 50; else accum_left -= 50;
	}

	/* TILT MATH FOR FINAL PARKING TURN */
	int final_turn_angle = 90;

	/* Because Obs 1 is 10cm, we need at least 2 steps (100mm) to get a valid angle */
	if (obs1_seen_dist >= 100) {
		float diff_mm = (obs1_ir_end - obs1_ir_start) * 10.0f;

		/* We subtract 50 from the distance because the first reading was at 0mm,
		 * and the final reading was recorded 50mm before the current total */
		float theta_rad = atanf(fabs(diff_mm) / (float)(obs1_seen_dist - 50));
		int tilt_deg = (int)(theta_rad * (180.0f / 3.14159f));

		if (tilt_deg > 5) tilt_deg = 5;

		if (tilt_deg > 0) {
			OLED_ShowString(0, 40, (const uint8_t *)"Align Final! ");
			OLED_Refresh_Gram();

			/* If distance increased, we drifted away.
			 * We need to turn MORE to perfectly face South. */
			if (diff_mm > 0.0f) {
				final_turn_angle = 90 + tilt_deg;
			} else {
				final_turn_angle = 90 - tilt_deg;
			}
		}
	}

	/* Reverse by Radius to give space to turn into carpark */
	move_straight_mm(-(R_out - PILLAR_HALF_WIDTH));
	if (dodge_left_2) accum_left -= (R_out - PILLAR_HALF_WIDTH); else accum_left += (R_out - PILLAR_HALF_WIDTH);

	/* Turn to face the carpark (East/West -> South) using the CORRECTED angle! */
	move_turn_deg(turn_out, 1, final_turn_angle);

	/* We maintain the standard 90-degree R_out footprint for the grid tracker.
	 * A 5-degree shift creates virtually zero lateral error, and the forward
	 * distance is handled flawlessly by the upcoming Ultrasonic loop! */
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
		move_straight_mm(final_approach);
	}

	/* ---------------------------------------------------------
	 * PARKING TIGHT FIT (Sideways Shift)
	 * --------------------------------------------------------- */
	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	float park_ir_left = get_sharp_ir_left_cm();
	float park_ir_right = get_sharp_ir_right_cm();

	if (park_ir_left < 15.0f && park_ir_right >= 20.0f) {
		OLED_ShowString(0, 40, (const uint8_t *)"Shift Right! ");
		OLED_Refresh_Gram();
		move_turn_deg(0, 1, 10);
		move_turn_deg(1, 1, 10);
	}
	else if (park_ir_right < 15.0f && park_ir_left >= 20.0f) {
		OLED_ShowString(0, 40, (const uint8_t *)"Shift Left!  ");
		OLED_Refresh_Gram();
		move_turn_deg(1, 1, 10);
		move_turn_deg(0, 1, 10);
	}

	servo_us(SERVO_CENTRE);
	HAL_Delay(150);

	/* Precision Ultrasonic loop to stop exactly at the back wall */
	while (1) {
		trigger_ultrasonic();
		HAL_Delay(60);

		if (ultrasonic_distance_cm <= 10.0f) {
			motion_stop();
			break;
		}

		if (ultrasonic_distance_cm <= 8.0f) {
			motion_stop();
			break;
		}

		move_straight_mm(30);
	}

	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"PARKED!");
	OLED_Refresh_Gram();
}
