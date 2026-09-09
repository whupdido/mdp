#include "obstacle_nav.h"
#include "control.h"
#include "calib.h"
#include "oled.h"
#include "command.h"
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
 * @brief Task 2 turn helper
 *
 * @param left           1 for a left turn, 0 for a right turn.
 * @param forward        1 for driving forward, 0 for driving backward.
 * @param degrees 		 Turn degree
 * @param accum_forward  Pointer to the forward/backward displacement tracker.
 * @param accum_left     Pointer to the left/right displacement tracker.
 * @return int           1 on success.
 */
int task_2_turn(int left, int forward, int degrees, int32_t* accum_forward, int32_t* accum_left) {
    /* Convert degrees to radians for math functions */
    float rad = (float)degrees * (3.14159f / 180.0f);
    float sin_val = sinf(rad);
    float cos_val = cosf(rad);

    int32_t R = 0;
    if (forward) {
        R = left ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;
    } else {
        R = left ? TURN_RADIUS_BL_MM : TURN_RADIUS_BR_MM;
    }

    /*
     * Kinematics of a turning car:
     * Forward displacement = R * sin(theta)
     * Lateral displacement = R * (1 - cos(theta))
     */
    int32_t d_forward = (int32_t)(R * sin_val);
    int32_t d_left    = (int32_t)(R * (1.0f - cos_val));

    /* Apply directional signs */
    if (!forward) d_forward = -d_forward;
    if (!left)    d_left    = -d_left;

    *accum_forward += d_forward;
    *accum_left    += d_left;

    /* Execute the physical turn */
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
 * @brief Executes a 45-degree lane change dynamically using IR sensors.
 */
void dodge_obstacle(int dodge_left, int32_t* accum_forward, int32_t* accum_left) {
    /* 1. Turn 45-degrees outward */
    task_2_turn(dodge_left, 1, 45, accum_forward, accum_left);

    /* 2. Dynamic Diagonal Drive using IR Sensor */
    int32_t diag_driven = 0;
    uint8_t obstacle_seen = 0;

    /* Loop with a failsafe to prevent driving into the outer wall */
    while (diag_driven < 750) {
        float inner_ir = dodge_left ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

        /* State Machine: Wait to see the obstacle, then wait for it to disappear */
        if (inner_ir < 45.0f) {
            obstacle_seen = 1;
        } else if (obstacle_seen && inner_ir >= 45.0f) {
            /* FALLING EDGE! The obstacle just ended. */
            /* Drive 150mm more to ensure the rear wheels clear the corner */
            move_straight_mm(150);
            diag_driven += 150;
            break;
        }

        /* If we drive 400mm diagonally and STILL haven't seen it,
         * we probably bypassed a very small obstacle entirely. Stop driving outward. */
        if (diag_driven > 400 && !obstacle_seen) {
            break;
        }

        /* Creep forward in small increments while scanning */
        move_straight_mm(30);
        diag_driven += 30;
    }

    /* Update grid accumulators using actual distance driven */
    float rad = 45.0f * (3.14159f / 180.0f);
    *accum_forward += (int32_t)(diag_driven * cosf(rad));
    *accum_left    += (int32_t)(diag_driven * sinf(rad)) * (dodge_left ? 1 : -1);

    /* 3. Turn 45-degrees inward to straighten out parallel to track */
    task_2_straighten(dodge_left, 45, accum_forward, accum_left);
}

/**
 * @brief Crosses the center line dynamically using IR sensors to clear the obstacle.
 */
void crossover_obstacle(int dodge_left, int32_t* accum_forward, int32_t* accum_left) {
    /* 1. Turn 45-degrees outward toward the opposite lane */
    task_2_turn(dodge_left, 1, 45, accum_forward, accum_left);

    /* 2. Dynamic Diagonal Drive using IR Sensor */
    int32_t diag_driven = 0;
    uint8_t obstacle_seen = 0;

    /* Loop with a 900mm absolute failsafe to prevent infinite driving */
    while (diag_driven < 900) {
        /* If dodging left, the obstacle is on our right side */
        float inner_ir = dodge_left ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

        /* State Machine: Look for the obstacle, then look for it to disappear */
        if (inner_ir < 45.0f) {
            obstacle_seen = 1; /* We are alongside the obstacle */
        } else if (obstacle_seen && inner_ir >= 45.0f) {
            /* FALLING EDGE! The obstacle just ended. */
            /* Drive 150mm more to ensure the rear wheels clear the corner before turning */
            move_straight_mm(50);
            diag_driven += 50;
            break;
        }

        /* Creep forward in small increments while scanning */
        move_straight_mm(50);
        diag_driven += 50;
    }

    /* Update grid accumulators using actual distance driven */
    float rad = 45.0f * (3.14159f / 180.0f);
    *accum_forward += (int32_t)(diag_driven * cosf(rad));
    *accum_left    += (int32_t)(diag_driven * sinf(rad)) * (dodge_left ? 1 : -1);

    /* 3. Turn 45-degrees inward to straighten out */
    task_2_straighten(dodge_left, 45, accum_forward, accum_left);
}

/**
 * @brief Function for Task 2 Fastest car task
 */
void task_2(void) {
    int32_t accum_forward = 0;
    int32_t accum_left = 0;
    char buf[64];

    /* ======================================================================
     * OBSTACLE 1
     * ====================================================================== */
    trigger_ultrasonic();
    HAL_Delay(60);

    /* Stop 30cm (300mm) away to read the image. */
    int32_t dist_forward = (ultrasonic_distance_cm * 10) - 300;

    move_straight_mm(dist_forward);
    accum_forward += dist_forward;

    int success = task_2_image_rec();

    /* Default to dodging right if image fails */
    int dodge_left = (image_found == 39) ? 1 : 0;

    OLED_Clear();
    OLED_ShowString(0, 20, dodge_left ? (const uint8_t *)"Dodging Left" : (const uint8_t *)"Dodging Right");
    OLED_Refresh_Gram();

    /* Execute the tight S-Curve */
    dodge_obstacle(dodge_left, &accum_forward, &accum_left);
    image_found = 0;

    /* --- OBSTACLE 2 --- */
	int current_lane = dodge_left ? 1 : -1;

	/* 1. The "Zero-Space" 30-Degree Pivot (Tilt Inward) */
	/* A 15-deg forward turn + 15-deg reverse turn = 30-deg heading change! */
	task_2_turn(!dodge_left, 1, 15, &accum_forward, &accum_left); /* Forward Inward */
	task_2_turn(dodge_left,  0, 15, &accum_forward, &accum_left); /* Reverse Outward */

	trigger_ultrasonic();
	HAL_Delay(60);
	OLED_Clear();
	snprintf(buf, sizeof(buf), "Dist: %.1f cm   ", ultrasonic_distance_cm);
	OLED_ShowString(0, 20, (const uint8_t *)buf);
	OLED_Refresh_Gram();

	/* Clamp distance at 0 to prevent reversing into Obs 1 */
	dist_forward = (ultrasonic_distance_cm * 10) - 400;
	if (dist_forward > 0) {
		move_straight_mm(dist_forward);

		/* We are now tilted by a full 30 degrees! Use 30 for the math. */
		float rad_30 = 30.0f * (3.14159f / 180.0f);
		accum_forward += (int32_t)(dist_forward * cosf(rad_30));
		accum_left += (int32_t)(dist_forward * sinf(rad_30)) * (dodge_left ? -1 : 1);
	}

	success = task_2_image_rec();

	int dodge_left_2 = (image_found == 39) ? 1 : 0;
	int target_lane = dodge_left_2 ? 1 : -1;

	OLED_Clear();
	OLED_ShowString(0, 20, dodge_left_2 ? (const uint8_t *)"Obs 2: Left" : (const uint8_t *)"Obs 2: Right");
	OLED_Refresh_Gram();

	/* 2. STRAIGHTEN OUT: The Inverse 30-Degree Pivot */
	/* Undo the 30-degree tilt using the exact same space-saving maneuver */
	task_2_turn(dodge_left,  1, 15, &accum_forward, &accum_left); /* Forward Outward */
	task_2_turn(!dodge_left, 0, 15, &accum_forward, &accum_left); /* Reverse Inward */

	/* 3. YOUR LANE LOGIC */
	if (current_lane == target_lane) {
		/* CONSECUTIVE ARROWS: Shift wider, then dynamically drive past. */
		OLED_ShowString(0, 40, (const uint8_t *)"Shifting Wider!");
		OLED_Refresh_Gram();

		/* Step A: Turn 15 degrees further outward */
		task_2_turn(dodge_left_2, 1, 15, &accum_forward, &accum_left);

		/* Step B: Short diagonal to gain lateral clearance */
		int32_t shift_diag = 100;
		move_straight_mm(shift_diag);

		float rad_15 = 15.0f * (3.14159f / 180.0f);
		accum_forward += (int32_t)(shift_diag * cosf(rad_15));
		accum_left    += (int32_t)(shift_diag * sinf(rad_15)) * (dodge_left_2 ? 1 : -1);

		/* Step C: Turn 15 degrees inward to straighten back out */
		task_2_straighten(dodge_left_2, 15, &accum_forward, &accum_left);

		/* Step D: DYNAMIC straight drive past the obstacle */
		int32_t straight_driven = 0;
		uint8_t obstacle_seen = 0;

		/* Loop with a 750mm failsafe */
		while (straight_driven < 750) {
			float inner_ir = dodge_left_2 ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

			if (inner_ir < 45.0f) {
				obstacle_seen = 1;
			} else if (obstacle_seen && inner_ir >= 45.0f) {
				/* Obstacle cleared! */
				move_straight_mm(150); /* Rear wheel safety margin */
				straight_driven += 150;
				break;
			}
			move_straight_mm(50);
			straight_driven += 50;
		}
		accum_forward += straight_driven;

	} else {
		/* ALTERNATING ARROWS: Execute dynamic crossover. */
		OLED_ShowString(0, 40, (const uint8_t *)"Crossing Over!");
		OLED_Refresh_Gram();
		crossover_obstacle(dodge_left_2, &accum_forward, &accum_left);
	}
	/* ======================================================================
	 * THE BOX U-TURN (Around the back of Obstacle 2)
	 * ====================================================================== */

	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"BOX U-TURN...");
	OLED_Refresh_Gram();

	int on_left_side = (accum_left > 0) ? 1 : 0;

	/* Step 1: Turn 90 degrees inward */
	task_2_turn(!on_left_side, 1, 90, &accum_forward, &accum_left);

	/* Step 2 & 3: Drive across the back of Obstacle 2 until IR clears it */
	int32_t cross_driven = 0;
	uint8_t obs_seen = 0;

	while (cross_driven < 800) {
		float inner_ir = on_left_side ? get_sharp_ir_right_cm() : get_sharp_ir_left_cm();

		if (inner_ir > 10.0f && inner_ir < 60.0f) {
			obs_seen = 1;
		} else if (obs_seen && inner_ir >= 60.0f) {
			break; /* FALLING EDGE! */
		}
		move_straight_mm(50);
		cross_driven += 50;
	}

	/* Step 4: Clear the rear wheels! */
//	move_straight_mm(200);
//	cross_driven += 200;

	if (on_left_side) {
		accum_left -= cross_driven;
	} else {
		accum_left += cross_driven;
	}

	/* Step 5: Second 90 degree turn to face the carpark */
	/* CRITICAL FIX: Bypass task_2_turn to manually subtract the radius */
	int turn_left = !on_left_side;
	move_turn_deg(turn_left, 1, 90);

	int32_t R2 = turn_left ? TURN_RADIUS_FL_MM : TURN_RADIUS_FR_MM;

	/* Turning to face backward brings us physically closer to the start line (-Y) */
	accum_forward -= R2;

	/* Sweeping down the track pushes us further out laterally */
	if (on_left_side) {
		accum_left -= R2;
	} else {
		accum_left += R2;
	}

	/* ======================================================================
	 * RETURNING TO CARPARK
	 * ====================================================================== */
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"Returning...");
	OLED_Refresh_Gram();

	/* 1. Safely bypass Obstacle 1 by staying in the side lane!
	 * We drive straight back until accum_forward is 700mm.
	 * This guarantees we have driven completely past Obstacle 1,
	 * dropping us safely into the void right in front of the carpark. */
	int32_t return_dist = accum_forward - 600;
	if (return_dist > 0) {
		move_straight_mm(return_dist);
		accum_forward -= return_dist;
	}

	/* ======================================================================
	 * THE 90-DEGREE MULTI-POINT PIVOT RECENTER
	 * ====================================================================== */
	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"Centering...");
	OLED_Refresh_Gram();

	if (accum_left > 40 || accum_left < -40) {
		int on_left_side = (accum_left > 0) ? 1 : 0;

		/* We are facing backward. Grid Right is the car's physical LEFT. */
		int turn_dir = on_left_side ? 1 : 0;

		/* --- DYNAMIC KINEMATIC CALCULATIONS --- */
		float rad45 = 45.0f * (3.14159f / 180.0f);
		float sin45 = sinf(rad45);
		float k45   = 1.0f - cosf(rad45);

		int32_t r_fwd_1, r_rev_2, r_fwd_3;

		if (turn_dir == 1) {
			/* Turn_dir = 1 (Left).
			 * 1st: Forward Left. 2nd: Reverse Right. Final: Forward Right. */
			r_fwd_1 = TURN_RADIUS_FL_MM;
			r_rev_2 = TURN_RADIUS_BR_MM;
			r_fwd_3 = TURN_RADIUS_FR_MM;
		} else {
			/* Turn_dir = 0 (Right).
			 * 1st: Forward Right. 2nd: Reverse Left. Final: Forward Left. */
			r_fwd_1 = TURN_RADIUS_FR_MM;
			r_rev_2 = TURN_RADIUS_BL_MM;
			r_fwd_3 = TURN_RADIUS_FL_MM;
		}

		/* Project the 45-degree arcs onto the global grid */
		float pivot_lat = (r_fwd_1 * k45) - (r_rev_2 * sin45);
		float pivot_fwd = (r_fwd_1 * sin45) - (r_rev_2 * k45);

		int32_t pivot_fwd_shift = (int32_t)pivot_fwd;
		int32_t fixed_lat_shift = (int32_t)(r_fwd_3 + pivot_lat);
		int32_t final_arc_fwd   = r_fwd_3;
		/* -------------------------------------- */

		/* STEP A: The 90-Degree Pivot (45 Forward, 45 Reverse) */
		move_turn_deg(turn_dir, 1, 45);
		move_turn_deg(!turn_dir, 0, 45);

		/* STEP B: Calculate and drive the exact straight distance */
		int32_t total_lat_needed = on_left_side ? accum_left : -accum_left;

		/* --- THE REAL-WORLD EMPIRICAL TUNERS ---
		 * LATERAL_SCALAR shrinks the accumulated error to compensate for
		 * servo-slew radius expansion during the S-Curve dodges.
		 * SLIP_COMPENSATION subtracts fixed mm to account for tire scrub
		 * during the aggressive 90-degree pivot maneuvers.
		 */
		const float LATERAL_SCALAR = 0.85f;   /* Tune this! Range: 0.80 to 0.95 */
		const int32_t SLIP_COMPENSATION = 50; /* Tune this! Fixed mm offset */

		int32_t tuned_lat_needed = (int32_t)(total_lat_needed * LATERAL_SCALAR);
		int32_t straight_drive = tuned_lat_needed - fixed_lat_shift - SLIP_COMPENSATION;

		/* Drive the remaining distance across the track to hit exact 0 */
		if (straight_drive > 0) {
			move_straight_mm(straight_drive);
		} else if (straight_drive < 0) {
			/* Failsafe: Drive backward to correct an overshoot */
			move_straight_mm(straight_drive);
		}

		/* STEP C: Final 90-degree forward turn to face the carpark */
		move_turn_deg(!turn_dir, 1, 90);

		/* Update the global tracking grid perfectly */
		accum_left = 0;
		accum_forward -= (pivot_fwd_shift + final_arc_fwd);
	}

	/* 3. Final approach to the carpark threshold */
	int32_t final_approach = accum_forward - 150;
	if (final_approach > 0) {
		move_straight_mm(final_approach);
		accum_forward -= final_approach;
	}

	uint8_t inside_carpark = 0;
	/* 4. The Precision Parking Loop */
	while (1) {
		float left_wall  = get_sharp_ir_left_cm();
		float right_wall = get_sharp_ir_right_cm();

		trigger_ultrasonic();
		HAL_Delay(60);

		if (!inside_carpark) {
			if (left_wall < 25.0f && right_wall < 25.0f) {
				inside_carpark = 1;
				command_send("[INFO] Carpark Walls Detected!\r\n");
			}
		}

		if (inside_carpark && ultrasonic_distance_cm <= 15) {
			motion_stop();
			break;
		}

		if (ultrasonic_distance_cm <= 8) {
			motion_stop();
			break;
		}

		move_straight_mm(30);
	}

	OLED_Clear();
	OLED_ShowString(0, 0, (const uint8_t *)"PARKED!");
	OLED_Refresh_Gram();
}
