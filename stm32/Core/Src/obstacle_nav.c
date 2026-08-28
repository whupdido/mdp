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
    safe_move_straight_mm(230);
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
