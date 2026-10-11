#ifndef OBSTACLE_NAV_H
#define OBSTACLE_NAV_H

#include <stdint.h>

/**
 * @brief Navigates to obstacle coordinates relative to robot start and circles all 4 faces.
 * @param target_x_mm Lateral distance in mm (+ = right, - = left, 0 = straight ahead)
 * @param target_y_mm Forward distance in mm
 */
void navigate_and_inspect_obstacle(int32_t target_x_mm, int32_t target_y_mm);

/**
 * @brief Function for Task 2 Fastest car task
 */
void task_2(void);

/* Kush (09-Oct): the two arrows, as the image IDs the Pi sends for them
   (IM039 = left, IM038 = right). */
#define T2_ARROW_LEFT   39u
#define T2_ARROW_RIGHT  38u

/**
 * @brief Task 2 with both arrows given instead of read by the Pi: no SCAN, no
 *        waiting for an IM, no retries. The car still stops at each standoff,
 *        so every move after it is the one a real run makes. For runs without
 *        the Pi (CAL_TASK2 in main.c).
 * @param arrow_obs1 T2_ARROW_LEFT or T2_ARROW_RIGHT for obstacle 1
 * @param arrow_obs2 the same for obstacle 2
 */
void task_2_with_arrows(uint8_t arrow_obs1, uint8_t arrow_obs2);

#endif /* OBSTACLE_NAV_H */
