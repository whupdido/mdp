#ifndef OBSTACLE_NAV_H
#define OBSTACLE_NAV_H

#include <stdint.h>

/**
 * @brief Navigates to obstacle coordinates relative to robot start and circles all 4 faces.
 * @param target_x_mm Lateral distance in mm (+ = right, - = left, 0 = straight ahead)
 * @param target_y_mm Forward distance in mm
 */
void navigate_and_inspect_obstacle(int32_t target_x_mm, int32_t target_y_mm);

#endif /* OBSTACLE_NAV_H */
