/*
 * ui.h -- SW1 (PE0) press handling: debounce, and telling a tap from a hold.
 *
 * Separate from turn_test.c only so the button logic can be tested and reused
 * on its own; there is nothing else in here.
 */

#ifndef UI_H
#define UI_H
#include <stdint.h>

#define UI_BTN_NONE   0u
#define UI_BTN_TAP    1u
#define UI_BTN_HOLD   2u

#define UI_HOLD_MS      700u   /* press this long or longer = a hold      */
#define UI_CONFIRM_MS    25u   /* continuous low before a press counts    */
#define UI_RELEASE_MS    60u   /* continuous high before a release counts */

/** @brief Raw debounced-free read: 1 while the button reads pressed. */
uint8_t  ui_btn_down(void);

/** @brief Block until the button has read released for UI_RELEASE_MS. */
void     ui_btn_wait_idle(void);

/** @brief Require UI_CONFIRM_MS of continuous low before a press counts. */
uint8_t  ui_btn_confirm_press(void);

/**
 * @brief Time a confirmed press until a confirmed release.
 * @param t_press     tick the press was first seen
 * @param feedback_y  OLED row to print "-- HOLD --" on when the threshold
 *                    passes, or 0xFF for no feedback
 * @return held duration in ms, measured to the last low sample
 */
uint32_t ui_btn_time_press(uint32_t t_press, uint8_t feedback_y);

/** @brief Block until a press and release. Returns UI_BTN_TAP or UI_BTN_HOLD. */
uint8_t  ui_btn_wait(void);

/**
 * @brief Read one gesture with on-screen feedback, for the main loop.
 *        Shows what letting go now will do and updates the moment the press
 *        becomes a hold. Returns UI_BTN_NONE if the press was only noise.
 */
uint8_t  ui_gesture(const char *tap_label, const char *hold_label);

#endif /* UI_H */
