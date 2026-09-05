#include "main.h"
#include "ui.h"
#include "oled.h"
#include <stdio.h>

uint8_t ui_btn_down(void) {
    return (HAL_GPIO_ReadPin(BTN_USER_GPIO_Port, BTN_USER_Pin) == GPIO_PIN_RESET);
}

/*
 * A release only counts once the pin has read high continuously for
 * UI_RELEASE_MS. Exiting on the first high sample is what made every hold
 * register as a tap: a tactile switch chatters mid-press, and one stray high
 * sample was enough to end the press early.
 */
void ui_btn_wait_idle(void) {
    uint32_t last_low = HAL_GetTick();
    for (;;) {
        if (ui_btn_down()) {
            last_low = HAL_GetTick();
        } else if (HAL_GetTick() - last_low >= UI_RELEASE_MS) {
            return;
        }
        HAL_Delay(2);
    }
}

uint8_t ui_btn_confirm_press(void) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < UI_CONFIRM_MS) {
        if (!ui_btn_down()) return 0;
        HAL_Delay(2);
    }
    return 1;
}

uint32_t ui_btn_time_press(uint32_t t_press, uint8_t feedback_y) {
    uint32_t last_low = HAL_GetTick();
    uint8_t announced = 0;

    for (;;) {
        if (ui_btn_down()) {
            last_low = HAL_GetTick();
        } else if (HAL_GetTick() - last_low >= UI_RELEASE_MS) {
            break;
        }
        if (!announced && (HAL_GetTick() - t_press) >= UI_HOLD_MS) {
            announced = 1;
            if (feedback_y != 0xFFu) {
                OLED_ShowString(0, feedback_y, (const uint8_t *)"-- HOLD --     ");
                OLED_Refresh_Gram();
            }
        }
        HAL_Delay(2);
    }
    return last_low - t_press;
}

uint8_t ui_btn_wait(void) {
    ui_btn_wait_idle();
    for (;;) {
        if (ui_btn_down()) {
            uint32_t t0 = HAL_GetTick();
            if (ui_btn_confirm_press()) {
                return (ui_btn_time_press(t0, 0xFFu) >= UI_HOLD_MS) ? UI_BTN_HOLD
                                                                    : UI_BTN_TAP;
            }
        }
        HAL_Delay(2);
    }
}

uint8_t ui_gesture(const char *tap_label, const char *hold_label) {
    char b[24];

    if (!ui_btn_down()) return UI_BTN_NONE;

    uint32_t t0 = HAL_GetTick();
    if (!ui_btn_confirm_press()) return UI_BTN_NONE;

    OLED_Clear();
    OLED_ShowString(0, 0, (const uint8_t *)"SW1 held...");
    snprintf(b, sizeof b, "now: %s", tap_label);
    OLED_ShowString(0, 20, (const uint8_t *)b);
    snprintf(b, sizeof b, "hold: %s", hold_label);
    OLED_ShowString(0, 40, (const uint8_t *)b);
    OLED_Refresh_Gram();

    uint32_t last_low = HAL_GetTick();
    uint8_t announced = 0;
    uint32_t held;
    for (;;) {
        if (ui_btn_down()) {
            last_low = HAL_GetTick();
        } else if (HAL_GetTick() - last_low >= UI_RELEASE_MS) {
            held = last_low - t0;
            break;
        }
        if (!announced && (HAL_GetTick() - t0) >= UI_HOLD_MS) {
            announced = 1;
            snprintf(b, sizeof b, "now: %s", hold_label);
            OLED_ShowString(0, 20, (const uint8_t *)b);
            OLED_ShowString(0, 40, (const uint8_t *)"release to go  ");
            OLED_Refresh_Gram();
        }
        HAL_Delay(2);
    }
    return (held >= UI_HOLD_MS) ? UI_BTN_HOLD : UI_BTN_TAP;
}
