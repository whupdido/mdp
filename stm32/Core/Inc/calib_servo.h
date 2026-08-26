/*
 * calib_servo.h -- safe steering servo & 90-degree turn calibration
 */

#ifndef CALIB_SERVO_H
#define CALIB_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --- Hardware Safety Clamps ---
   HWZ020 servo and tie-rod linkage safety limits.
   Pulses outside this range will bind the mechanical linkage or strip gears. */
#define SERVO_SAFE_MIN_US       1050u   /* Absolute hardware floor (Full Left)  */
#define SERVO_SAFE_MAX_US       1950u   /* Absolute hardware ceiling (Full Right)*/
#define SERVO_CENTRE_SEARCH_MIN 1400u   /* Min search bound for straight centre */
#define SERVO_CENTRE_SEARCH_MAX 1600u   /* Max search bound for straight centre */

/* Results container for physical constants */
typedef struct {
    uint16_t centre_us;
    uint16_t left_steer_us;
    uint16_t right_steer_us;
    uint32_t turn_counts_fl;    /* Encoder counts for 90-deg Forward-Left  */
    uint32_t turn_counts_fr;    /* Encoder counts for 90-deg Forward-Right */
} ServoCalibResult_t;

/**
  * @brief Safe servo pulse writer that guarantees pulse width stays within
  *        the verified non-destructive mechanical range.
  */
void safe_servo_us(uint16_t us);

/**
  * @brief Runs full calibration sequence (Centre trim + 90° turn verification)
  *        and outputs ready-to-paste `#define` values to the serial console.
  */
void run_servo_autocalibration(void);

/**
  * @brief Iteratively drives straight segments and trims servo PWM until
  *        rear encoder delta (enc_right - enc_left) is minimised.
  * @retval Calibrated centre pulse width in microseconds (us).
  */
uint16_t auto_calib_servo_centre(void);

/**
  * @brief Measures encoder counts required to execute a 90-degree arc.
  * @param steer_pwm Steering servo angle to hold during the turn.
  * @param direction -1 for Left turn, +1 for Right turn.
  * @retval Average rear wheel encoder ticks traversed during the turn.
  */
uint32_t auto_calib_turn_90_deg(uint16_t steer_pwm, int8_t direction);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_SERVO_H */
