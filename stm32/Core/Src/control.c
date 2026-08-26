/*
 * control.c -- closed-loop motion controller with IMU heading & distance feedback
 */

#include "control.h"
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "calib_servo.h"
#include "calib.h"
#include "icm20948.h"
#include "flash_storage.h"
#include "command.h"
#include <math.h>
#include <stdlib.h>

/* --- Motion Mode State --- */
typedef enum {
    MODE_IDLE = 0,
    MODE_STRAIGHT,
    MODE_TURN_DEG,
    MODE_TURN_RAW,
	MODE_PIVOT_DEG
} control_mode_t;

static volatile control_mode_t current_mode = MODE_IDLE;
static volatile move_result_t  last_result  = MOVE_NONE;
static volatile uint8_t        busy_flag    = 0;
static float current_speed_ramp = 0.0f;
static float pivot_speed_ramp = 0.0f;
/* Track cumulative signed ticks during in-place pivots */
static volatile int32_t pivot_enc_l_accum = 0;
static volatile int32_t pivot_enc_r_accum = 0;

/* Motion Targets and Accumulators */
static volatile int32_t  target_counts_total = 0;
static volatile int32_t  accum_counts        = 0;
static volatile float    target_deg_total    = 0.0f;
static volatile float    accum_deg           = 0.0f;
static volatile int8_t   dir_forward         = 1;
static volatile int8_t   turn_left           = 0;
/* Relative encoder accumulators for straight driving */
static volatile int32_t enc_left_straight_accum  = 0;
static volatile int32_t enc_right_straight_accum = 0;

/* Active Straight-Line Heading Lock Reference */
static volatile float    locked_heading_deg  = 0.0f;
static volatile float    global_yaw_deg      = 0.0f;

/* Safety & Diagnostics */
static volatile uint32_t move_ticks          = 0;
static volatile uint32_t stall_ticks_count   = 0;

/* Speed PID States (tracks counts per 10ms tick) */
static float left_pid_integral  = 0.0f;
static float right_pid_integral = 0.0f;

/* PID Gains for speed control (MG513 motors) */
#define SPEED_KP    120.0f
#define SPEED_KI    15.0f

/* ------------------------------------------------------------------------- */
/* Helper Functions                                                          */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* Sensor Placeholder (Replace with actual IR/Ultrasonic reading later)   */
/* ------------------------------------------------------------------------- */
uint8_t check_front_collision(void)
{
    /* Example future code:
     * if (ultrasonic_distance_cm() < 12) return 1;
     */
    return 0; /* 0 = Path Clear */
}

static void reset_speed_pid(void)
{
    left_pid_integral  = 0.0f;
    right_pid_integral = 0.0f;
    current_speed_ramp = 0.0f;
}

static void stop_hardware(move_result_t result)
{
    motor_left(0);
    motor_right(0);
    safe_servo_us(SERVO_CENTRE);

    current_mode = MODE_IDLE;
    last_result  = result;
    busy_flag    = 0;
    reset_speed_pid();
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void control_init(void)
{
    current_mode = MODE_IDLE;
    last_result  = MOVE_NONE;
    busy_flag    = 0;
    global_yaw_deg = 0.0f;
    /* Ensure steering is centered while idle at startup */
	safe_servo_us(SERVO_CENTRE);
	motor_left(0);
	motor_right(0);
    //stop_hardware(MOVE_NONE);
}

uint8_t motion_busy(void)
{
    return busy_flag;
}

void motion_stop(void)
{
    stop_hardware(MOVE_ABORT);
}

move_result_t motion_result(void)
{
    return last_result;
}

/* ------------------------------------------------------------------------- */
/* High-Level Motion Commands (Blocking)                                    */
/* ------------------------------------------------------------------------- */

uint8_t move_straight_mm(int32_t mm)
{
    if (mm == 0) return 1;

    target_counts_total       = (int32_t)(fabsf((float)mm) / MM_PER_COUNT);
    dir_forward               = (mm > 0) ? 1 : -1;
    accum_counts              = 0;
    enc_left_straight_accum   = 0;
    enc_right_straight_accum  = 0;
    move_ticks                = 0;
    stall_ticks_count         = 0;

    /* Lock wheels to calibrated center at launch */
    safe_servo_us(SERVO_CENTRE);

    reset_speed_pid();
    busy_flag    = 1;
    current_mode = MODE_STRAIGHT;

    /* Monitor sensors while moving */
	while (busy_flag) {
		/* Only check for front collisions if we are driving forward */
		if (dir_forward == 1 && check_front_collision()) {
			stop_hardware(MOVE_DONE);
			busy_flag = 0;
			command_send("\r\n[WARN] COLLISION AVOIDED! Stopping early.\r\n");
			return 0; /* Return 0 = Aborted */
		}
		HAL_Delay(5);
	}
	return 1;
}

uint8_t move_turn_deg(int8_t left, int8_t forward, int32_t degrees)
{
    if (degrees <= 0) return 1;

    turn_left           = left;
    dir_forward         = forward ? 1 : -1;
    target_deg_total    = (float)degrees;
    accum_deg           = 0.0f;
    move_ticks          = 0;
    stall_ticks_count   = 0;

    /* Set Ackermann steering angle */
    if (left) {
        safe_servo_us(SERVO_LEFT);
    } else {
        safe_servo_us(SERVO_RIGHT);
    }
    HAL_Delay(250); /* Allow servo to reach mechanical position */

    reset_speed_pid();
    busy_flag    = 1;
    current_mode = MODE_TURN_DEG;

    /* Block until IMU confirms rotation complete */
    /* Monitor sensors while turning */
	while (busy_flag) {
		/* Only check for front collisions if driving FORWARD in the turn */
		if (dir_forward == 1 && check_front_collision()) {
			stop_hardware(MOVE_DONE);
			busy_flag = 0;
			command_send("\r\n[WARN] COLLISION AVOIDED MID-TURN! Stopping early.\r\n");
			return 0; /* Return 0 = Aborted */
		}
		HAL_Delay(5);
	}
	return 1;
}

void move_pivot_deg(int8_t left, int32_t degrees)
{
    if (degrees <= 0) return;

    turn_left           = left;
    dir_forward         = 1;
    target_deg_total    = (float)degrees;
    accum_deg           = 0.0f;
    move_ticks          = 0;
    stall_ticks_count   = 0;
    pivot_enc_l_accum   = 0;
    pivot_enc_r_accum   = 0;

    /* Command mechanical steering lock */
    if (left) {
        safe_servo_us(SERVO_LEFT); /* 1000 us */
    } else {
        safe_servo_us(2000);       /* Backed off from 2100 us to prevent binding */
    }
    HAL_Delay(220);

    reset_speed_pid();
    busy_flag    = 1;
    current_mode = MODE_PIVOT_DEG;

    while (busy_flag) {
        HAL_Delay(5);
    }
}

/**
 * @brief Executes a compact 90-degree 3-point turn.
 * @return 1 if successful, 0 if aborted due to obstacle.
 */
uint8_t move_kturn_90(int8_t left)
{
    uint8_t safe = 1;

    if (left)
    {
        /* 1. Forward-Left 45 degrees (Checks for obstacles) */
        safe = move_turn_deg(1, 1, 45);
        HAL_Delay(150);

        /* 2. Reverse-Right 45 degrees (Only if forward was safe) */
        if (safe) {
            move_turn_deg(0, 0, 45);
            HAL_Delay(150);
        }
    }
    else
    {
        /* 1. Forward-Right 45 degrees (Checks for obstacles) */
        safe = move_turn_deg(0, 1, 45);
        HAL_Delay(150);

        /* 2. Reverse-Left 45 degrees (Only if forward was safe) */
        if (safe) {
            move_turn_deg(1, 0, 45);
            HAL_Delay(150);
        }
    }

    /* Straighten wheels */
    safe_servo_us(SERVO_CENTRE);
    HAL_Delay(100);

    return safe;
}

void move_turn(int8_t left, int8_t forward, int32_t counts)
{
    if (counts <= 0) return;

    turn_left           = left;
    dir_forward         = forward ? 1 : -1;
    target_counts_total = counts;
    accum_counts        = 0;
    move_ticks          = 0;
    stall_ticks_count   = 0;

    if (left) {
		safe_servo_us(SERVO_LEFT);   /* 1000 us */
	} else {
		safe_servo_us(SERVO_RIGHT);
	}
    HAL_Delay(200);

    reset_speed_pid();
    busy_flag    = 1;
    current_mode = MODE_TURN_RAW;

    while (busy_flag) {
        HAL_Delay(5);
    }
}

/* ------------------------------------------------------------------------- */
/* 100 Hz Control Interrupt (TIM6 Callback)                                  */
/* ------------------------------------------------------------------------- */

void control_tick(void)
{
    const float dt = 0.01f; /* 10 ms period */

    /* 1. Sample IMU Gyro Z */
    float gz = icm20948_read_gyro_z(); /* in deg/sec */
//    if (fabsf(gz) < 0.25f) {  /* Ignore noise below 0.25 deg/sec */
//        gz = 0.0f;
//    }
    float delta_yaw = gz * dt;
    global_yaw_deg += delta_yaw;

    /* 2. Sample Encoders (ticks in this 10ms slice) */
    encoders_sample();
    int32_t left_delta  = enc_left_delta;
    int32_t right_delta = enc_right_delta;
    int32_t avg_delta   = (abs(left_delta) + abs(right_delta)) / 2;

    if (current_mode == MODE_IDLE) {
        return;
    }

    move_ticks++;

    /* --- Safety Checks (Stall & Timeout) --- */
    if (move_ticks > 25) { /* Grace period for initial motor startup */
        if (abs(left_delta) < STALL_MIN_COUNTS && abs(right_delta) < STALL_MIN_COUNTS) {
            stall_ticks_count++;
            if (stall_ticks_count >= STALL_TICKS) {
                stop_hardware(MOVE_STALL);
                return;
            }
        } else {
            stall_ticks_count = 0;
        }
    }

    if (move_ticks >= MOVE_TIMEOUT_TICKS) {
        stop_hardware(MOVE_TIMEOUT);
        return;
    }

    /* --- Closed-Loop Motion Modes --- */
    switch (current_mode)
    {
    	case MODE_STRAIGHT:
		{
			/* 1. Track cumulative ticks for target distance */
			accum_counts += avg_delta;

			if (accum_counts >= target_counts_total) {
				stop_hardware(MOVE_DONE);
				return;
			}

			/* --- Slew-Rate Acceleration Ramp --- */
			float target_speed = (float)(dir_forward * SPEED_STRAIGHT);
			const float RAMP_STEP = 1.2f; /* Accelerates by ~1.2 ticks per 10ms */

			if (current_speed_ramp < target_speed) {
				current_speed_ramp += RAMP_STEP;
				if (current_speed_ramp > target_speed) current_speed_ramp = target_speed;
			} else if (current_speed_ramp > target_speed) {
				current_speed_ramp -= RAMP_STEP;
				if (current_speed_ramp < target_speed) current_speed_ramp = target_speed;
			}

			/* 2. Accumulate individual wheel ticks for differential steering */
			enc_left_straight_accum  += abs(left_delta);
			enc_right_straight_accum += abs(right_delta);

			/* Position error (cumulative difference) and Velocity error (instantaneous rate) */
			int32_t pos_error  = enc_right_straight_accum - enc_left_straight_accum;
			int32_t rate_error = abs(right_delta) - abs(left_delta);

			/* Controller gains (Servo us per tick difference) */
			const float ENC_KP = 6.0f;   /* Proportional: restores straight track */
			const float ENC_KD = 0.0f;   /* Derivative: damps oscillation */
			const int16_t MAX_STEER_TRIM = 220; /* Increased to overcome linkage play */

			/* Calculate dynamic steering correction */
			int16_t steer_correction = (int16_t)((pos_error * ENC_KP) + (rate_error * ENC_KD));

			/* Clamp maximum steering authority */
			if (steer_correction > MAX_STEER_TRIM)  steer_correction = MAX_STEER_TRIM;
			if (steer_correction < -MAX_STEER_TRIM) steer_correction = -MAX_STEER_TRIM;

			/* Apply to servo (>1500 = Right, <1500 = Left) */
			uint16_t commanded_servo = (uint16_t)(SERVO_CENTRE + (dir_forward * steer_correction));
			safe_servo_us(commanded_servo);

			/* 3. Velocity PI Controller */
			target_speed = (float)(dir_forward * SPEED_STRAIGHT);
			float err_l = target_speed - (float)left_delta;
			float err_r = target_speed - (float)right_delta;

			left_pid_integral  += err_l * dt;
			right_pid_integral += err_r * dt;

			/* Anti-windup clamping */
			if (left_pid_integral > 250.0f)  left_pid_integral = 250.0f;
			if (left_pid_integral < -250.0f) left_pid_integral = -250.0f;
			if (right_pid_integral > 250.0f)  right_pid_integral = 250.0f;
			if (right_pid_integral < -250.0f) right_pid_integral = -250.0f;

			int32_t duty_l = (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
			int32_t duty_r = (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

			motor_left(duty_l);
			motor_right(duty_r);
			break;
		}

    	case MODE_TURN_DEG:
		{
			/* 1. True Ackermann Yaw Integration (handles forward AND reverse correctly) */
			float step_yaw = delta_yaw * (float)dir_forward;
			if (turn_left) {
				accum_deg += step_yaw;
			} else {
				accum_deg -= step_yaw;
			}

			/* 2. Angle Completion Check */
			const float BRAKING_LEAD_DEG = 2.5f; /* Compensates for chassis inertia */

			if (accum_deg >= (target_deg_total - BRAKING_LEAD_DEG)) {
			    stop_hardware(MOVE_DONE);
			    return;
			}

			/* 3. Differential Ackermann Wheel Speeds */
			float base_speed = (float)(dir_forward * SPEED_TURN);
			float target_l, target_r;

			if (turn_left) {
				target_l = base_speed * 0.70f;  /* Inner wheel slower */
				target_r = base_speed * 1.30f;  /* Outer wheel faster */
			} else {
				target_l = base_speed * 1.30f;  /* Outer wheel faster */
				target_r = base_speed * 0.70f;  /* Inner wheel slower */
			}

			float err_l = target_l - (float)left_delta;
			float err_r = target_r - (float)right_delta;

			left_pid_integral  += err_l * dt;
			right_pid_integral += err_r * dt;

			/* Anti-windup clamping */
			if (left_pid_integral > 250.0f)  left_pid_integral = 250.0f;
			if (left_pid_integral < -250.0f) left_pid_integral = -250.0f;
			if (right_pid_integral > 250.0f)  right_pid_integral = 250.0f;
			if (right_pid_integral < -250.0f) right_pid_integral = -250.0f;

			/* 4. Direction-Aware Feedforward */
			int32_t ff = (int32_t)dir_forward * 1200;

			int32_t duty_l = ff + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
			int32_t duty_r = ff + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

			motor_left(duty_l);
			motor_right(duty_r);
			break;
		}
    	case MODE_PIVOT_DEG:
		{
			/* 1. Integrate Absolute Gyro Yaw */
			if (turn_left) {
				accum_deg += delta_yaw;
			} else {
				accum_deg -= delta_yaw;
			}

			/* 2. Target Completion Check */
			if (accum_deg >= target_deg_total) {
				stop_hardware(MOVE_DONE);
				return;
			}

			/* 3. Symmetric Counter-Rotating Speed Targets */
			const float MAX_PIVOT_SPEED = (float)SPEED_TURN;
			const float RAMP_STEP = 1.5f; /* Accelerates by 1.5 ticks every 10ms */

			if (pivot_speed_ramp < MAX_PIVOT_SPEED) {
			    pivot_speed_ramp += RAMP_STEP;
			    if (pivot_speed_ramp > MAX_PIVOT_SPEED) pivot_speed_ramp = MAX_PIVOT_SPEED;
			}

			float target_l = turn_left ? -pivot_speed_ramp :  pivot_speed_ramp;
			float target_r = turn_left ?  pivot_speed_ramp : -pivot_speed_ramp;

			/* 4. PID Error Calculations */
			float err_l = target_l - (float)left_delta;
			float err_r = target_r - (float)right_delta;

			left_pid_integral  += err_l * dt;
			right_pid_integral += err_r * dt;

			/* Anti-windup clamping */
			if (left_pid_integral > 300.0f)  left_pid_integral = 300.0f;
			if (left_pid_integral < -300.0f) left_pid_integral = -300.0f;
			if (right_pid_integral > 300.0f)  right_pid_integral = 300.0f;
			if (right_pid_integral < -300.0f) right_pid_integral = -300.0f;

			/* 5. Anti-Creep (Forward Drift Prevention) */
			pivot_enc_l_accum += left_delta;
			pivot_enc_r_accum += right_delta;
			int32_t net_translation = pivot_enc_l_accum + pivot_enc_r_accum;

			int32_t creep_trim = 0;
			if (net_translation > 10) { /* If chassis creeps forward more than ~1.5mm */
				creep_trim = (net_translation - 10) * 12; /* Kp of 12 for drift correction */
				if (creep_trim > 600) creep_trim = 600;   /* Cap trim to prevent violent vibration */
			}

			/* 6. Symmetric Feedforward */
			/* 1500 is roughly 9% duty cycle, enough to break average stiction */
			int32_t ff_l = turn_left ? -3000 :  3000;
			int32_t ff_r = turn_left ?  3000 : -3000;

			/* 7. Motor Output */
			/* Subtracting creep_trim forces the robot backward if it drifts forward */
			int32_t duty_l = ff_l + (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral) - creep_trim;
			int32_t duty_r = ff_r + (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral) - creep_trim;

			motor_left(duty_l);
			motor_right(duty_r);
			break;
		}

        case MODE_TURN_RAW:
        {
            accum_counts += avg_delta;

            if (accum_counts >= target_counts_total) {
                stop_hardware(MOVE_DONE);
                return;
            }

            float target_speed = (float)(dir_forward * SPEED_TURN);
            float err_l = target_speed - (float)left_delta;
            float err_r = target_speed - (float)right_delta;

            left_pid_integral  += err_l * dt;
            right_pid_integral += err_r * dt;

            int32_t duty_l = (int32_t)(SPEED_KP * err_l + SPEED_KI * left_pid_integral);
            int32_t duty_r = (int32_t)(SPEED_KP * err_r + SPEED_KI * right_pid_integral);

            motor_left(duty_l);
            motor_right(duty_r);
            break;
        }

        case MODE_IDLE:
        default:
            break;
    }
}
