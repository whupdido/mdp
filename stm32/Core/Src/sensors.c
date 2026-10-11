#include "main.h"
#include "sensors.h"
#include "calib.h"
#include "oled.h"
#include "command.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim8;

volatile float    ultrasonic_distance_cm = 0.0f;
volatile uint8_t image_found = 0;

/*
 * The DMA will automatically dump the 12-bit ADC values here.
 * Even index = Rank 1 (PA2 / Right IR), odd index = Rank 2 (PA3 / Left IR).
 * IR_RIGHT and IR_LEFT below ARE those offsets, so the buffer, the fit arrays
 * and the getters all use one numbering instead of swapping it per function.
 *
 * Kush (09-Oct): IR_OVERSAMPLE pairs instead of one. The ADC still free-runs
 * into it with the DMA interrupt masked (see ir_dma_start), so it costs no CPU;
 * a reading now averages the last IR_OVERSAMPLE conversions per sensor
 * (~150 us of signal) instead of trusting a single one.
 */
#define IR_OVERSAMPLE   16u
volatile uint16_t ir_adc_buffer[2u * IR_OVERSAMPLE];

#define IR_RIGHT   IR_SIDE_RIGHT
#define IR_LEFT    IR_SIDE_LEFT

/* Make sure you have access to your ADC handle (defined in adc.c) */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/**
 * @brief Start the ADC free-running into ir_adc_buffer, with the DMA interrupt
 *        kept masked.
 *
 * HAL_ADC_Start_DMA enables the DMA half- and full-transfer interrupts, and at
 * these conversion rates that is ruinous. Two channels free-run into a two
 * entry circular buffer, so at the stock 84 cycle sampling time a conversion
 * lands every ~4.6 us and the CPU takes a DMA interrupt that often, spending a
 * large slice of every second inside HAL_DMA_IRQHandler for nothing.
 *
 * At the 3 cycle sampling time the source test uses it stops being merely
 * wasteful: an interrupt is requested every ~0.7 us, which is faster than the
 * handler can run. The CPU never leaves the ISR, and since DMA2_Stream0 sits at
 * NVIC priority 0 while SysTick is at 15, SysTick stops being serviced, uwTick
 * freezes, and HAL_Delay never returns. That is precisely what left the source
 * test stuck on "sampling...".
 *
 * Nothing here uses the DMA transfer callbacks, the buffer is polled, so the
 * interrupt has no job. Masked, the DMA keeps refilling the buffer purely in
 * hardware with zero CPU cost, which also gives the rest of the firmware back
 * the cycles it was quietly losing.
 */
static uint8_t ir_dma_start(void) {
    uint8_t ok = (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ir_adc_buffer, 2u * IR_OVERSAMPLE) == HAL_OK);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    return ok;
}

/* Live model coefficients, one pair per sensor, seeded from calib.h.
   Fixed at the calib.h values; the on-car calibration UI that used to rewrite
   them at run time has been removed. */
static float ir_m[2] = { IR_RIGHT_M, IR_LEFT_M };
static float ir_b[2] = { IR_RIGHT_B, IR_LEFT_B };
static float ir_k[2] = { IR_RIGHT_K, IR_LEFT_K };

/* Set by ir_sensors_init(). A failed DMA start leaves the buffer at zero
   forever, which is indistinguishable from a dead sensor unless you check. */
static uint8_t ir_dma_started = 0;

static void us_capture_init(void);

/**
 * @brief Call this ONCE before your main loop starts to kick off the background DMA
 */
void ir_sensors_init(void) {
    ir_dma_started = ir_dma_start();
    us_capture_init();
}

/**
 * @brief Re-arm the ADC/DMA after a failed or aborted start.
 */
uint8_t ir_sensors_restart(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    ir_dma_started = ir_dma_start();
    return ir_dma_started;
}

/**
 * @brief Helper to convert raw 12-bit ADC (0-4095) to Voltage (0.0 - 3.3V)
 */
static float adc_to_voltage(uint16_t raw_adc) {
    return ((float)raw_adc / 4095.0f) * 3.3f;
}

/**
 * @brief Voltage on one IR channel: the mean of the IR_OVERSAMPLE samples the
 *        DMA holds for it.
 */
static float ir_volts(uint8_t ch) {
    uint32_t sum = 0u;
    for (uint32_t i = 0u; i < IR_OVERSAMPLE; i++) {
        sum += ir_adc_buffer[2u * i + ch];
    }
    return adc_to_voltage((uint16_t)(sum / IR_OVERSAMPLE));
}

/**
 * @brief The transfer function: d = m/(V - b), clamped to the trusted band.
 */
static float ir_volts_to_cm(uint8_t ch, float voltage) {
    float denom = voltage - ir_b[ch];

    /* Far away, or nothing plugged in: the model blows up here, so bail out
       to the far limit instead of returning a huge or negative distance. */
    if (denom < 0.01f) return IR_MAX_CM;

    float distance_cm = ir_m[ch] / denom - ir_k[ch];

    if (distance_cm > IR_MAX_CM) distance_cm = IR_MAX_CM;
    if (distance_cm < IR_MIN_CM) distance_cm = IR_MIN_CM;
    return distance_cm;
}

/* --- Public Getters for your Control Loop --- */

float get_sharp_ir_left_cm(void) {
    return ir_volts_to_cm(IR_LEFT, ir_volts(IR_LEFT));
}

float get_sharp_ir_right_cm(void) {
    return ir_volts_to_cm(IR_RIGHT, ir_volts(IR_RIGHT));
}

/* ------------------------------------------------------------------------- */
/* 100 Hz sensing (called from control_tick)                                  */
/* ------------------------------------------------------------------------- */

/* IR: a 5-tick median. The Sharp sensors only update every ~38 ms and throw
   the odd spike, so the median spans more than one of their updates and
   ignores up to two bad ticks out of five. Costs ~30 ms of lag to a step. */
#define IR_MEDIAN_N     5u
static float   ir_hist[2][IR_MEDIAN_N];
static uint8_t ir_hist_i    = 0u;
static uint8_t ir_hist_init = 0u;
static volatile float ir_filt[2] = { IR_MAX_CM, IR_MAX_CM };

/* Ultrasonic: one ping every US_PING_TICKS ticks, for the collision check and
   the parking guard. 60 ms, the HC-SR04 datasheet's minimum cycle (it was
   40 ms, and the sound of one ping can still be around for the next). The
   distances the approaches drive on come from ultrasonic_quiet_ping_cm().
   Kush (11-Oct, speed step 3): US_QUIET_MS 100 -> 60. 100 was the original's
   pause; 60 is the datasheet cycle the background pings already keep, and it
   takes 40 ms off each of the ~9-12 quiet pings in a Task 2 run. If the
   [T2] "obs1 us=" / "obs2 us=" readings start coming out longer than the
   tape says (a missed weak echo, see ultrasonic_quiet_ping_cm), put it back
   to 100. */
#define US_PING_TICKS   6u
#define US_QUIET_MS     60u       /* silence before a quiet ping (was 100)    */
#define US_ECHO_WAIT_MS 60u       /* the original's wait after its trigger    */
#define US_MIN_CM       2.0f      /* below this the HC-SR04 is guessing        */
#define US_MAX_CM       400.0f    /* past its range: a missing echo, not a wall */
#define US_HIST_N       3u
#define US_FRESH_MS     250u      /* readings older than this are not used     */
static uint8_t           us_div       = 0u;
static volatile uint8_t  us_wait_fall = 0u;
static volatile uint16_t us_rise      = 0u;
static volatile uint32_t us_count     = 0u;
static volatile uint32_t us_last_ms   = 0u;
static volatile float    us_hist[US_HIST_N];
static volatile uint32_t us_hist_ms[US_HIST_N];
static volatile uint8_t  us_hist_i    = 0u;
static volatile uint8_t  us_bg_on     = 1u;   /* background pings (0 during a quiet ping) */

static float median5(const float *a)
{
    float s[5] = { a[0], a[1], a[2], a[3], a[4] };
    for (uint8_t i = 1u; i < 5u; i++) {          /* insertion sort, 5 items */
        float v = s[i];
        int8_t j = (int8_t)(i - 1u);
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    return s[2];
}

void sensors_tick(void)
{
    /* --- IR --- */
    float now_cm[2];
    now_cm[IR_RIGHT] = ir_volts_to_cm(IR_RIGHT, ir_volts(IR_RIGHT));
    now_cm[IR_LEFT]  = ir_volts_to_cm(IR_LEFT,  ir_volts(IR_LEFT));

    if (!ir_hist_init) {                         /* first tick: fill the window */
        for (uint8_t i = 0u; i < IR_MEDIAN_N; i++) {
            ir_hist[IR_RIGHT][i] = now_cm[IR_RIGHT];
            ir_hist[IR_LEFT][i]  = now_cm[IR_LEFT];
        }
        ir_hist_init = 1u;
    }
    ir_hist[IR_RIGHT][ir_hist_i] = now_cm[IR_RIGHT];
    ir_hist[IR_LEFT][ir_hist_i]  = now_cm[IR_LEFT];
    ir_hist_i = (uint8_t)((ir_hist_i + 1u) % IR_MEDIAN_N);

    ir_filt[IR_RIGHT] = median5(ir_hist[IR_RIGHT]);
    ir_filt[IR_LEFT]  = median5(ir_hist[IR_LEFT]);

    /* --- Ultrasonic --- */
    if (us_bg_on && ++us_div >= US_PING_TICKS) {
        us_div = 0u;
        trigger_ultrasonic();
    }
}

float ir_filtered_cm(uint8_t side)
{
    return ir_filt[(side == IR_LEFT) ? IR_LEFT : IR_RIGHT];
}

/**
 * @brief Formats the voltages into strings for your OLED screen.
 */
void display_ir_voltages_oled(void) {
    float left_v   = ir_volts(IR_LEFT);
    float right_v  = ir_volts(IR_RIGHT);
    float left_d   = ir_volts_to_cm(IR_LEFT,  left_v);
    float right_d  = ir_volts_to_cm(IR_RIGHT, right_v);

    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];

    sprintf(line1, "Left : %.2fV", left_v);
    sprintf(line3, "Distance: %.2f", left_d);
    sprintf(line2, "Right: %.2fV", right_v);
    sprintf(line4, "Distance: %.2f", right_d);

    OLED_Clear();
    OLED_ShowString(0,  0, (const uint8_t *)line1);
    OLED_ShowString(0, 10, (const uint8_t *)line3);
    OLED_ShowString(0, 20, (const uint8_t *)line2);
    OLED_ShowString(0, 30, (const uint8_t *)line4);
    OLED_Refresh_Gram();
}

/* ------------------------------------------------------------------------- */
/* Ultrasonic                                                                 */
/* ------------------------------------------------------------------------- */

/* Busy-wait on the Cortex-M4 cycle counter (enabled in us_capture_init). */
static void us_delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* One-time setup, before TIM6 starts pinging: let HAL configure and start the
   capture channel and the 1 MHz counter once, then leave its interrupt off
   until the first ping. From here on the channel is driven at register level
   (trigger_ultrasonic / HAL_TIM_IC_CaptureCallback). The old code stopped and
   restarted it through HAL on every ping and on every edge, from two
   interrupt levels; if an echo edge landed inside one of those HAL calls the
   handle's channel state could be left BUSY, after which HAL_TIM_IC_Start_IT
   refuses and the ultrasonic goes quiet for good. */
static void us_capture_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    TIM8->DIER &= ~TIM_DIER_CC2IE;
    TIM8->SR    = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
}

/**
 * @brief Re-arms the capture for a rising edge and sends the TRIG pulse.
 *        Runs inside the 100 Hz TIM6 interrupt (sensors_tick).
 */
void trigger_ultrasonic(void)
{
    /* 1. Re-arm: rising edge, stale flags dropped. Whatever the last ping was
          doing (an echo that never came back, say) is abandoned here. */
    TIM8->DIER &= ~TIM_DIER_CC2IE;
    TIM8->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
    us_wait_fall = 0u;
    TIM8->SR    = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
    TIM8->DIER |= TIM_DIER_CC2IE;

    /* 2. Send the TRIG pulse (the HC-SR04 wants at least 10 us). */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    us_delay_us(12u);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
 * @brief Hardware Interrupt Callback. Runs automatically when ECHO changes state.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM8 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return;

    uint16_t now = (uint16_t)TIM8->CCR2;

    if (!us_wait_fall)
    {
        /* 1st interrupt: Rising Edge. Swap to falling and clear the flag the
           polarity change can raise (the old "phantom edge" fix). */
        us_rise      = now;
        us_wait_fall = 1u;
        TIM8->CCER  |= TIM_CCER_CC2P;
        TIM8->SR     = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
    }
    else
    {
        /* 2nd interrupt: Falling Edge. The counter wraps every 65.5 ms, so a
           16-bit difference is right for any echo the sensor can return. */
        uint16_t width = (uint16_t)(now - us_rise);
        TIM8->DIER  &= ~TIM_DIER_CC2IE;       /* quiet until the next ping */
        us_wait_fall = 0u;

        float cm = (float)width / 58.0f;
        if (cm >= US_MIN_CM && cm <= US_MAX_CM) {
            uint32_t t = HAL_GetTick();
            ultrasonic_distance_cm = cm;
            us_hist[us_hist_i]     = cm;
            us_hist_ms[us_hist_i]  = t;
            us_hist_i  = (uint8_t)((us_hist_i + 1u) % US_HIST_N);
            us_last_ms = t;
            us_count++;
        }
    }
}

uint32_t ultrasonic_seq(void)
{
    return us_count;
}

uint32_t ultrasonic_age_ms(void)
{
    if (us_count == 0u) return 0xFFFFFFFFu;
    return HAL_GetTick() - us_last_ms;
}

float ultrasonic_median_cm(void)
{
    float v[US_HIST_N];
    uint8_t n = 0u;
    const uint32_t now = HAL_GetTick();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t count = us_count;
    for (uint8_t i = 0u; i < US_HIST_N; i++) {
        if ((uint32_t)i < count && (now - us_hist_ms[i]) <= US_FRESH_MS) v[n++] = us_hist[i];
    }
    __set_PRIMASK(primask);

    if (n == 0u) return -1.0f;
    if (n == 1u) return v[0];
    /* Two: the nearer, not the average. A near echo and a far one averaged
       give a distance nothing is at, and every approach that reads this
       drives that far. The nearer one only ever stops the car short. */
    if (n == 2u) return (v[0] < v[1]) ? v[0] : v[1];
    /* median of three */
    float a = v[0], b = v[1], c = v[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}

float ultrasonic_read_cm(uint8_t new_readings, uint32_t timeout_ms)
{
    const uint32_t seq0 = us_count;
    const uint32_t t0   = HAL_GetTick();
    while ((uint32_t)(us_count - seq0) < new_readings && (HAL_GetTick() - t0) < timeout_ms) {
        command_poll();
        HAL_Delay(1);
    }
    return ultrasonic_median_cm();
}

/* Kush (09-Oct): one reading the way the original code took the reading each
   Task 2 approach drives on: no ping at all for US_QUIET_MS first (the car
   used to sit silent before it), then a single ping, and up to
   US_ECHO_WAIT_MS for its echo. The background pings pause meanwhile. A weak
   echo, like obstacle 1's, can be missed while the last ping's sound is still
   around, and the sensor then reports something further back. */
float ultrasonic_quiet_ping_cm(void)
{
    us_bg_on = 0u;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < US_QUIET_MS) {
        command_poll();
        HAL_Delay(1);
    }

    /* Background pings are off, so nothing else touches TIM8 or TRIG now. */
    const uint32_t seq0 = us_count;
    trigger_ultrasonic();
    t0 = HAL_GetTick();
    while (us_count == seq0 && (HAL_GetTick() - t0) < US_ECHO_WAIT_MS) {
        command_poll();
        HAL_Delay(1);
    }
    const float d = (us_count != seq0) ? ultrasonic_distance_cm : -1.0f;

    us_div   = 0u;
    us_bg_on = 1u;
    return d;
}

/**
 * @brief Checks if the forward path is blocked using the HC-SR04.
 * @return 1 if collision is imminent, 0 if path is clear.
 *
 * Kush (09-Oct): reads the background pings instead of firing its own. The
 * move loops call this every ~5 ms, and re-triggering that often reset the
 * capture before any echo from further than ~85 cm could come back, so the
 * reading went stale whenever nothing was close. Two readings in a row (so
 * one bad echo cannot stop a move), the newer one fresh.
 */
uint8_t check_front_collision(void)
{
    if (us_count < 2u) return 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t newest = (uint8_t)((us_hist_i + US_HIST_N - 1u) % US_HIST_N);
    const uint8_t prev   = (uint8_t)((us_hist_i + US_HIST_N - 2u) % US_HIST_N);
    const float    d1 = us_hist[newest], d0 = us_hist[prev];
    const uint32_t t1 = us_hist_ms[newest], t0 = us_hist_ms[prev];
    __set_PRIMASK(primask);

    const uint32_t now = HAL_GetTick();
    if ((now - t1) > 100u || (t1 - t0) > 100u) return 0;   /* not two fresh, consecutive pings */

    if (d1 < 5.0f && d0 < 5.0f) {
    	OLED_Clear();
    	char buf[32];
    	snprintf(buf, sizeof(buf), "Object %fcm away!", d1);
    	OLED_ShowString(0,0,(const uint8_t* ) buf);
    	OLED_Refresh_Gram();
        return 1; /* Collision imminent */
    }

    return 0; /* Path clear */
}

/**
 * @brief Infinite loop to test the HC-SR04 and display on the OLED.
 *        WARNING: This is a blocking loop. Call this in main() for testing only.
 */
void test_ultrasonic_oled(void)
{
    char buf[32];

    /* Setup the static part of the OLED screen */
    OLED_Clear();
    OLED_ShowString(0, 0, (const uint8_t *)"HC-SR04 Test");
    OLED_Refresh_Gram();

    while (1)
    {
        /* The 100 Hz tick pings it; wait for the next echo. */
        float d = ultrasonic_read_cm(1u, 200u);

        /* Format the distance string. Trailing spaces overwrite leftover
         * characters if the string length shrinks (e.g. 100.5 to 9.5). */
        if (d > 0.0f) snprintf(buf, sizeof(buf), "Dist: %.1f cm   ", d);
        else          snprintf(buf, sizeof(buf), "Dist: no echo   ");

        /* Display the live reading on the next line (Y = 20) */
        OLED_ShowString(0, 20, (const uint8_t *)buf);
        OLED_Refresh_Gram();
    }
}

/**
 * @brief Display both IR and Ultrasonic
 */
void display_both_sensors_oled(void) {
	char buf[32];
	OLED_Clear();

	/* The 100 Hz tick pings the HC-SR04; wait for a fresh echo or two. */
	float d = ultrasonic_read_cm(2u, 200u);

	if (d > 0.0f) snprintf(buf, sizeof(buf), "Dist: %.1f cm   ", d);
	else          snprintf(buf, sizeof(buf), "Dist: no echo   ");

	OLED_ShowString(0, 0, (const uint8_t *)buf);

    float left_v   = ir_volts(IR_LEFT);
    float right_v  = ir_volts(IR_RIGHT);
    float left_d   = ir_filtered_cm(IR_LEFT);
    float right_d  = ir_filtered_cm(IR_RIGHT);

    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];

    sprintf(line1, "Left : %.2fV", left_v);
    sprintf(line3, "Distance: %.2f", left_d);
    sprintf(line2, "Right: %.2fV", right_v);
    sprintf(line4, "Distance: %.2f", right_d);

    OLED_ShowString(0,  10, (const uint8_t *)line1);
    OLED_ShowString(0, 20, (const uint8_t *)line3);
    OLED_ShowString(0, 30, (const uint8_t *)line2);
    OLED_ShowString(0, 40, (const uint8_t *)line4);
    OLED_Refresh_Gram();
}

/**
 * @brief Checks if the forward path is blocked (Using dual IRs)
 */
//uint8_t check_front_collision(void)
//{
//    /* Read analog values and convert to CM (requires your ADC conversion code) */
//    float left_dist_cm = get_sharp_ir_left_cm();
//    float right_dist_cm = get_sharp_ir_right_cm();
//
//    /* If EITHER sensor sees an obstacle closer than 12 cm, abort! */
//    /* (We use 12cm so it stops before entering the 10cm IR blind spot) */
//    if (left_dist_cm < 12.0f || right_dist_cm < 12.0f) {
//        return 1;
//    }
//    return 0;
//}
