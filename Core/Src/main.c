/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motors.h"
#include "encoders.h"
#include "servo.h"
#include "control.h"
#include "command.h"
#include "calib.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Set to 1 to run the open-loop motor/encoder sign check at boot instead of
   accepting commands. Do this once, with the wheels OFF THE GROUND, before
   you trust the closed loop. Set back to 0 afterwards. */
#define SELFTEST 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Latched at the very top of main() so you can tell a brown-out reset from a
   normal power-up after the motors have been running. Inspect in the debugger:
   bit 31 LPWRRSTF, 30 WWDGRSTF, 29 IWDGRSTF, 28 SFTRSTF, 27 PORRSTF,
   26 PINRSTF, 25 BORRSTF. BORRSTF or PORRSTF appearing after a motor stall
   means the supply is collapsing, not that the firmware is wrong. */
volatile uint32_t reset_flags;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if SELFTEST
static void selftest(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  reset_flags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;          /* clear so the next reset reads cleanly */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  motors_init();
  encoders_init();
  servo_init();
  control_init();
  command_init();
  HAL_TIM_Base_Start_IT(&htim6);       /* starts the 100 Hz control loop */

#if SELFTEST
  selftest();
#endif

  command_send("READY\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    command_poll();

    /* Heartbeat. If LED3 stops blinking the firmware has trapped -- most
       likely in Error_Handler(), which now blinks fast instead of dying
       silently, so the two are easy to tell apart. */
    static uint32_t last = 0;
    if (HAL_GetTick() - last >= 500u) {
        last = HAL_GetTick();
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM6 period-elapsed callback -- runs the 100 Hz control loop.
  *
  * This was the missing link: HAL_TIM_IRQHandler() calls this, and without
  * an override here it resolved to HAL's empty weak stub, so control_tick()
  * was never executed and encoders were never sampled.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    control_tick();
  }
}

#if SELFTEST
/**
  * @brief Open-loop check of motor direction and encoder sign.
  *
  * Run this with the wheels off the ground and a serial terminal on the
  * USB-C COM port at 115200 8N1. Every reported total should be POSITIVE.
  * If a total is negative, flip the matching INVERT_* in calib.h. If the
  * WRONG wheel's total moves, your TIM2/TIM3 encoder wiring is swapped.
  */
static void selftest(void)
{
  char buf[96];

  command_send("\r\nSELFTEST -- wheels OFF the ground\r\n");
  HAL_Delay(1500);

  /* ---- LEFT wheel, positive duty ---- */
  encoders_reset();
  for (int32_t d = 0; d <= 6000; d += 100) { motor_left(d); HAL_Delay(10); }
  HAL_Delay(700);
  motor_left(0);
  HAL_Delay(600);
  snprintf(buf, sizeof buf,
           "LEFT  +duty : enc_left=%ld  enc_right=%ld  (want left>0, right~0)\r\n",
           (long)enc_left_total, (long)enc_right_total);
  command_send(buf);
  HAL_Delay(800);

  /* ---- RIGHT wheel, positive duty ---- */
  encoders_reset();
  for (int32_t d = 0; d <= 6000; d += 100) { motor_right(d); HAL_Delay(10); }
  HAL_Delay(700);
  motor_right(0);
  HAL_Delay(600);
  snprintf(buf, sizeof buf,
           "RIGHT +duty : enc_left=%ld  enc_right=%ld  (want right>0, left~0)\r\n",
           (long)enc_left_total, (long)enc_right_total);
  command_send(buf);

  motors_coast();
  command_send("SELFTEST done -- set SELFTEST to 0 when the signs are right\r\n\r\n");
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* A silent while(1) here cost days of debugging: a failed clock init or a
     failed HAL_TIM_PWM_Start() looked identical to "the motors don't work".
     Kill the bridges first, then blink LED3 fast so a trap is unmistakable
     against the 1 Hz heartbeat in the main loop. */
  __disable_irq();

  /* Force the four bridge inputs low (coast) with direct register writes.
     Do NOT go via motors_coast() -- if this trap came from SystemClock_Config
     the timer handles are still zeroed and htim->Instance would be NULL. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIOB->MODER = (GPIOB->MODER & ~((3u << 16) | (3u << 18)))
                                | ((1u << 16) | (1u << 18));   /* PB8, PB9 out */
  GPIOB->BSRR  = (1u << (8 + 16)) | (1u << (9 + 16));          /* both low     */

  GPIOE->MODER = (GPIOE->MODER & ~((3u << 10) | (3u << 12)))
                                | ((1u << 10) | (1u << 12));   /* PE5, PE6 out */
  GPIOE->BSRR  = (1u << (5 + 16)) | (1u << (6 + 16));          /* both low     */

  GPIOE->MODER = (GPIOE->MODER & ~(3u << (8 * 2))) | (1u << (8 * 2));  /* LED3 */

  while (1)
  {
    GPIOE->ODR ^= (1u << 8);
    for (volatile uint32_t i = 0; i < 400000u; i++) { }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
