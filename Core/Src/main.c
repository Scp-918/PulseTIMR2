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
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t g_tim1_isr_count = 0;
static volatile uint32_t g_master_cmp4_isr_count = 0;
static volatile uint32_t g_tima_out2_rst_isr_count = 0;

static uint32_t g_last_tim1_total = 0;
static uint32_t g_last_master_cmp4_total = 0;
static uint32_t g_last_tima_out2_rst_total = 0;
static uint32_t g_last_print_tick = 0;
static char g_ble_msg[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // MX_DMA_Init();
  // MX_HRTIM1_Init();
  // MX_I2C3_Init();
  // MX_SPI1_Init();
  // MX_SPI3_Init();
  // MX_USART1_UART_Init();
  // MX_USB_Device_Init();
  // MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();

  if (BLE_Init() != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(2000);

  // 启动 HRTIM master/timerA 计数器，等待 TIM1 TRGO(sync) 激活 master
  if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1,
                              HRTIM_MASTER_IT_MCMP4 |
                              HRTIM_MASTER_IT_MUPD |
                              HRTIM_MASTER_IT_MREP |
                              HRTIM_MASTER_IT_SYNC);
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1,
                             HRTIM_TIMERINDEX_TIMER_A,
                             HRTIM_TIM_IT_RST2);
  NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
  NVIC_ClearPendingIRQ(HRTIM1_TIMA_IRQn);

  __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1,
                               HRTIM_MASTER_IT_MCMP4);
  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,
                              HRTIM_TIMERINDEX_TIMER_A,
                              HRTIM_TIM_IT_RST2);
  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  __enable_irq();

  (void)snprintf(g_ble_msg, sizeof(g_ble_msg), "TIM1->HRTIM(M+TA) sync start\r\n");
  (void)BLE_Transmit_Data_DMA((uint8_t *)g_ble_msg, (uint16_t)strlen(g_ble_msg));

  g_last_print_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if ((now - g_last_print_tick) >= 1000U)
    {
      uint32_t total_tim1;
      uint32_t total_master_cmp4;
      uint32_t total_tima_out2_rst;

      __disable_irq();
      total_tim1 = g_tim1_isr_count;
      total_master_cmp4 = g_master_cmp4_isr_count;
      total_tima_out2_rst = g_tima_out2_rst_isr_count;
      __enable_irq();

      uint32_t per_sec_tim1 = total_tim1 - g_last_tim1_total;
      uint32_t per_sec_master_cmp4 = total_master_cmp4 - g_last_master_cmp4_total;
      uint32_t per_sec_tima_out2_rst = total_tima_out2_rst - g_last_tima_out2_rst_total;
      g_last_tim1_total = total_tim1;
      g_last_master_cmp4_total = total_master_cmp4;
      g_last_tima_out2_rst_total = total_tima_out2_rst;
      g_last_print_tick = now;

      int len = snprintf(g_ble_msg,
                         sizeof(g_ble_msg),
              "TIM1:%lu/s(%lu) MCMP4:%lu/s(%lu) TA_O2RST:%lu/s(%lu)\r\n",
                         (unsigned long)per_sec_tim1,
                         (unsigned long)total_tim1,
             (unsigned long)per_sec_master_cmp4,
             (unsigned long)total_master_cmp4,
              (unsigned long)per_sec_tima_out2_rst,
              (unsigned long)total_tima_out2_rst);
      if (len > 0)
      {
        (void)BLE_Transmit_Data_DMA((uint8_t *)g_ble_msg, (uint16_t)len);
      }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    g_tim1_isr_count++;
  }
}

void HAL_HRTIM_Compare4EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_MASTER))
  {
    g_master_cmp4_isr_count++;
  }
}

void HAL_HRTIM_Output2ResetCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    g_tima_out2_rst_isr_count++;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
