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
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble.h"
#if 0
#include "MAX30101.h"
#include "sensor_ringbuffer.h"
#endif
#include "LSM9DS1.h"
#include "usbd_cdc_if.h"
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
#if 0
/* 旧版本计数/发送缓存：保留用于回退对照，不删除 */
static volatile uint32_t g_tim1_isr_count = 0;
static volatile uint32_t g_master_cmp4_isr_count = 0;
static volatile uint32_t g_tima_out2_rst_isr_count = 0;

static uint32_t g_last_tim1_total = 0;
static uint32_t g_last_master_cmp4_total = 0;
static uint32_t g_last_tima_out2_rst_total = 0;
static uint32_t g_last_print_tick = 0;
static char g_ble_msg[128];
#endif

static uint32_t g_last_print_tick = 0;
static char g_usb_msg[192];
/* MAX30101 调试缓存：当前 MIMU 联调阶段暂不启用。 */
#if 0
static SensorRingBuffer_t g_ppg_rb;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 程序总体用途（当前调试版本）：
 * 1) 暂时关闭 MAX30101 PPG 调试链路，保留 BLE 与 HRTIM/TIM1 的注释风格。
 * 2) 每 1 秒执行三步 MIMU 调试并通过 USB CDC 输出结果：
 *    - A: 阻塞式 SPI 读取 WHO_AM_I（通信连通性）
 *    - B: 阻塞式 SPI 连续读取 6 轴原始数据（SPI+器件基础功能）
 *    - C: SPI+DMA 读取 6 轴原始数据（DMA链路功能）
 * 3) 保留旧代码块（#if 0 / 注释）用于快速回退和对照，不做删除。
 */

#if 0
/*
 * 直接 I2C 读取 1 组 PPG 样本。
 * 返回值约定：
 * - 1: 成功读到 1 组 G/R/IR 样本。
 * - 2: FIFO 当前无新样本（WR/RD 指针相等）。
 * - 0: 通信失败或参数无效。
 */
static uint8_t PPG_ReadOneSample_DirectI2C(uint32_t *green, uint32_t *red, uint32_t *ir,
                                           uint8_t *wr_ptr, uint8_t *rd_ptr)
{
  uint8_t wr = 0U;
  uint8_t rd = 0U;
  uint8_t fifo_data[MAX30101_SAMPLE_BYTES] = {0};
  uint8_t available = 0U;

  if ((green == NULL) || (red == NULL) || (ir == NULL) || (wr_ptr == NULL) || (rd_ptr == NULL))
  {
    return 0U;
  }

  if ((MAX30101_ReadReg(MAX30101_REG_FIFO_WR_PTR, &wr) == 0U) ||
      (MAX30101_ReadReg(MAX30101_REG_FIFO_RD_PTR, &rd) == 0U))
  {
    return 0U;
  }

  wr &= 0x1FU;
  rd &= 0x1FU;
  *wr_ptr = wr;
  *rd_ptr = rd;

  /* MAX30101 FIFO 深度为 32，按环形队列计算当前可读样本数。 */
  available = (uint8_t)((wr + MAX30101_FIFO_DEPTH - rd) % MAX30101_FIFO_DEPTH);
  if (available == 0U)
  {
    return 2U;
  }

  if (HAL_I2C_Mem_Read(&hi2c3,
                       MAX30101_I2C_ADDR,
                       MAX30101_REG_FIFO_DATA,
                       I2C_MEMADD_SIZE_8BIT,
                       fifo_data,
                       MAX30101_SAMPLE_BYTES,
                       50U) != HAL_OK)
  {
    return 0U;
  }

  *green = ((((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2]) >> 1) & 0x1FFFFU;
  *red = ((((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5]) >> 1) & 0x1FFFFU;
  *ir = ((((uint32_t)fifo_data[6] << 16) | ((uint32_t)fifo_data[7] << 8) | fifo_data[8]) >> 1) & 0x1FFFFU;

  return 1U;
}
#endif

static int16_t MIMU_AssembleInt16LE(uint8_t low_byte, uint8_t high_byte)
{
  /*
   * LSM9DS1 的 OUT_* 数据寄存器按“小端序”输出：
   * - 低字节在前 (L)
   * - 高字节在后 (H)
   * 此处按 low/high 组包为 16bit 有符号补码，供 gx/gy/gz/ax/ay/az 解析使用。
   */
  return (int16_t)(((uint16_t)high_byte << 8) | (uint16_t)low_byte);
}

static HAL_StatusTypeDef MIMU_ReadWhoAmI_Blocking(uint8_t *who_am_i)
{
  /*
   * WHO_AM_I 寄存器地址为 0x0F：
   * SPI 读命令格式为 [bit7=1 | addr(6:0)]，所以命令字节 = 0x8F。
   * 2 字节全双工含义：
   * - 第 1 字节发送命令，同时回收无效字节
   * - 第 2 字节发送 dummy(0x00) 以产生时钟，MISO 回来才是寄存器值
   */
  uint8_t tx_buf[2] = {0x8FU, 0x00U};
  uint8_t rx_buf[2] = {0};

  if (who_am_i == NULL)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
  /* CS 拉低后执行 SPI 事务，严格对应手册的片选时序要求。 */
  if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2U, 20U) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    return HAL_ERROR;
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

  /* WHO_AM_I 期望值为 0x68，用于判断通信链路是否打通。 */
  *who_am_i = rx_buf[1];
  return HAL_OK;
}

static HAL_StatusTypeDef MIMU_ReadBurst_Blocking(LSM9DS1_RawData_t *raw)
{
  /*
   * 连续读取命令 0x98 = 0x18 | 0x80：
   * - 起始地址 0x18 = OUT_X_L_G
   * - bit7=1 表示读
   * 结合 LSM9DS1_Init() 中 CTRL_REG8=0x44 (BDU=1, IF_ADD_INC=1)：
   * - IF_ADD_INC 允许地址自动递增
   * - BDU 防止高低字节撕裂
   */
  uint8_t tx_buf[13] = {0};
  uint8_t rx_buf[13] = {0};

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  tx_buf[0] = LSM9DS1_BURST_READ_CMD;
  /* tx_buf[1..12] 保持 0x00，仅用于提供 12 字节 SPI 时钟。 */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 13U, 20U) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    return HAL_ERROR;
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

  /*
   * rx_buf 对应关系：
   * - rx_buf[0]   : 命令阶段回传废位
   * - rx_buf[1:6] : Gyro X/Y/Z (0x18~0x1D)
   * - rx_buf[7:12]: Acc  X/Y/Z (设备自动跳转到 0x28~0x2D)
   */
  raw->gx = MIMU_AssembleInt16LE(rx_buf[1], rx_buf[2]);
  raw->gy = MIMU_AssembleInt16LE(rx_buf[3], rx_buf[4]);
  raw->gz = MIMU_AssembleInt16LE(rx_buf[5], rx_buf[6]);
  raw->ax = MIMU_AssembleInt16LE(rx_buf[7], rx_buf[8]);
  raw->ay = MIMU_AssembleInt16LE(rx_buf[9], rx_buf[10]);
  raw->az = MIMU_AssembleInt16LE(rx_buf[11], rx_buf[12]);

  return HAL_OK;
}

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
#if 0
  /* 旧版本初始化流程：保留用于回退对照，不删除 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();

  //1. 电源上电序列，为PPG.MIMU供电
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);  // E5V
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // E3.3V
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // E4V
  HAL_Delay(50); // 等待电源稳定

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

  // (void)snprintf(g_ble_msg, sizeof(g_ble_msg), "TIM1->HRTIM(M+TA) sync start\r\n");
  // (void)BLE_Transmit_Data_DMA((uint8_t *)g_ble_msg, (uint16_t)strlen(g_ble_msg));
#endif

  /*
   * 当前启用的初始化流程：
    * - 当前是 MIMU 调试模式：启用 GPIO/DMA/SPI1/USART/USB。
    * - I2C3/HRTIM/TIM1/BLE 保留注释，避免引入额外干扰。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  // MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  // MX_HRTIM1_Init();
  // MX_TIM1_Init();
  MX_USB_Device_Init();

  /*
   * PPG/MIMU 电源上电时序：
   * - 先开 E5V，再开 E3.3V/E4V，每步留 50ms 建立时间
   * - 避免传感器在电源未稳定时读写寄存器导致总线异常
   */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);  // E5V
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // E3.3V
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // E4V
  HAL_Delay(50); // 等待电源稳定

  // if (BLE_Init() != HAL_OK)
  // {
  //   Error_Handler();
  // }

#if 0
  /* 绑定 ring buffer 给 MAX30101 DMA 回调，用于主循环取样观测。 */
  RingBuffer_Init(&g_ppg_rb);
  MAX30101_AttachRingBuffer(&g_ppg_rb);

  /* 设备初始化：校验 PartID、软复位、寄存器默认配置。 */
  if (MAX30101_Init() == 0U)
  {
    Error_Handler();
  }

  /*
   * 重配置为调试目标模式：
   * - 三光路轮切：Green -> Red -> IR
   * - 8 次平均（降低噪声）
   * - SR=0x00（驱动接口对应最低采样档）
   * 主循环仍按 1Hz 输出观测结果，不等同于传感器内部 ODR。
   */
  if (MAX30101_SetLEDMode(MAX30101_LED_MODE_MULTI_G_R_IR,
                          MAX30101_DEFAULT_LED_GREEN_PA,
                          MAX30101_DEFAULT_LED_RED_PA,
                          MAX30101_DEFAULT_LED_IR_PA,
                          0x00U,
                          0x03U) == 0U)
  {
    Error_Handler();
  }
#endif

  /*
   * LSM9DS1_Init() 内部已完成与底层寄存器的关键对应：
   * 1) WHO_AM_I(0x0F) 读取并校验 0x68
   * 2) CTRL_REG8(0x22) 写 0x05 软复位，再写 0x44 开 BDU/地址自增
   * 3) CTRL_REG9/FIFO_CTRL/INT1_CTRL 关闭 FIFO 与中断
   * 4) CTRL_REG1_G/3_G/6_XL/7_XL 写入 119Hz + 目标量程/滤波配置
   */
  /*
   * 调用驱动初始化入口：
   * - 函数: LSM9DS1_Init()
   * - 作用: 完成 WHO_AM_I 校验、CTRL_REG8/9、FIFO_CTRL、INT1_CTRL、
   *         CTRL_REG1_G/3_G/6_XL/7_XL 一次性配置
   * - 实际意义: 上电后先把器件状态固定到“可预测的轮询采集模式”
   */
  if (LSM9DS1_Init() != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(2000);
  /* USB 启动提示：便于串口工具中确认程序已进入调试模式。 */
  (void)snprintf(g_usb_msg, sizeof(g_usb_msg),
                 "MIMU debug start: WHO_AM_I + SPI + SPI_DMA, report=1Hz\r\n");
  (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));

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
#if 0
      /* 旧版本统计与BLE/USB双发：保留用于回退对照，不删除 */
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
        /* USB CDC 非阻塞对比发送：BUSY 时直接返回，不影响主循环节拍。 */
        (void)CDC_Transmit_FS2((uint8_t *)g_ble_msg, (uint16_t)len);
      }
#endif

#if 0
      /* 旧版本 PPG 1Hz 双路径调试：完整保留，便于后续快速回滚。 */
      uint32_t green = 0U;
      uint32_t red = 0U;
      uint32_t ir = 0U;
      uint8_t wr_ptr = 0U;
      uint8_t rd_ptr = 0U;
      uint8_t direct_ret;
      HAL_StatusTypeDef ppg_dma_start_ret;
      const MAX30101_RuntimeState_t *ppg_rt;
      uint32_t ppg_dma_ok_before;
      uint32_t ppg_dma_err_before;
      uint32_t ppg_wait_begin;
      SensorDataFrame_t dma_frame = {0};
      uint8_t dma_got_sample = 0U;

      /* 统一 1Hz 节拍。 */
      g_last_print_tick = now;

      /* A) 直接 I2C 读取：先验证基础总线读写与FIFO数据格式是否正确。 */
      direct_ret = PPG_ReadOneSample_DirectI2C(&green, &red, &ir, &wr_ptr, &rd_ptr);
      if (direct_ret == 1U)
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[I2C ] OK wr=%u rd=%u G=%lu R=%lu IR=%lu\r\n",
                       (unsigned int)wr_ptr,
                       (unsigned int)rd_ptr,
                       (unsigned long)green,
                       (unsigned long)red,
                       (unsigned long)ir);
      }
      else if (direct_ret == 2U)
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[I2C ] EMPTY wr=%u rd=%u\r\n",
                       (unsigned int)wr_ptr,
                       (unsigned int)rd_ptr);
      }
      else
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[I2C ] FAIL reg/fifo read error\r\n");
      }
      (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));

      /*
       * B) I2C+DMA 读取：
       * - 触发异步状态机读取 WR/RD 指针并发起 FIFO DMA。
       * - 主循环短等待 busy 结束后，从 ring buffer 取样并输出。
       */
      ppg_rt = MAX30101_GetRuntimeState();
      ppg_dma_ok_before = ppg_rt->dma_read_ok_count;
      ppg_dma_err_before = ppg_rt->i2c_error_count;
      ppg_dma_start_ret = MAX30101_TriggerPointerRead_IT();

      if (ppg_dma_start_ret == HAL_OK)
      {
        ppg_wait_begin = HAL_GetTick();
        /* 限时等待避免主循环卡死；300ms 足够覆盖一次异步读取流程。 */
        while ((MAX30101_GetRuntimeState()->busy != 0U) && ((HAL_GetTick() - ppg_wait_begin) < 300U))
        {
        }

        if (RingBuffer_Pop(&g_ppg_rb, &dma_frame))
        {
          dma_got_sample = 1U;
        }

        ppg_rt = MAX30101_GetRuntimeState();
        if (dma_got_sample != 0U)
        {
          /* 成功弹出一帧 DMA 数据。 */
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[DMA ] OK G=%lu R=%lu IR=%lu dma_ok=%lu err=%lu\r\n",
                         (unsigned long)dma_frame.ppg_data[0],
                         (unsigned long)dma_frame.ppg_data[1],
                         (unsigned long)dma_frame.ppg_data[2],
                         (unsigned long)ppg_rt->dma_read_ok_count,
                         (unsigned long)ppg_rt->i2c_error_count);
        }
        else if ((ppg_rt->dma_read_ok_count > ppg_dma_ok_before) || (ppg_rt->i2c_error_count != ppg_dma_err_before))
        {
          /* DMA流程走完但未取到帧，输出状态计数辅助定位。 */
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[DMA ] DONE no sample pop, ptr_ok=%lu dma_ok=%lu empty=%lu err=%lu\r\n",
                         (unsigned long)ppg_rt->ptr_read_ok_count,
                         (unsigned long)ppg_rt->dma_read_ok_count,
                         (unsigned long)ppg_rt->skip_empty_count,
                         (unsigned long)ppg_rt->i2c_error_count);
        }
        else
        {
          /* 既无计数变化也无样本，判为超时。 */
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[DMA ] TIMEOUT busy=%u ptr_stage=%u err=%lu\r\n",
                         (unsigned int)ppg_rt->busy,
                         (unsigned int)ppg_rt->ptr_stage,
                         (unsigned long)ppg_rt->i2c_error_count);
        }
      }
      else
      {
        /* 触发入口返回 BUSY/FAIL，通常表示 I2C 仍忙或前次流程未释放。 */
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[DMA ] START BUSY/FAIL ret=%d i2c_state=%d\r\n",
                       (int)ppg_dma_start_ret,
                       (int)hi2c3.State);
      }

      (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
#endif

      uint8_t who = 0U;
      LSM9DS1_RawData_t direct_raw = {0};
      LSM9DS1_RawData_t dma_raw = {0};
      HAL_StatusTypeDef step_ret;
      HAL_StatusTypeDef dma_start_ret;
      const LSM9DS1_RuntimeState_t *rt;
      uint32_t dma_ok_before;
      uint32_t dma_err_before;
      uint32_t wait_begin;

      /* 统一 1Hz 节拍。 */
      g_last_print_tick = now;

      /*
       * A) WHO_AM_I 通信体检
       * 目标：只验证“命令字 + 片选时序 + SPI收发”是否正确。
       * 预期：读取值固定 0x68。
       * 若 FAIL/UNEXPECTED：优先排查 SPI 模式(Mode3)、CS 引脚、供电与连线。
       */
      step_ret = MIMU_ReadWhoAmI_Blocking(&who);
      if (step_ret == HAL_OK)
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[MIMU-A] WHO_AM_I=0x%02X %s\r\n",
                       (unsigned int)who,
                       (who == LSM9DS1_WHO_AM_I_VALUE) ? "OK" : "UNEXPECTED");
      }
      else
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[MIMU-A] WHO_AM_I read FAIL\r\n");
      }
      (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));

      /*
       * B) 阻塞式 SPI 连续读取 6 轴
       * 对应底层寄存器：
       * - 起始 0x18(OUT_X_L_G)，随后取 Gx/Gy/Gz
       * - 自动跨区到 0x28(OUT_X_L_XL)，再取 Ax/Ay/Az
       * 目标：确认“传感器本体在出数”，不仅仅是 WHO_AM_I 可读。
       */
      step_ret = MIMU_ReadBurst_Blocking(&direct_raw);
      if (step_ret == HAL_OK)
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[MIMU-B] SPI gx=%d gy=%d gz=%d ax=%d ay=%d az=%d\r\n",
                       (int)direct_raw.gx,
                       (int)direct_raw.gy,
                       (int)direct_raw.gz,
                       (int)direct_raw.ax,
                       (int)direct_raw.ay,
                       (int)direct_raw.az);
      }
      else
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[MIMU-B] SPI burst read FAIL\r\n");
      }
      (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));

      /*
       * C) SPI+DMA 读取
       * 调用 LSM9DS1_TriggerRead_IT() 发起非阻塞 DMA 事务：
       * - 成功完成后在 HAL_SPI_TxRxCpltCallback 中累计 dma_ok_count
       * - 出错则在 HAL_SPI_ErrorCallback 中累计 dma_error_count
       * 这里通过前后计数差判断本秒 DMA 是否真正跑通。
       */
      /*
       * 获取 DMA 运行态快照：
       * - dma_ok_count: 成功完成次数
       * - dma_error_count: 错误次数
       * 通过“前后差分”判断本秒触发是否真正执行到回调。
       */
      rt = LSM9DS1_GetRuntimeState();
      dma_ok_before = rt->dma_ok_count;
      dma_err_before = rt->dma_error_count;

      /*
       * 发起一次 SPI1+DMA 连续读：
       * - 函数: LSM9DS1_TriggerRead_IT()
       * - 底层: 发送 0x98 + 12 字节 dummy，回收 12 字节六轴原始值
       * - 若返回 HAL_BUSY: 说明 SPI 或驱动仍在忙态
       */
      dma_start_ret = LSM9DS1_TriggerRead_IT();

      if (dma_start_ret == HAL_OK)
      {
        wait_begin = HAL_GetTick();
        /*
         * 限时轮询 100ms：
         * - 若计数变化，说明 DMA 回调已触发
         * - 若超时无变化，说明 DMA 可能未起传/中断未进/总线阻塞
         */
        while ((HAL_GetTick() - wait_begin) < 100U)
        {
          rt = LSM9DS1_GetRuntimeState();
          if ((rt->dma_ok_count > dma_ok_before) || (rt->dma_error_count != dma_err_before))
          {
            break;
          }
        }

        rt = LSM9DS1_GetRuntimeState();
        if (rt->dma_ok_count > dma_ok_before)
        {
          /* DMA 成功后读取驱动内最新解包结果（与阻塞读格式一致）。 */
          /*
           * 读取驱动内部“最近一次 DMA 解包结果”：
           * - 数据来自 LSM9DS1_SPI_TxRxCpltHandler() -> LSM9DS1_ProcessDmaRxData()
           * - 格式顺序固定为 Gx/Gy/Gz/Ax/Ay/Az
           */
          LSM9DS1_GetLatestRaw(&dma_raw);
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[MIMU-C] DMA OK gx=%d gy=%d gz=%d ax=%d ay=%d az=%d ok=%lu err=%lu busy_drop=%lu\r\n",
                         (int)dma_raw.gx,
                         (int)dma_raw.gy,
                         (int)dma_raw.gz,
                         (int)dma_raw.ax,
                         (int)dma_raw.ay,
                         (int)dma_raw.az,
                         (unsigned long)rt->dma_ok_count,
                         (unsigned long)rt->dma_error_count,
                         (unsigned long)rt->trigger_busy_count);
        }
        else if (rt->dma_error_count != dma_err_before)
        {
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[MIMU-C] DMA ERROR ok=%lu err=%lu\r\n",
                         (unsigned long)rt->dma_ok_count,
                         (unsigned long)rt->dma_error_count);
        }
        else
        {
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[MIMU-C] DMA TIMEOUT ok=%lu err=%lu busy=%u\r\n",
                         (unsigned long)rt->dma_ok_count,
                         (unsigned long)rt->dma_error_count,
                         (unsigned int)rt->dma_busy);
        }
      }
      else
      {
        (void)snprintf(g_usb_msg,
                       sizeof(g_usb_msg),
                       "[MIMU-C] DMA START BUSY/FAIL ret=%d spi_state=%d\r\n",
                       (int)dma_start_ret,
                       (int)hspi1.State);
      }

      (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
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
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance == TIM1)
//   {
//     g_tim1_isr_count++;
//   }
// }

// void HAL_HRTIM_Compare4EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
// {
//   if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_MASTER))
//   {
//     g_master_cmp4_isr_count++;
//   }
// }

// void HAL_HRTIM_Output2ResetCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
// {
//   if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
//   {
//     g_tima_out2_rst_isr_count++;
//   }
// }

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
