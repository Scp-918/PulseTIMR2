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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD4007.h"
#include "TMUX1108.h"
#include "ble_comm.h"
#include "ble.h"
/*
 * 传感器驱动与发送协议：
 * - MAX30101: PPG 采样（I2C + IT/DMA）
 * - LSM9DS1 : MIMU 采样（SPI + DMA）
 * - ble_comm: 将 SensorDataFrame_t 打包为固定帧格式
 */
#include "MAX30101.h"
#include "sensor_ringbuffer.h"
#include "LSM9DS1.h"
#if 0
/* 旧版 USB 通道头文件保留用于回滚，不删除。 */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_SEND_BATCH_THRESHOLD (10U)
#define CYCLE_RAW_DRAIN_LIMIT       (64U)
/*
 * PPG 发光电流初始化（MAX30101，单位为寄存器码值，约 0.2mA/LSB）：
 * - Green 提升以增强绿光通道信号幅度
 * - Red/IR 维持较低电流用于抑制饱和
 */
#define PPG_INIT_GREEN_PA (0x8FU)
#define PPG_INIT_RED_PA   (0x24U)
#define PPG_INIT_IR_PA    (0x24U)

#if 0
/* 旧参数保留用于快速回滚对照。 */
#define PPG_INIT_GREEN_PA_OLD (0x7FU)
#define PPG_INIT_RED_PA_OLD   (0x24U)
#define PPG_INIT_IR_PA_OLD    (0x24U)
#endif

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

/* 旧版 PPG DMA 调试缓存：保留用于回退对照，不删除。 */
static SensorRingBuffer_t g_ppg_rb;
#endif

static volatile uint32_t g_tim1_tick_count = 0U;
static volatile uint32_t g_master_cmp4_count = 0U;
static volatile uint8_t g_tim_group_phase = 1U;
static volatile uint8_t g_group_frame_ready_for_send = 0U;

static volatile uint8_t g_adc_dma_pending = 0U;
static volatile uint8_t g_adc_dma_pending_slot = 0U;
static volatile uint8_t g_adc_pulse_start_index = 0U;

static int64_t g_adc_early_sum = 0;
static int64_t g_adc_late_sum = 0;
static uint8_t g_adc_early_count = 0U;
static uint8_t g_adc_late_count = 0U;

static uint8_t g_adc_dma_raw[6][AD4007_FRAME_BYTES] = {{0}};
static SensorDataFrame_t g_group_frame = {0};
static SensorDataFrame_t g_ble_send_frame = {0};
static uint8_t g_ble_send_pending = 0U;

static uint32_t g_adc_dma_start_fail_count = 0U;
static uint32_t g_adc_dma_decode_fail_count = 0U;
static uint32_t g_adc_dma_timeout_count = 0U;
static uint32_t g_adc_busy_skip_count = 0U;
static uint32_t g_adc_fall_event_count = 0U;
static uint32_t g_adc_rst2_isr_count = 0U;
static uint32_t g_adc_rst2_to_dma_miss_count = 0U;
static uint32_t g_adc_cmp1_isr_count = 0U;
static uint32_t g_adc_cmp3_isr_count = 0U;
static uint32_t g_adc_rep_isr_count = 0U;
static uint32_t g_adc_start_ok_count = 0U;
static uint32_t g_adc_sample_ok_count = 0U;

/*
 * 当前生效数据通路：
 * ISR 生产者：MAX30101/LSM9DS1 DMA 回调 -> RingBuffer_Push()
 * 主循环消费者：RingBuffer_Pop() -> 融合到当前大周期帧 -> BLE_PackSingleFrame() -> BLE UART DMA
 */
static SensorRingBuffer_t g_sensor_rb;
/* 主协议帧缓存。 */
static uint8_t g_ble_frame[BLE_COMM_SINGLE_FRAME_SIZE];
#if 0
/* 调试文本输出状态缓存：保留用于回滚。 */
static uint32_t g_last_print_tick = 0U;
static uint32_t g_last_tim1_tick_count = 0U;
static uint32_t g_last_master_cmp4_count = 0U;
static uint32_t g_last_adc_rst2_isr_count = 0U;
static uint32_t g_last_adc_start_ok_count = 0U;
static uint32_t g_last_adc_sample_ok_count = 0U;
static uint32_t g_last_adc_rst2_to_dma_miss_count = 0U;
static uint32_t g_last_adc_cmp1_isr_count = 0U;
static uint32_t g_last_adc_cmp3_isr_count = 0U;
static uint32_t g_last_adc_rep_isr_count = 0U;
static char g_ble_dbg_msg[256];
#endif
#if 0
/* 旧版文本调试缓存：保留用于回滚。 */
static char g_ble_msg[128];
#endif

/*
 * 旧版“融合后再发”缓存：
 * - 保留用于快速回滚对照；当前策略改为“每大周期固定发送1帧”，因此不再启用。
 */
#if 0
static SensorDataFrame_t g_fused_frame = {0};
static uint8_t g_fused_has_ppg = 0U;
static uint8_t g_fused_has_imu = 0U;
#endif
static uint32_t g_group_total_count = 0U;
static uint32_t g_group_miss_ppg_count = 0U;
static uint32_t g_group_miss_imu_count = 0U;

#if 0
/* 旧版一帧延迟发送管线：保留用于回滚对照。 */
static SensorDataFrame_t g_prev_cycle_frame = {0};
static SensorDataFrame_t g_pending_cycle_frame = {0};
static uint8_t g_prev_cycle_valid = 0U;
static uint8_t g_pending_cycle_valid = 0U;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Bridge_ApplyState(uint8_t phase)
{
  switch (phase)
  {
    case 1U:
      TMUX_KB_SetChannel(TMUX_CH_S5);
      TMUX_KH_SetChannel(TMUX_CH_S7);
      TMUX_KL_SetChannel(TMUX_CH_S1);
      break;

    case 2U:
      TMUX_KB_SetChannel(TMUX_CH_S6);
      TMUX_KH_SetChannel(TMUX_CH_S6);
      TMUX_KL_SetChannel(TMUX_CH_S5);
      break;

    case 3U:
      TMUX_KB_SetChannel(TMUX_CH_S7);
      TMUX_KH_SetChannel(TMUX_CH_S5);
      TMUX_KL_SetChannel(TMUX_CH_S6);
      break;

    case 4U:
    default:
      TMUX_KB_SetChannel(TMUX_CH_S8);
      TMUX_KH_SetChannel(TMUX_CH_S1);
      TMUX_KL_SetChannel(TMUX_CH_S7);
      break;
  }
}

static void ADC_ResetCycleAccumulator(void)
{
  g_adc_early_sum = 0;
  g_adc_late_sum = 0;
  g_adc_early_count = 0U;
  g_adc_late_count = 0U;
  g_adc_pulse_start_index = 0U;
  g_adc_dma_pending = 0U;
  g_adc_dma_pending_slot = 0U;
}

static void ADC_TryHarvestPendingSample(void)
{
  int32_t code = 0;
  uint8_t slot;

  if (g_adc_dma_pending == 0U)
  {
    return;
  }

  if (hspi3.State != HAL_SPI_STATE_READY)
  {
    return;
  }

  slot = g_adc_dma_pending_slot;
  if (AD4007_ProcessRawData(g_adc_dma_raw[slot], 1U, &code) != HAL_OK)
  {
    g_adc_dma_decode_fail_count++;
    g_adc_dma_pending = 0U;
    return;
  }

  if (slot < 3U)
  {
    g_adc_early_sum += (int64_t)code;
    g_adc_early_count++;
  }
  else
  {
    g_adc_late_sum += (int64_t)code;
    g_adc_late_count++;
  }

  g_adc_sample_ok_count++;
  g_adc_dma_pending = 0U;
}

static void ADC_FinalizeStateAndRotateBridge(void)
{
  uint8_t state_index = (uint8_t)(g_tim_group_phase - 1U);
  int32_t early_avg = 0;
  int32_t late_avg = 0;

  ADC_TryHarvestPendingSample();
  if (g_adc_dma_pending != 0U)
  {
    (void)HAL_SPI_Abort(&hspi3);
    g_adc_dma_timeout_count++;
    g_adc_dma_pending = 0U;
  }

  if (g_adc_early_count != 0U)
  {
    early_avg = (int32_t)(g_adc_early_sum / (int64_t)g_adc_early_count);
  }
  if (g_adc_late_count != 0U)
  {
    late_avg = (int32_t)(g_adc_late_sum / (int64_t)g_adc_late_count);
  }

  g_group_frame.adc_data[state_index].early_code = early_avg;
  g_group_frame.adc_data[state_index].late_code = late_avg;

  g_tim_group_phase++;
  if (g_tim_group_phase > 4U)
  {
    g_tim_group_phase = 1U;
  }

  Bridge_ApplyState(g_tim_group_phase);
  ADC_ResetCycleAccumulator();
}

static uint8_t FrameHasPPGPayload(const SensorDataFrame_t *frame)
{
  if ((frame->ppg_data[0] != 0U) || (frame->ppg_data[1] != 0U) || (frame->ppg_data[2] != 0U))
  {
    return 1U;
  }
  return 0U;
}

static uint8_t FrameHasIMUPayload(const SensorDataFrame_t *frame)
{
  if ((frame->imu_data[0] != 0) || (frame->imu_data[1] != 0) || (frame->imu_data[2] != 0) ||
      (frame->imu_data[3] != 0) || (frame->imu_data[4] != 0) || (frame->imu_data[5] != 0))
  {
    return 1U;
  }
  return 0U;
}

static void Sensor_IngestPartialFramesForCurrentGroup(void)
{
  SensorDataFrame_t frame = {0};
  uint16_t count_snapshot;
  uint16_t i;
  uint8_t got_ppg = 0U;
  uint8_t got_imu = 0U;

  count_snapshot = RingBuffer_GetCount(&g_sensor_rb);
  if (count_snapshot == 0U)
  {
    g_group_miss_ppg_count++;
    g_group_miss_imu_count++;
    return;
  }

  for (i = 0U; i < count_snapshot; i++)
  {
    if (!RingBuffer_Pop(&g_sensor_rb, &frame))
    {
      break;
    }

    if (FrameHasPPGPayload(&frame) != 0U)
    {
      g_group_frame.ppg_data[0] = frame.ppg_data[0];
      g_group_frame.ppg_data[1] = frame.ppg_data[1];
      g_group_frame.ppg_data[2] = frame.ppg_data[2];
      got_ppg = 1U;
    }

    if (FrameHasIMUPayload(&frame) != 0U)
    {
      g_group_frame.imu_data[0] = frame.imu_data[0];
      g_group_frame.imu_data[1] = frame.imu_data[1];
      g_group_frame.imu_data[2] = frame.imu_data[2];
      g_group_frame.imu_data[3] = frame.imu_data[3];
      g_group_frame.imu_data[4] = frame.imu_data[4];
      g_group_frame.imu_data[5] = frame.imu_data[5];
      got_imu = 1U;
    }
  }

  if (got_ppg == 0U)
  {
    g_group_miss_ppg_count++;
  }
  if (got_imu == 0U)
  {
    g_group_miss_imu_count++;
  }
}

static void PrepareAndCommitGroupFrame(void)
{
  uint8_t committed = 0U;

  Sensor_IngestPartialFramesForCurrentGroup();

#if 0
  /*
   * 旧联调策略（保留用于回滚）：
   * - 发送帧中 ADC 字段强制置 0，仅验证 PPG/MIMU/BLE 通路。
   */
  (void)memset(g_group_frame.adc_data, 0, sizeof(g_group_frame.adc_data));
#endif

  __disable_irq();
  if (g_ble_send_pending == 0U)
  {
    g_ble_send_frame = g_group_frame;
    g_ble_send_pending = 1U;
    committed = 1U;
  }
  __enable_irq();

  if (committed != 0U)
  {
    (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
    g_group_total_count++;
    g_group_frame_ready_for_send = 0U;
  }
}

/*
 * 程序总体用途（当前调试版本）：
 * 1) 启用 TIM1(400Hz) -> HRTIM 同步的 4 状态机。
 * 2) 在每次状态周期中由 TimerA Output2 产生 3+3 个 CNV 脉冲，并在 reset 边沿触发 AD4007 SPI+DMA 读取。
 * 3) 状态机1/2在 Master CMP4 分别触发 PPG/MIMU DMA 读取并写入 ringbuffer。
 * 4) 状态机4完成后融合为单帧，通过 BLE 协议格式发送（包含 4 组 ADC early/late）。
 * 5) 旧版 PPG/MIMU/BLE 调试实现保留在 #if 0 中便于回滚。
 */

#if 0
/*
 * 旧版 AD4007 调试辅助：保留用于回滚对照。
 */
static float AD4007_CodeToVoltage(int32_t code)
{
  return ((float)code * AD4007_VREF) / (float)AD4007_SIGN_BIT_18BIT;
}
#endif

#if 0
/* 判断当前出队帧是否包含有效 PPG 数据。 */
static uint8_t FrameHasPPGPayload(const SensorDataFrame_t *frame)
{
  if ((frame->ppg_data[0] != 0U) || (frame->ppg_data[1] != 0U) || (frame->ppg_data[2] != 0U))
  {
    return 1U;
  }
  return 0U;
}

/* 判断当前出队帧是否包含有效 MIMU 数据。 */
static uint8_t FrameHasIMUPayload(const SensorDataFrame_t *frame)
{
  if ((frame->imu_data[0] != 0) || (frame->imu_data[1] != 0) || (frame->imu_data[2] != 0) ||
      (frame->imu_data[3] != 0) || (frame->imu_data[4] != 0) || (frame->imu_data[5] != 0))
  {
    return 1U;
  }
  return 0U;
}

/*
 * 若待发送帧 PPG 全0，则尝试用“上下两帧”的 PPG 做均值修复：
 * - 上一帧：prev_frame
 * - 下一帧：next_frame（当前最新采到但尚未发送的帧）
 * 仅当上下两帧都含有效 PPG 时执行替换。
 */
static void PatchZeroPPGWithNeighborAverage(SensorDataFrame_t *out_frame,
                                            const SensorDataFrame_t *prev_frame,
                                            const SensorDataFrame_t *next_frame)
{
  uint32_t sum;

  if ((out_frame == NULL) || (prev_frame == NULL) || (next_frame == NULL))
  {
    return;
  }

  if (FrameHasPPGPayload(out_frame) != 0U)
  {
    return;
  }

  if ((FrameHasPPGPayload(prev_frame) == 0U) || (FrameHasPPGPayload(next_frame) == 0U))
  {
    return;
  }

  sum = prev_frame->ppg_data[0] + next_frame->ppg_data[0];
  out_frame->ppg_data[0] = (sum >> 1);
  sum = prev_frame->ppg_data[1] + next_frame->ppg_data[1];
  out_frame->ppg_data[1] = (sum >> 1);
  sum = prev_frame->ppg_data[2] + next_frame->ppg_data[2];
  out_frame->ppg_data[2] = (sum >> 1);
}
#endif

#if 0
/*
 * 旧版“发送前补读一次DMA”逻辑：保留用于回滚对照。
 */
static void RefillPPGOnceBeforeSendIfNeeded(SensorDataFrame_t *cycle_frame,
                                            uint8_t *has_ppg,
                                            uint8_t *has_imu)
{
}
#endif

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

#if 0
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
#endif

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
  /*
   * 下方为 CubeMX 默认生成的外设初始化调用清单。
   * 真实启用顺序在 USER CODE BEGIN 2 内按当前调试架构手动安排。
   */
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

  /*
   * 启动 TimerA 的波形输出门控：
   * - TA1: 电桥激励门控输出
   * - TA2: AD4007 CNV 脉冲输出
   * 仅启动计数器不足以把波形真正送到引脚，需显式打开输出门控。
   */
  if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) != HAL_OK)
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

#if 0
  /*
   * 旧版 AD4007 调试初始化流程：保留用于回滚。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USB_Device_Init();
  if (AD4007_Init() != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(2000);
  /* USB 启动提示：便于串口工具中确认程序已进入调试模式。 */
  (void)snprintf(g_usb_msg, sizeof(g_usb_msg),
                 "AD4007 debug start: A=SW_CNV+SPI, C=SW_CNV+SPI_DMA, report=1Hz\r\n");
  (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
#endif

  #if 0
  /*
  * 当前启用的旧流程（PPG/MIMU/BLE 调试路径）：保留用于回滚，不删除。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* BLE 串口链路初始化（复位、波特率同步、进入目标速率）。 */
  if (BLE_Init() != HAL_OK)
  {
    Error_Handler();
  }

  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();

  /* 传感器电源上电顺序：E5V -> E3.3V -> E4V。启动MIMU/PPG/ADC供电 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(50);

  RingBuffer_Init(&g_sensor_rb);
  MAX30101_AttachRingBuffer(&g_sensor_rb);
  LSM9DS1_AttachRingBuffer(&g_sensor_rb);

  /* MAX30101 基础初始化：PartID/软复位/FIFO/LED 模式。 */
  if (MAX30101_Init() == 0U)
  {
    Error_Handler();
  }

  /*
   * MAX30101 运行模式：Green/Red/IR 三光路轮切 + 平均配置。
   * 当前验证目标：
   * - 保持三路轮切（G/R/IR）
   * - 100Hz 采样率
   * - 4 倍过采样
   * 参数换算：SR=400sps(code=0x03), SMP_AVE=4(code=0x02)，等效 ODR≈400/4=100Hz。
   */
  if (MAX30101_SetLEDMode(MAX30101_LED_MODE_MULTI_G_R_IR,
                          PPG_INIT_GREEN_PA,
                          PPG_INIT_RED_PA,
                          PPG_INIT_IR_PA,
                          0x03U,
                          0x02U) == 0U)
  {
    Error_Handler();
  }

#if 0
  /* 旧参数保留用于回滚：SR=3200/AVE=4，等效约 800Hz。 */
  if (MAX30101_SetLEDMode(MAX30101_LED_MODE_MULTI_G_R_IR,
                          PPG_INIT_GREEN_PA,
                          PPG_INIT_RED_PA,
                          PPG_INIT_IR_PA,
                          0x07U,
                          0x02U) == 0U)
  {
    Error_Handler();
  }
#endif

  /* LSM9DS1 基础初始化：WHO_AM_I + 工作寄存器配置。 */
  if (LSM9DS1_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 HRTIM 主定时器与 TimerA，等待 TIM1_TRGO 同步触发。 */
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
                             HRTIM_TIM_IT_CMP1 |
                             HRTIM_TIM_IT_CMP3 |
                             HRTIM_TIM_IT_REP);

  /* 清 pending，避免上电残留中断状态导致首拍异常。 */
  NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
  NVIC_ClearPendingIRQ(HRTIM1_TIMA_IRQn);

  /* 仅打开本轮调试必需中断：Master CMP4 与 TimerA RST2。 */
  __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MCMP4);
  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,
                              HRTIM_TIMERINDEX_TIMER_A,
                              HRTIM_TIM_IT_CMP1 |
                              HRTIM_TIM_IT_CMP3 |
                              HRTIM_TIM_IT_REP  |
                              HRTIM_TIM_IT_RST2);

  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动提示：通过 BLE 链路确认固件已进入新调试架构。 */
  (void)snprintf(g_usb_msg, sizeof(g_usb_msg),
                 "TIM1+HRTIM start: phase1=PPG, phase2=MIMU, phase3=BLE\r\n");
  (void)BLE_Transmit_Data_DMA((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
  #endif

  #if 0
  /*
   * 旧版“仅 ADC + USB 发送”流程：保留用于回滚，不删除。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();

  TMUX_Global_Init();
  g_tim_group_phase = 1U;
  Bridge_ApplyState(g_tim_group_phase);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(50);

  if (AD4007_Init() != HAL_OK)
  {
    Error_Handler();
  }

  ADC_ResetCycleAccumulator();
  (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
  (void)memset(&g_ble_send_frame, 0, sizeof(g_ble_send_frame));

  if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) != HAL_OK)
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
                             HRTIM_TIM_IT_CMP1 |
                             HRTIM_TIM_IT_CMP3 |
                             HRTIM_TIM_IT_REP);

  NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
  NVIC_ClearPendingIRQ(HRTIM1_TIMA_IRQn);

  __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MCMP4);
  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,
                              HRTIM_TIMERINDEX_TIMER_A,
                              HRTIM_TIM_IT_CMP1 |
                              HRTIM_TIM_IT_CMP3 |
                              HRTIM_TIM_IT_REP);

  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  #endif

  /*
   * 当前生效流程（ADC Timer success + PPG/MIMU + BLE-Frame）：
   * 1) 初始化 GPIO/DMA/I2C3/SPI1/SPI3/UART1/HRTIM/TIM1；
   * 2) BLE 初始化后，初始化 PPG/IMU 并绑定同一 ringbuffer；
   * 3) TIM1(400Hz) 驱动 4 状态机：phase1 触发PPG，phase2触发MIMU，phase4融合并发送；
    * 4) AD4007 按 3+3 脉冲运行，4 个状态的 early/late 结果合并进同一发送帧。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();

  TMUX_Global_Init();
  RingBuffer_Init(&g_sensor_rb);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(50);

  if (BLE_Init() != HAL_OK)
  {
    Error_Handler();
  }

  MAX30101_AttachRingBuffer(&g_sensor_rb);
  LSM9DS1_AttachRingBuffer(&g_sensor_rb);

  if (MAX30101_Init() == 0U)
  {
    Error_Handler();
  }

  /* 采用三光路轮切 + 4次平均 + 100Hz 等效输出。 */
  if (MAX30101_SetLEDMode(MAX30101_LED_MODE_MULTI_G_R_IR,
                          PPG_INIT_GREEN_PA,
                          PPG_INIT_RED_PA,
                          PPG_INIT_IR_PA,
                          0x03U,
                          0x02U) == 0U)
  {
    Error_Handler();
  }

  if (LSM9DS1_Init() != HAL_OK)
  {
    Error_Handler();
  }

  if (AD4007_Init() != HAL_OK)
  {
    Error_Handler();
  }

  g_tim_group_phase = 1U;
  Bridge_ApplyState(g_tim_group_phase);
  ADC_ResetCycleAccumulator();
  (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
  (void)memset(&g_ble_send_frame, 0, sizeof(g_ble_send_frame));

  if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) != HAL_OK)
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
                             HRTIM_TIM_IT_CMP1 |
                             HRTIM_TIM_IT_CMP3 |
                             HRTIM_TIM_IT_REP |
                             HRTIM_TIM_IT_RST2);

  NVIC_ClearPendingIRQ(HRTIM1_Master_IRQn);
  NVIC_ClearPendingIRQ(HRTIM1_TIMA_IRQn);

  __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MCMP4);
  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,
                              HRTIM_TIMERINDEX_TIMER_A,
                              HRTIM_TIM_IT_CMP1 |
                             HRTIM_TIM_IT_CMP3 |
                             HRTIM_TIM_IT_REP |
                             HRTIM_TIM_IT_RST2);

  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

#if 0
  g_last_print_tick = HAL_GetTick();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  #if 0
    uint32_t now = HAL_GetTick();
  #endif
    ADC_TryHarvestPendingSample();

#if 0
    if ((now - g_last_print_tick) >= 1000U)
    {
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

      /* 旧版本 PPG 1Hz 双路径调试：完整保留，便于后续快速回滚。 */
      {
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
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[DMA ] START BUSY/FAIL ret=%d i2c_state=%d\r\n",
                         (int)ppg_dma_start_ret,
                         (int)hi2c3.State);
        }

        (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
      }

      /* 旧版 MIMU 每秒调试流程：保留用于回退对照，不删除。 */
      {
        uint8_t who = 0U;
        LSM9DS1_RawData_t direct_raw = {0};
        LSM9DS1_RawData_t dma_raw = {0};
        HAL_StatusTypeDef step_ret;
        HAL_StatusTypeDef dma_start_ret;
        const LSM9DS1_RuntimeState_t *rt;
        uint32_t dma_ok_before;
        uint32_t dma_err_before;
        uint32_t wait_begin;

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

        rt = LSM9DS1_GetRuntimeState();
        dma_ok_before = rt->dma_ok_count;
        dma_err_before = rt->dma_error_count;
        dma_start_ret = LSM9DS1_TriggerRead_IT();

        if (dma_start_ret == HAL_OK)
        {
          wait_begin = HAL_GetTick();
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

      /* 旧版 AD4007 每秒调试流程：保留用于回退对照，不删除。 */
      {
        int32_t adc_code_spi = 0;
        int32_t adc_code_dma = 0;
        float adc_voltage_spi = 0.0f;
        float adc_voltage_dma = 0.0f;
        HAL_StatusTypeDef step_ret;

        step_ret = AD4007_test_Rx(&adc_code_spi);
        if (step_ret == HAL_OK)
        {
          adc_voltage_spi = AD4007_CodeToVoltage(adc_code_spi);
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[AD4007-A] SPI OK voltage=%.6f V\r\n",
                         (double)adc_voltage_spi);
        }
        else
        {
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[AD4007-A] SPI FAIL ret=%d\r\n",
                         (int)step_ret);
        }
        (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));

        step_ret = AD4007_test_DMA_Rx(&adc_code_dma, 20U);
        if (step_ret == HAL_OK)
        {
          adc_voltage_dma = AD4007_CodeToVoltage(adc_code_dma);
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[AD4007-C] DMA OK voltage=%.6f V spi_state=%d\r\n",
                         (double)adc_voltage_dma,
                         (int)hspi3.State);
        }
        else
        {
          (void)snprintf(g_usb_msg,
                         sizeof(g_usb_msg),
                         "[AD4007-C] DMA FAIL ret=%d spi_state=%d\r\n",
                         (int)step_ret,
                         (int)hspi3.State);
        }
        (void)CDC_Transmit_FS2((uint8_t *)g_usb_msg, (uint16_t)strlen(g_usb_msg));
      }
    }
#endif

 #if 1
    /*
     * 主协议帧发送路径（当前生效）。
     */
    if (g_group_frame_ready_for_send != 0U)
    {
      PrepareAndCommitGroupFrame();
    }

    if (g_ble_send_pending != 0U)
    {
      BLE_PackSingleFrame(&g_ble_send_frame, g_ble_frame);
      if (BLE_Transmit_Data_DMA(g_ble_frame, BLE_COMM_SINGLE_FRAME_SIZE) == HAL_OK)
      {
        g_ble_send_pending = 0U;
      }
    }
 #endif

  #if 0
    /*
     * 调试模式：仍执行组帧提交以维持状态机节拍，但丢弃主协议发送。
     */
    if (g_group_frame_ready_for_send != 0U)
    {
      PrepareAndCommitGroupFrame();
      if (g_ble_send_pending != 0U)
      {
        g_ble_send_pending = 0U;
      }
    }

    /* 每秒发送1条 BLE 文本调试帧。 */
    if ((now - g_last_print_tick) >= 1000U)
    {
      int len;
      uint32_t d_tim1;
      uint32_t d_m4;
      uint32_t d_cmp1;
      uint32_t d_cmp3;
      uint32_t d_rep;
      uint32_t d_rst2;
      uint32_t d_start;
      uint32_t d_ok;
      uint32_t d_miss;

      g_last_print_tick = now;
      d_tim1 = g_tim1_tick_count - g_last_tim1_tick_count;
      d_m4 = g_master_cmp4_count - g_last_master_cmp4_count;
      d_cmp1 = g_adc_cmp1_isr_count - g_last_adc_cmp1_isr_count;
      d_cmp3 = g_adc_cmp3_isr_count - g_last_adc_cmp3_isr_count;
      d_rep = g_adc_rep_isr_count - g_last_adc_rep_isr_count;
      d_rst2 = g_adc_rst2_isr_count - g_last_adc_rst2_isr_count;
      d_start = g_adc_start_ok_count - g_last_adc_start_ok_count;
      d_ok = g_adc_sample_ok_count - g_last_adc_sample_ok_count;
      d_miss = g_adc_rst2_to_dma_miss_count - g_last_adc_rst2_to_dma_miss_count;

      g_last_tim1_tick_count = g_tim1_tick_count;
      g_last_master_cmp4_count = g_master_cmp4_count;
      g_last_adc_cmp1_isr_count = g_adc_cmp1_isr_count;
      g_last_adc_cmp3_isr_count = g_adc_cmp3_isr_count;
      g_last_adc_rep_isr_count = g_adc_rep_isr_count;
      g_last_adc_rst2_isr_count = g_adc_rst2_isr_count;
      g_last_adc_start_ok_count = g_adc_start_ok_count;
      g_last_adc_sample_ok_count = g_adc_sample_ok_count;
      g_last_adc_rst2_to_dma_miss_count = g_adc_rst2_to_dma_miss_count;

      len = snprintf(g_ble_dbg_msg,
                     sizeof(g_ble_dbg_msg),
                     "DBG tot t1=%lu m4=%lu cmp1=%lu cmp3=%lu rep=%lu rst2=%lu st=%lu ok=%lu miss=%lu busy=%lu sf=%lu dec=%lu to=%lu | d1s t1=%lu m4=%lu cmp1=%lu cmp3=%lu rep=%lu rst2=%lu st=%lu ok=%lu miss=%lu\r\n",
                     (unsigned long)g_tim1_tick_count,
                     (unsigned long)g_master_cmp4_count,
                     (unsigned long)g_adc_cmp1_isr_count,
                     (unsigned long)g_adc_cmp3_isr_count,
                     (unsigned long)g_adc_rep_isr_count,
                     (unsigned long)g_adc_rst2_isr_count,
                     (unsigned long)g_adc_start_ok_count,
                     (unsigned long)g_adc_sample_ok_count,
                     (unsigned long)g_adc_rst2_to_dma_miss_count,
                     (unsigned long)g_adc_busy_skip_count,
                     (unsigned long)g_adc_dma_start_fail_count,
                     (unsigned long)g_adc_dma_decode_fail_count,
                     (unsigned long)g_adc_dma_timeout_count,
                     (unsigned long)d_tim1,
                     (unsigned long)d_m4,
                     (unsigned long)d_cmp1,
                     (unsigned long)d_cmp3,
                     (unsigned long)d_rep,
                     (unsigned long)d_rst2,
                     (unsigned long)d_start,
                     (unsigned long)d_ok,
                     (unsigned long)d_miss);
      if (len > 0)
      {
        (void)BLE_Transmit_Data_DMA((uint8_t *)g_ble_dbg_msg, (uint16_t)len);
      }
    }
#endif
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
    /* TIM1 为 400Hz 基准节拍，用于整体时序观测。 */
    g_tim1_tick_count++;
  }
}

void HAL_HRTIM_Compare4EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  uint8_t phase_before_rotate;

  if ((hhrtim != &hhrtim1) || (TimerIdx != HRTIM_TIMERINDEX_MASTER))
  {
    return;
  }

  g_master_cmp4_count++;

  phase_before_rotate = g_tim_group_phase;

  /* 事件4：归档本周期 3+3 脉冲采样结果，并切换到下一桥臂状态。 */
  ADC_FinalizeStateAndRotateBridge();

  if (phase_before_rotate == 1U)
  {
    /* 状态机1：读取 PPG（Green/Red/IR）并压入 ringbuffer。 */
    (void)MAX30101_TriggerPointerRead_IT();
  }
  else if (phase_before_rotate == 2U)
  {
    /* 状态机2：读取 MIMU 六轴并压入 ringbuffer。 */
    (void)LSM9DS1_TriggerRead_IT();
  }
  else if (phase_before_rotate == 4U)
  {
    /* 状态机4：当前4相位完成，通知主循环融合并发送。 */
    g_group_frame_ready_for_send = 1U;
  }
}

static void ADC_OnFallingEdgeTrigger(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    g_adc_fall_event_count++;

    /* TA2 reset：对应 CNV 下降沿，启动 AD4007 单点 SPI+DMA 读取。 */
    ADC_TryHarvestPendingSample();

    if (g_adc_dma_pending == 0U)
    {
      if (g_adc_pulse_start_index < 6U)
      {
        if (AD4007_Start_DMA_Rx(g_adc_dma_raw[g_adc_pulse_start_index], 1U) == HAL_OK)
        {
          g_adc_dma_pending_slot = g_adc_pulse_start_index;
          g_adc_dma_pending = 1U;
          g_adc_pulse_start_index++;
          g_adc_start_ok_count++;
        }
        else
        {
          g_adc_dma_start_fail_count++;
          g_adc_rst2_to_dma_miss_count++;
        }
      }
      else
      {
        /* 超出本轮6个采样槽位的RST2，不再启动DMA。 */
        g_adc_rst2_to_dma_miss_count++;
      }
    }
    else
    {
      /* 当前下降沿到来时，前一笔 SPI DMA 仍未完成，记为 busy skip。 */
      g_adc_busy_skip_count++;
      g_adc_rst2_to_dma_miss_count++;
    }
  }
}

void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    g_adc_cmp1_isr_count++;
  }

  /* 当前生效方案：在 CMP1 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_Compare3EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    g_adc_cmp3_isr_count++;
  }

  /* 当前生效方案：在 CMP3 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    g_adc_rep_isr_count++;
  }

  /* 当前生效方案：在 REP 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_Output2ResetCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    /* 当前生效仅计数：RST2 不再触发 DMA，保留用于口径对照。 */
    g_adc_rst2_isr_count++;
  }

  /*
   * 旧方案保留用于回滚：在 RST2 事件触发 ADC DMA。
   * ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
