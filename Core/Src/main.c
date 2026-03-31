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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * PPG 发光电流初始化（MAX30101，单位为寄存器码值，约 0.2mA/LSB）：
 * - Green 提升以增强绿光通道信号幅度
 * - Red/IR 维持较低电流用于抑制饱和
 */
#define PPG_INIT_GREEN_PA (0x8FU)
#define PPG_INIT_RED_PA   (0x24U)
#define PPG_INIT_IR_PA    (0x24U)

/*
 * BLE 参数帧固定长度（13字节）：
 * [0]=0xAB [1]=0xCD
 * [2]=PPG mode(01~03)
 * [3]=PPG multi sub-mode(01~05，仅 mode=01 有效)
 * [4]=Green 电流档位(00~09)
 * [5]=Red 电流档位(00~09)
 * [6]=IR 电流档位(00~09)
 * [7]=PPG ADC 量程(01~04)
 * [8]=PPG 脉宽(01~04)
 * [9]=Gyro 量程(01~03)
 * [10]=Accel 量程(01~04)
 * [11]=0xEF [12]=0xFA
 */
#define SENSOR_PARAM_FRAME_LEN                (13U)
/* 修改后禁发窗口：保留采样更新，但暂停发送 100 个大周期。 */
#define SENSOR_SEND_HOLD_CYCLES               (100U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 4 相位状态机的当前相位（1~4），由 HRTIM Master CMP4 事件推进。 */
static volatile uint8_t g_tim_group_phase = 1U;
/* 一组 4 相位结束后由 ISR 置位，主循环据此提交并发送融合帧。 */
static volatile uint8_t g_group_frame_ready_for_send = 0U;

/* AD4007 当前是否仍有一笔 DMA 结果待解析（避免重入覆盖）。 */
static volatile uint8_t g_adc_dma_pending = 0U;
/* 待解析 DMA 数据对应的槽位索引（0~5，对应 3+3 脉冲）。 */
static volatile uint8_t g_adc_dma_pending_slot = 0U;
/* 本相位周期内下一个可用采样槽位。 */
static volatile uint8_t g_adc_pulse_start_index = 0U;

/* 早窗/晚窗累加器：每相位内分别统计 3 个采样点并取平均。 */
static int64_t g_adc_early_sum = 0;
static int64_t g_adc_late_sum = 0;
static uint8_t g_adc_early_count = 0U;
static uint8_t g_adc_late_count = 0U;

/* AD4007 原始 DMA 缓冲：6 槽 * 每槽 1 帧原始字节。 */
static uint8_t g_adc_dma_raw[6][AD4007_FRAME_BYTES] = {{0}};
/* 当前正在构建的组帧（4 相位内持续填充）。 */
static SensorDataFrame_t g_group_frame = {0};
/* 主循环待发送帧（一帧延迟发送策略）。 */
static SensorDataFrame_t g_ble_send_frame = {0};
static uint8_t g_ble_send_pending = 0U;
/* 邻帧补零策略使用的历史帧缓存。 */
static SensorDataFrame_t g_tx_prev_frame = {0};
static SensorDataFrame_t g_tx_prev_prev_frame = {0};
static uint8_t g_tx_prev_valid = 0U;
static uint8_t g_tx_prev_prev_valid = 0U;

/*
 * 当前生效数据通路：
 * ISR 生产者：MAX30101/LSM9DS1 DMA 回调 -> RingBuffer_Push()
 * 主循环消费者：RingBuffer_Pop() -> 融合到当前大周期帧 -> BLE_PackSingleFrame() -> BLE UART DMA
 */
static SensorRingBuffer_t g_sensor_rb;
/* 主协议帧缓存。 */
static uint8_t g_ble_frame[BLE_COMM_SINGLE_FRAME_SIZE];

/* 统计每组融合帧中 PPG/MIMU 是否缺失，用于链路健康度评估。 */
static uint32_t g_group_total_count = 0U;
static uint32_t g_group_miss_ppg_count = 0U;
static uint32_t g_group_miss_imu_count = 0U;

/* BLE 串口 DMA 接收缓存：用于上位机->USART1 的透传链路。 */
static uint8_t g_ble_uart_rx_dma_buf[BLE_RX_FRAME_MAX_LEN] = {0};
/* 主循环参数解析缓冲（与 DMA 接收缓存分离，避免覆盖竞争）。 */
static uint8_t g_ble_rx_parse_buf[BLE_RX_FRAME_MAX_LEN] = {0};

/* 单片机当前生效传感参数数组（协议格式与上位机一致）。 */
static volatile uint8_t g_sensor_param_array[SENSOR_PARAM_FRAME_LEN] = {
  0xABU, 0xCDU,
  0x01U, /* mode: Multi-LED */
  0x01U, /* multimode: G-R-IR */
  0x05U, /* green level */
  0x01U, /* red level */
  0x01U, /* ir level */
  0x03U, /* ADC range: 8192nA */
  0x03U, /* pulse width: 215us */
  0x02U, /* gyro FS: +-500dps */
  0x01U, /* accel FS: +-2g */
  0xEFU, 0xFAU
};

/*
 * 参数下发状态位说明：
 * g_sensor_param_pending:
 *   主循环已收到并通过校验，等待在 phase1/phase2 写入传感器。
 * g_sensor_cfg_apply_cycle_active:
 *   “配置应用周期”标志。置位后，该大周期不采集、不发送，整帧记0。
 * g_sensor_drop_pipeline_once:
 *   清空一帧延迟发送流水线，避免配置切换前的历史帧被发出。
 * g_sensor_send_hold_countdown:
 *   配置完成后继续禁发 N 周期，期间仍正常采样并刷新缓存。
 * g_sensor_cfg_apply_error:
 *   记录本轮配置是否发生寄存器写失败（保留供后续扩展上报）。
 */
static volatile uint8_t g_sensor_param_pending = 0U;
static uint8_t g_sensor_param_pending_buf[SENSOR_PARAM_FRAME_LEN] = {0};
static volatile uint8_t g_sensor_cfg_apply_cycle_active = 0U;
static volatile uint8_t g_sensor_cfg_apply_error = 0U;
static volatile uint8_t g_sensor_drop_pipeline_once = 0U;
static volatile uint16_t g_sensor_send_hold_countdown = 0U;
/* 每秒自检发送逻辑已停用，保留旧定义便于回滚。 */
#if 0
static const uint8_t g_check_ble_text[] = "[SYS->BLE] check\r\n";
static const uint8_t g_check_usb_text[] = "[SYS->USB] check\r\n";
static uint32_t g_last_check_tick_ms = 0U;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 根据 4 相位状态机选择三路 TMUX 开关组合，驱动桥臂切换。 */
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

/*
 * BLE 亮度档位(0..9)映射到 MAX30101 LED 电流寄存器码值(0..255)。
 * 采用线性映射：level=0 -> 0x00，level=9 -> 0xFF。
 * 映射后典型电流近似 I_LED ~= reg * 0.2mA。
 */
static uint8_t SensorParam_LevelToLedCurrentCode(uint8_t level)
{
  if (level > 9U)
  {
    level = 9U;
  }

  return (uint8_t)(((uint16_t)level * 255U + 4U) / 9U);
}

/*
 * 判断当前是否“单色光”配置：
 * - mode=0x02(HR) 固定单色（Red）
 * - mode=0x01(Multi) 且 sub-mode=02/03/04 时为单色（G/R/IR 单路）
 */
static uint8_t SensorParam_IsSingleColor(const uint8_t *params)
{
  if (params == NULL)
  {
    return 0U;
  }

  if (params[2] == 0x02U)
  {
    return 1U;
  }

  if ((params[2] == 0x01U) &&
      ((params[3] == 0x02U) || (params[3] == 0x03U) || (params[3] == 0x04U)))
  {
    return 1U;
  }

  return 0U;
}

/*
 * 按协议脉宽索引映射 SR 与 SMP_AVE：
 * - 单色：
 *   69us -> 3200sps + 32x
 *   118us/215us -> 1600sps + 16x
 *   411us -> 800sps + 8x
 * - 多色（双色/三色）：
 *   69/118/215us -> 400sps + 4x
 *   411us -> 200sps + 2x
 */
static uint8_t SensorParam_GetPPGTimingCodes(uint8_t pulse_sel,
                                             uint8_t is_single_color,
                                             uint8_t *out_sr_code,
                                             uint8_t *out_smp_ave_code)
{
  if ((out_sr_code == NULL) || (out_smp_ave_code == NULL))
  {
    return 0U;
  }

  switch (pulse_sel)
  {
    case 0x01U:
      if (is_single_color != 0U)
      {
        *out_sr_code = 0x07U;  /* 3200sps */
        *out_smp_ave_code = 0x05U; /* 32x */
      }
      else
      {
        *out_sr_code = 0x03U;  /* 400sps */
        *out_smp_ave_code = 0x02U; /* 4x */
      }
      break;

    case 0x02U:
      if (is_single_color != 0U)
      {
        *out_sr_code = 0x06U;  /* 1600sps */
        *out_smp_ave_code = 0x04U; /* 16x */
      }
      else
      {
        *out_sr_code = 0x03U;  /* 400sps */
        *out_smp_ave_code = 0x02U; /* 4x */
      }
      break;

    case 0x03U:
      if (is_single_color != 0U)
      {
        *out_sr_code = 0x06U;  /* 1600sps */
        *out_smp_ave_code = 0x04U; /* 16x */
      }
      else
      {
        *out_sr_code = 0x03U;  /* 400sps */
        *out_smp_ave_code = 0x02U; /* 4x */
      }
      break;

    case 0x04U:
      if (is_single_color != 0U)
      {
        *out_sr_code = 0x04U;  /* 800sps */
        *out_smp_ave_code = 0x03U; /* 8x */
      }
      else
      {
        *out_sr_code = 0x02U;  /* 200sps */
        *out_smp_ave_code = 0x01U; /* 2x */
      }
      break;

    default:
      return 0U;
  }

  return 1U;
}

/*
 * 把协议参数应用到 MAX30101：
 * 1) 先根据 mode/sub-mode 判断单色或多色，得到 SR/平均值组合。
 * 2) 把 00~09 的亮度档位换算到 LED 寄存器码值。
 * 3) 把协议中 01 基索引转换为寄存器字段 00 基编码。
 * 4) 调用驱动统一入口写寄存器与更新解码档位。
 */
static uint8_t SensorParam_ApplyPPG(const uint8_t *params)
{
  uint8_t sr_code;
  uint8_t smp_ave_code;
  uint8_t is_single_color;
  uint8_t green_current;
  uint8_t red_current;
  uint8_t ir_current;
  uint8_t adc_rge_code;
  uint8_t led_pw_code;

  if (params == NULL)
  {
    return 0U;
  }

  is_single_color = SensorParam_IsSingleColor(params);
  if (SensorParam_GetPPGTimingCodes(params[8], is_single_color, &sr_code, &smp_ave_code) == 0U)
  {
    return 0U;
  }

  green_current = SensorParam_LevelToLedCurrentCode(params[4]);
  red_current = SensorParam_LevelToLedCurrentCode(params[5]);
  ir_current = SensorParam_LevelToLedCurrentCode(params[6]);

  adc_rge_code = (uint8_t)(params[7] - 1U);
  led_pw_code = (uint8_t)(params[8] - 1U);

  return MAX30101_ApplyBleParamConfig(params[2],
                                      params[3],
                                      green_current,
                                      red_current,
                                      ir_current,
                                      adc_rge_code,
                                      led_pw_code,
                                      sr_code,
                                      smp_ave_code);
}

/*
 * 把协议参数应用到 LSM9DS1：
 * - 只改 FS（量程）位，不改 ODR/BW/滤波链路。
 * - 通过驱动层 LSM9DS1_SetFullScale() 执行原子更新。
 */
static HAL_StatusTypeDef SensorParam_ApplyIMU(const uint8_t *params)
{
  uint8_t gyro_fs;
  uint8_t accel_fs;

  if (params == NULL)
  {
    return HAL_ERROR;
  }

  switch (params[9])
  {
    case 0x01U:
      gyro_fs = LSM9DS1_GYRO_FS_245DPS;
      break;
    case 0x02U:
      gyro_fs = LSM9DS1_GYRO_FS_500DPS;
      break;
    case 0x03U:
      gyro_fs = LSM9DS1_GYRO_FS_2000DPS;
      break;
    default:
      return HAL_ERROR;
  }

  switch (params[10])
  {
    case 0x01U:
      accel_fs = LSM9DS1_ACCEL_FS_2G;
      break;
    case 0x02U:
      accel_fs = LSM9DS1_ACCEL_FS_4G;
      break;
    case 0x03U:
      accel_fs = LSM9DS1_ACCEL_FS_8G;
      break;
    case 0x04U:
      accel_fs = LSM9DS1_ACCEL_FS_16G;
      break;
    default:
      return HAL_ERROR;
  }

  return LSM9DS1_SetFullScale(accel_fs, gyro_fs);
}

/*
 * 从 UART DMA 字节流中提取并校验 13 字节参数帧：
 * - 允许在长串流中滑窗查找 AB CD ... EF FA。
 * - 逐字段做范围校验，过滤噪声帧/半包。
 * - 非 Multi 模式时，sub-mode 固定要求 0x01（保留位语义一致）。
 */
static uint8_t SensorParam_ParseFrame(const uint8_t *rx, uint16_t rx_len, uint8_t out_params[SENSOR_PARAM_FRAME_LEN])
{
  uint16_t i;
  const uint8_t *frm;

  if ((rx == NULL) || (out_params == NULL) || (rx_len < SENSOR_PARAM_FRAME_LEN))
  {
    return 0U;
  }

  for (i = 0U; i <= (uint16_t)(rx_len - SENSOR_PARAM_FRAME_LEN); i++)
  {
    if ((rx[i] == 0xABU) &&
      (rx[i + 1U] == 0xCDU) &&
        (rx[i + 11U] == 0xEFU) &&
        (rx[i + 12U] == 0xFAU))
    {
      frm = &rx[i];

      if ((frm[2] < 0x01U) || (frm[2] > 0x03U))
      {
        continue;
      }
      if ((frm[3] < 0x01U) || (frm[3] > 0x05U))
      {
        continue;
      }
      if ((frm[4] > 0x09U) || (frm[5] > 0x09U) || (frm[6] > 0x09U))
      {
        continue;
      }
      if ((frm[7] < 0x01U) || (frm[7] > 0x04U))
      {
        continue;
      }
      if ((frm[8] < 0x01U) || (frm[8] > 0x04U))
      {
        continue;
      }
      if ((frm[9] < 0x01U) || (frm[9] > 0x03U))
      {
        continue;
      }
      if ((frm[10] < 0x01U) || (frm[10] > 0x04U))
      {
        continue;
      }
      if ((frm[2] != 0x01U) && (frm[3] != 0x01U))
      {
        /* 非 Multi 模式时，子模式字段必须保持 0x01。 */
        continue;
      }

      (void)memcpy(out_params, frm, SENSOR_PARAM_FRAME_LEN);
      return 1U;
    }
  }

  return 0U;
}

/*
 * 写入参数时采用短临界区：
 * - 主循环解析与 HRTIM 回调可能并发访问 pending buffer。
 * - 通过关中断保证 13 字节数组与 pending 标记一致更新。
 */
static void SensorParam_QueueFromBleRx(const uint8_t *rx, uint16_t rx_len)
{
  uint8_t parsed[SENSOR_PARAM_FRAME_LEN] = {0};

  if (SensorParam_ParseFrame(rx, rx_len, parsed) == 0U)
  {
    return;
  }

  __disable_irq();
  (void)memcpy((void *)g_sensor_param_array, parsed, SENSOR_PARAM_FRAME_LEN);
  (void)memcpy(g_sensor_param_pending_buf, parsed, SENSOR_PARAM_FRAME_LEN);
  g_sensor_param_pending = 1U;
  __enable_irq();
}

/*
 * 清空环形缓冲区：
 * 配置切换周期需要“整帧记0且不发送”，
 * 因此必须丢弃切换前残留的 PPG/MIMU 片段，防止跨配置污染。
 */
static void Sensor_DrainRingBuffer(SensorRingBuffer_t *rb)
{
  SensorDataFrame_t dummy;

  if (rb == NULL)
  {
    return;
  }

  while (RingBuffer_Pop(rb, &dummy))
  {
  }
}

/* 清空“单相位”内 ADC 统计窗口，为下一相位 3+3 脉冲采样做准备。 */
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

/* 在 SPI3 空闲时解析待处理 DMA 原始数据，并归入 early/late 累加器。 */
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

  g_adc_dma_pending = 0U;
}

/*
 * 在每个相位结束时：
 * 1) 结算该相位 early/late 平均值写入组帧；
 * 2) 轮转桥臂到下一个相位；
 * 3) 复位相位内采样窗口。
 */
static void ADC_FinalizeStateAndRotateBridge(void)
{
  uint8_t state_index = (uint8_t)(g_tim_group_phase - 1U);
  int32_t early_avg = 0;
  int32_t late_avg = 0;

  ADC_TryHarvestPendingSample();
  if (g_adc_dma_pending != 0U)
  {
    (void)HAL_SPI_Abort(&hspi3);
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

/* 判断帧内是否包含有效 PPG 三通道数据。 */
static uint8_t FrameHasPPGPayload(const SensorDataFrame_t *frame)
{
  if ((frame->ppg_data[0] != 0U) || (frame->ppg_data[1] != 0U) || (frame->ppg_data[2] != 0U))
  {
    return 1U;
  }
  return 0U;
}

/* 判断帧内是否包含有效 IMU 六轴数据。 */
static uint8_t FrameHasIMUPayload(const SensorDataFrame_t *frame)
{
  if ((frame->imu_data[0] != 0) || (frame->imu_data[1] != 0) || (frame->imu_data[2] != 0) ||
      (frame->imu_data[3] != 0) || (frame->imu_data[4] != 0) || (frame->imu_data[5] != 0))
  {
    return 1U;
  }
  return 0U;
}

/* 若“待发送帧”PPG全0，则用前后帧PPG均值修复。 */
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

  sum = prev_frame->ppg_data[0] + next_frame->ppg_data[0];
  out_frame->ppg_data[0] = (sum >> 1);
  sum = prev_frame->ppg_data[1] + next_frame->ppg_data[1];
  out_frame->ppg_data[1] = (sum >> 1);
  sum = prev_frame->ppg_data[2] + next_frame->ppg_data[2];
  out_frame->ppg_data[2] = (sum >> 1);
}

/*
 * 从传感器环形缓冲区提取当前周期可用的最新片段帧：
 * - PPG 与 IMU 分别以“最后一次非零数据”为准覆盖到 g_group_frame；
 * - 若本周期缺失对应数据，则更新缺失计数。
 */
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

/*
 * 提交当前组帧并维护一帧延迟发送管线：
 * - 当前周期只提交，不直接发送；
 * - 真正发送的是上一个周期帧（可做邻帧补零）；
 * - 提交后立即清空当前组帧，进入下一周期。
 */
static void PrepareAndCommitGroupFrame(void)
{
  uint8_t committed = 0U;

  /* UART DMA 忙时不推进缓存，避免丢失“上一帧发送”时序关系。 */
  if (g_ble_send_pending != 0U)
  {
    return;
  }

  if (g_sensor_drop_pipeline_once != 0U)
  {
    /*
     * 一次性丢弃发送流水线：
     * - 清 pending 发送与 prev/prev_prev 缓存
     * - 清 ringbuffer 中旧参数残留片段
     */
    __disable_irq();
    g_ble_send_pending = 0U;
    g_tx_prev_valid = 0U;
    g_tx_prev_prev_valid = 0U;
    __enable_irq();

    Sensor_DrainRingBuffer(&g_sensor_rb);
    (void)memset(&g_tx_prev_frame, 0, sizeof(g_tx_prev_frame));
    (void)memset(&g_tx_prev_prev_frame, 0, sizeof(g_tx_prev_prev_frame));
    g_sensor_drop_pipeline_once = 0U;
  }

  if (g_sensor_cfg_apply_cycle_active == 0U)
  {
    Sensor_IngestPartialFramesForCurrentGroup();
  }
  else
  {
    /* 配置应用周期内，本组帧强制全零。 */
    (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
  }

  __disable_irq();

  /* 一帧延迟发送：当前周期到来时发送上一周期缓存；禁发窗口内跳过。 */
  if ((g_tx_prev_valid != 0U) && (g_sensor_send_hold_countdown == 0U))
  {
    g_ble_send_frame = g_tx_prev_frame;
    if (g_tx_prev_prev_valid != 0U)
    {
      PatchZeroPPGWithNeighborAverage(&g_ble_send_frame, &g_tx_prev_prev_frame, &g_group_frame);
    }
    g_ble_send_pending = 1U;
  }

  g_tx_prev_prev_frame = g_tx_prev_frame;
  g_tx_prev_prev_valid = g_tx_prev_valid;
  g_tx_prev_frame = g_group_frame;
  g_tx_prev_valid = 1U;

  committed = 1U;
  __enable_irq();

  if (committed != 0U)
  {
    (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
    g_group_total_count++;
    g_group_frame_ready_for_send = 0U;
    if (g_sensor_cfg_apply_cycle_active != 0U)
    {
      /* 配置应用周期在“提交点”自动退出。 */
      g_sensor_cfg_apply_cycle_active = 0U;
    }
    else if (g_sensor_send_hold_countdown > 0U)
    {
      /* 后续周期按 1/group 递减禁发计数。 */
      g_sensor_send_hold_countdown--;
    }
  }
}

/*
 * 程序总体用途（当前生效版本）：
 * 1) 启用 TIM1(400Hz) -> HRTIM 同步的 4 状态机。
 * 2) 在每次状态周期中由 TimerA Output2 产生 3+3 个 CNV 脉冲，并在 reset 边沿触发 AD4007 SPI+DMA 读取。
 * 3) 状态机1/2在 Master CMP4 分别触发 PPG/MIMU DMA 读取并写入 ringbuffer。
 * 4) 状态机4完成后融合为单帧，通过 BLE 协议格式发送（包含 4 组 ADC early/late）。
 */

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
  /*
   * 当前生效流程（ADC + PPG/MIMU + BLE 帧发送）：
   * 1) 初始化 GPIO/DMA/I2C3/SPI1/SPI3/UART1/HRTIM/TIM1；
   * 2) BLE 初始化后，初始化 PPG/IMU 并绑定同一 ringbuffer；
   * 3) TIM1(400Hz) 驱动 4 相位状态机：phase1 触发 PPG，phase2 触发 MIMU，phase4 提交并发送；
   * 4) AD4007 每相位执行 3+3 脉冲采样，4 相位 early/late 结果汇入同一发送帧。
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  // MX_USB_Device_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();

  /* 阶段1：初始化模拟开关与统一 ringbuffer。 */
  TMUX_Global_Init();
  RingBuffer_Init(&g_sensor_rb);

  /*
   * 阶段2：传感器电源上电序列。
   * 必须按 E5V -> E3.3V -> E4V 顺序拉起，并预留稳定时间。
   */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(50);

  /* 阶段3：建立 BLE 通道，作为主协议帧输出链路。 */
  if (BLE_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 阶段4：绑定 PPG/MIMU 到同一 ringbuffer，统一交由主循环融合。 */
  MAX30101_AttachRingBuffer(&g_sensor_rb);
  LSM9DS1_AttachRingBuffer(&g_sensor_rb);

  /* 阶段5：初始化 PPG，配置为三光路轮切。 */
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

  /* 阶段6：初始化 MIMU。 */
  if (LSM9DS1_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 阶段7：初始化 AD4007，准备进入 HRTIM 驱动采样。 */
  if (AD4007_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 阶段8：设置初始桥臂相位，并清空本轮采样累加器。 */
  g_tim_group_phase = 1U;
  Bridge_ApplyState(g_tim_group_phase);
  ADC_ResetCycleAccumulator();
  (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
  (void)memset(&g_ble_send_frame, 0, sizeof(g_ble_send_frame));

  /* 启用 BLE RX DMA：用于持续接收上位机 13 字节参数配置帧。 */

  if (BLE_Start_Receive_DMA(g_ble_uart_rx_dma_buf, (uint16_t)sizeof(g_ble_uart_rx_dma_buf)) != HAL_OK)
  {
    Error_Handler();
  }

  /* 阶段9：启动 HRTIM 计数器与输出，进入硬件触发采样状态。 */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ADC_TryHarvestPendingSample();

    /*
     * 仅在 phase4 提交标志到来后执行一次组帧提交：
     * - 从 ringbuffer 合并本大周期最新 PPG/IMU
     * - 维护一帧延迟发送管线
     * - 根据禁发窗口决定是否产生 g_ble_send_pending
     */
    if (g_group_frame_ready_for_send != 0U)
    {
      PrepareAndCommitGroupFrame();
    }

    /*
     * 发送阶段与采样阶段解耦：
     * - 仅当上一帧已就绪且 UART DMA 空闲时发起发送
     * - 发送成功后清 pending；若串口忙则保持 pending 等下一轮
     */
    if (g_ble_send_pending != 0U)
    {
      BLE_PackSingleFrame(&g_ble_send_frame, g_ble_frame);
      if (BLE_Transmit_Data_DMA(g_ble_frame, BLE_COMM_SINGLE_FRAME_SIZE) == HAL_OK)
      {
        g_ble_send_pending = 0U;
      }
    }

    /*
     * BLE UART DMA 收到上位机参数后：
     * 1) 在主循环完成帧提取与格式校验（避免中断中做复杂解析）
     * 2) 仅置位 pending，实际寄存器写入延后到 HRTIM 固定相位执行
     */
    {
      uint16_t rx_len = 0U;

      if (BLE_FetchRxFrame(g_ble_rx_parse_buf, (uint16_t)sizeof(g_ble_rx_parse_buf), &rx_len) != 0U)
      {
        SensorParam_QueueFromBleRx(g_ble_rx_parse_buf, rx_len);
      }
    }

    /* USB 发送路径按需求停用，保留旧逻辑便于快速回滚。 */
    #if 0
    {
      uint16_t rx_len = 0U;
      if (BLE_FetchRxFrame(g_ble_to_usb_buf, (uint16_t)sizeof(g_ble_to_usb_buf), &rx_len) != 0U)
      {
        if (rx_len > 0U)
        {
          (void)CDC_Transmit_FS2((uint8_t *)g_ble_to_usb_tag,
                                 (uint16_t)(sizeof(g_ble_to_usb_tag) - 1U));
          (void)CDC_Transmit_FS2(g_ble_to_usb_buf, rx_len);
          (void)CDC_Transmit_FS2((uint8_t *)g_ble_to_usb_eol,
                                 (uint16_t)(sizeof(g_ble_to_usb_eol) - 1U));
        }
      }
    }
    #endif

    /* 每秒 check 发送已按需求移除（旧逻辑见上方 #if 0 变量定义）。 */
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
    /* TIM1 仅作为 HRTIM 同步触发基准；软件侧无需额外处理。 */
  }
}

/*
 * Master CMP4 是 4 相位状态机的“相位提交点”：
 * - 结算当前相位 ADC 结果并轮转桥臂；
 * - 正常情况下按相位触发 PPG/MIMU 读取；
 * - 若检测到新参数，在 phase1/phase2 改为写配置，不读传感器；
 * - phase4 结束后通知主循环提交本组帧（配置周期内该帧为全0）。
 */
void HAL_HRTIM_Compare4EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  uint8_t phase_before_rotate;

  if ((hhrtim != &hhrtim1) || (TimerIdx != HRTIM_TIMERINDEX_MASTER))
  {
    return;
  }

  phase_before_rotate = g_tim_group_phase;

  /* 事件4：归档本周期 3+3 脉冲采样结果，并切换到下一桥臂状态。 */
  ADC_FinalizeStateAndRotateBridge();

  if (phase_before_rotate == 1U)
  {
    /*
     * 状态机1：
     * - 正常：触发 PPG 指针读取
     * - 收到参数：进入“配置应用周期”，写 MAX30101 参数并清 ADC 累加器
     */
    if (g_sensor_param_pending != 0U)
    {
      g_sensor_cfg_apply_cycle_active = 1U;
      g_sensor_drop_pipeline_once = 1U;
      /* 先写 PPG 参数；失败仅置错误标记，不打断状态机。 */
      g_sensor_cfg_apply_error = (SensorParam_ApplyPPG(g_sensor_param_pending_buf) == 0U) ? 1U : 0U;
      ADC_ResetCycleAccumulator();
      (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
    }
    else if (g_sensor_cfg_apply_cycle_active == 0U)
    {
      (void)MAX30101_TriggerPointerRead_IT();
    }
  }
  else if (phase_before_rotate == 2U)
  {
    /*
     * 状态机2：
     * - 正常：触发 MIMU 读取
     * - 配置周期：写 IMU 量程、清 pending 标志，并启动 100 周期禁发窗口
     */
    if (g_sensor_cfg_apply_cycle_active != 0U)
    {
      if (SensorParam_ApplyIMU(g_sensor_param_pending_buf) != HAL_OK)
      {
        /* IMU 参数写失败同样只置位，防止打乱既定采样节拍。 */
        g_sensor_cfg_apply_error = 1U;
      }
      /* 2/2 写完后清 pending，并进入“继续采样但禁发”窗口。 */
      g_sensor_param_pending = 0U;
      g_sensor_send_hold_countdown = SENSOR_SEND_HOLD_CYCLES;
    }
    else
    {
      (void)LSM9DS1_TriggerRead_IT();
    }
  }
  else if (phase_before_rotate == 4U)
  {
    if (g_sensor_cfg_apply_cycle_active != 0U)
    {
      /* 配置应用周期内，state4 提交前再次确保组帧全零。 */
      (void)memset(&g_group_frame, 0, sizeof(g_group_frame));
    }
    /* 状态机4：当前4相位完成，通知主循环融合并发送。 */
    g_group_frame_ready_for_send = 1U;
  }
}

static void ADC_OnFallingEdgeTrigger(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1) && (TimerIdx == HRTIM_TIMERINDEX_TIMER_A))
  {
    if (g_sensor_cfg_apply_cycle_active != 0U)
    {
      /* 配置应用周期内禁用 AD4007 采样触发，确保本周期 ADC 也记零。 */
      return;
    }

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
        }
      }
    }
  }
}

void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  /* 当前生效方案：在 CMP1 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_Compare3EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  /* 当前生效方案：在 CMP3 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  /* 当前生效方案：在 REP 事件触发 ADC DMA。 */
  ADC_OnFallingEdgeTrigger(hhrtim, TimerIdx);
}

void HAL_HRTIM_Output2ResetCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  (void)hhrtim;
  (void)TimerIdx;
  /* 当前策略不使用 RST2 触发采样，仅保留回调占位。 */
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
