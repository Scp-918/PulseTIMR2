#include "ble_comm.h"

#include <string.h>

#include "usart.h"

/* USART1 句柄由 CubeMX 生成。 */
extern UART_HandleTypeDef huart1;

/*
 * 按需求固定：10 帧批量发送缓存，单帧长度由协议宏决定。
 * 使用静态全局（文件作用域）避免栈开销，且确保 DMA 发送期间内存稳定。
 */
static uint8_t ble_tx_buffer[BLE_COMM_BATCH_TX_SIZE] = {0};

/* 当前批次已打包帧数。 */
static uint8_t frame_cnt = 0U;

/* 源端帧序号：每成功打包一帧后自增，uint16_t 自然回绕。 */
static uint16_t s_frame_seq = 0U;

/* 运行统计：用于诊断链路拥塞与发送异常。 */
static uint32_t s_batch_drop_busy_count = 0U;
static uint32_t s_batch_send_fail_count = 0U;

/*
 * @brief 将 int32_t 的低 24 位按小端写入 3 字节。
 * @note  显式位掩码与移位，避免类型提升与端序歧义。
 */
static void BLE_Write24LE_FromS32(int32_t value, uint8_t *dst)
{
  uint32_t raw = (uint32_t)value;

  dst[0] = (uint8_t)(raw & 0xFFU);
  dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
  dst[2] = (uint8_t)((raw >> 16) & 0xFFU);
}

/*
 * @brief 将 uint32_t 的低 24 位按小端写入 3 字节。
 */
static void BLE_Write24LE_FromU32(uint32_t value, uint8_t *dst)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
}

/*
 * @brief 将 int16_t 按小端写入 2 字节。
 */
static void BLE_Write16LE_FromS16(int16_t value, uint8_t *dst)
{
  uint16_t raw = (uint16_t)value;
  dst[0] = (uint8_t)(raw & 0xFFU);
  dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
}

void BLE_PackSingleFrame(SensorDataFrame_t *frame, uint8_t *out_buffer)
{
  uint8_t checksum = 0U;
  uint8_t ch;
  uint8_t idx;
  uint16_t offset;

  if ((frame == NULL) || (out_buffer == NULL))
  {
    return;
  }

  /* 先清零，避免未覆盖字节带入脏数据。 */
  (void)memset(out_buffer, 0, BLE_COMM_SINGLE_FRAME_SIZE);

  /* 帧头 */
  out_buffer[BLE_COMM_IDX_HEADER0] = BLE_COMM_FRAME_HEADER_BYTE0;
  out_buffer[BLE_COMM_IDX_HEADER1] = BLE_COMM_FRAME_HEADER_BYTE1;

  /*
   * [2..25] ADC 区：
   * - 共 4 通道（adc_data[0..3]）
   * - 每通道按 early(3B) + late(3B) 排列
   * - 单通道占 6 字节，总计 24 字节
   */
  for (ch = 0U; ch < BLE_COMM_ADC_CHANNEL_COUNT; ch++)
  {
    offset = (uint16_t)BLE_COMM_IDX_ADC_START +
             ((uint16_t)ch * (uint16_t)BLE_COMM_ADC_BYTES_PER_CHANNEL);

    BLE_Write24LE_FromS32(frame->adc_data[ch].early_code, &out_buffer[offset]);
    BLE_Write24LE_FromS32(frame->adc_data[ch].late_code,
                          &out_buffer[offset + BLE_COMM_ADC_BYTES_PER_VALUE]);
  }

  /*
   * [26..34] PPG 区：
   * - 3 通道（Green/Red/IR）
   * - 每通道低 24 位小端写入 3 字节
   */
  for (ch = 0U; ch < BLE_COMM_PPG_CHANNEL_COUNT; ch++)
  {
    offset = (uint16_t)BLE_COMM_IDX_PPG_START +
             ((uint16_t)ch * (uint16_t)BLE_COMM_PPG_BYTES_PER_CHANNEL);
    BLE_Write24LE_FromU32(frame->ppg_data[ch], &out_buffer[offset]);
  }

  /*
   * [35..46] MIMU 区：
   * - 6 轴（Gx/Gy/Gz/Ax/Ay/Az）
   * - 每轴 int16 小端写入 2 字节
   */
  for (ch = 0U; ch < BLE_COMM_IMU_AXIS_COUNT; ch++)
  {
    offset = (uint16_t)BLE_COMM_IDX_IMU_START +
             ((uint16_t)ch * (uint16_t)BLE_COMM_IMU_BYTES_PER_AXIS);
    BLE_Write16LE_FromS16(frame->imu_data[ch], &out_buffer[offset]);
  }

  /* [47] Checksum = XOR([2]..[46])，校验区不包含帧头和 frame_seq。 */
  for (idx = (uint8_t)BLE_COMM_XOR_START_IDX; idx <= (uint8_t)BLE_COMM_XOR_END_IDX; idx++)
  {
    checksum ^= out_buffer[idx];
  }
  out_buffer[BLE_COMM_IDX_CHECKSUM] = checksum;

  /* [48..49] 源端帧序号，小端，不参与旧 XOR 校验。 */
  out_buffer[BLE_COMM_IDX_FRAME_SEQ_L] = (uint8_t)(s_frame_seq & 0xFFU);
  out_buffer[BLE_COMM_IDX_FRAME_SEQ_H] = (uint8_t)((s_frame_seq >> 8) & 0xFFU);

  /* [50] 单字节帧尾：0xCC。 */
  out_buffer[BLE_COMM_IDX_TAIL0] = BLE_COMM_FRAME_TAIL_BYTE0;

  s_frame_seq++;
}

bool BLE_Task_Process(SensorRingBuffer_t *rb)
{
  SensorDataFrame_t frame;
  uint16_t offset;

  if (rb == NULL)
  {
    return false;
  }

  /* 无可用数据则直接返回，保持任务非阻塞。 */
  if (RingBuffer_GetCount(rb) == 0U)
  {
    return false;
  }

  if (!RingBuffer_Pop(rb, &frame))
  {
    return false;
  }

  /* 将当前帧打包到批次缓存的对应槽位。 */
  offset = (uint16_t)frame_cnt * (uint16_t)BLE_COMM_SINGLE_FRAME_SIZE;
  BLE_PackSingleFrame(&frame, &ble_tx_buffer[offset]);

  frame_cnt++;

  /* 聚满 10 帧后，尝试启动一次 DMA 批量发送。 */
  if (frame_cnt >= BLE_COMM_BATCH_FRAME_COUNT)
  {
    if (huart1.gState == HAL_UART_STATE_READY)
    {
      HAL_StatusTypeDef tx_ret = HAL_UART_Transmit_DMA(&huart1, ble_tx_buffer, BLE_COMM_BATCH_TX_SIZE);
      if (tx_ret != HAL_OK)
      {
        /* DMA 启动失败：记录计数，丢弃该批，避免卡死在满批状态。 */
        s_batch_send_fail_count++;
      }
    }
    else
    {
      /* UART 仍忙：按需求可选择丢弃整批，本实现采用丢批并计数。 */
      s_batch_drop_busy_count++;
    }

    /* 无论发送成功与否，本批次结束，重新累计下一批 10 帧。 */
    frame_cnt = 0U;
  }

  return true;
}

uint32_t BLE_Comm_GetBatchDropBusyCount(void)
{
  return s_batch_drop_busy_count;
}

uint32_t BLE_Comm_GetBatchSendFailCount(void)
{
  return s_batch_send_fail_count;
}

uint8_t BLE_Comm_GetPendingFrameCount(void)
{
  return frame_cnt;
}
