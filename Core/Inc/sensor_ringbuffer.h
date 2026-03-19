/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_ringbuffer.h
  * @brief          : 多传感器同步数据环形缓冲区接口定义
  ******************************************************************************
  * @attention
  *
  * 本模块用于连接 "中断/DMA 生产者" 与 "主循环消费者"。
  * 设计目标：
  * 1) 入队在中断中高频调用，尽量无阻塞。
  * 2) 出队在主循环调用，采用极短临界区保护关键索引更新。
  * 3) 满载时丢弃最老帧，优先保留最新数据，降低蓝牙阻塞导致的时序失真。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SENSOR_RINGBUFFER_H
#define __SENSOR_RINGBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
/**
  * @brief 环形缓冲区深度。
  *
  * 32 帧通常可覆盖超过 300ms 的缓存窗口（取决于系统采样帧率），
  * 可用于缓解短时链路阻塞（例如蓝牙发送拥塞）带来的读写速率不匹配。
  */
#define RING_BUFFER_SIZE (32U)

/* Exported types ------------------------------------------------------------*/
/**
  * @brief 单路电桥 ADC 数据：分为“初期均值”和“末期均值”。
  */
typedef struct
{
  int32_t early_code; /**< 初期均值码值（有效位不超过 24 位） */
  int32_t late_code;  /**< 末期均值码值（有效位不超过 24 位） */
} ADC_ChannelData_t;

/**
  * @brief 多传感器同步总帧。
  *
  * 字段说明：
  * - adc_data[4]：4 组电桥 ADC 采样对（early/late）。
  * - ppg_data[3]：MAX30101 解析值（Green/Red/IR），已右移对齐。
  * - imu_data[6]：LSM9DS1 原始补码（Gx,Gy,Gz,Ax,Ay,Az）。
  */
typedef struct
{
  ADC_ChannelData_t adc_data[4]; /**< 4 组电桥 ADC 成对数据 */
  uint32_t ppg_data[3];          /**< MAX30101: Green, Red, IR（有效位不超过 24 位） */
  int16_t imu_data[6];           /**< LSM9DS1: Gx, Gy, Gz, Ax, Ay, Az */
} SensorDataFrame_t;

/**
  * @brief 环形缓冲区控制块。
  *
  * 注意：head/tail/count 使用 volatile，反映其会在不同执行上下文
  * （中断与主循环）被并发访问，避免编译器做不安全优化。
  */
typedef struct
{
  SensorDataFrame_t buffer[RING_BUFFER_SIZE]; /**< 固定容量帧存储区 */
  volatile uint16_t head;                     /**< 写索引：指向下一次写入位置 */
  volatile uint16_t tail;                     /**< 读索引：指向下一次读取位置 */
  volatile uint16_t count;                    /**< 当前有效帧数量 */
} SensorRingBuffer_t;

/* Exported functions prototypes ---------------------------------------------*/
void RingBuffer_Init(SensorRingBuffer_t *rb);
bool RingBuffer_Push(SensorRingBuffer_t *rb, const SensorDataFrame_t *frame);
bool RingBuffer_Pop(SensorRingBuffer_t *rb, SensorDataFrame_t *frame);
uint16_t RingBuffer_GetCount(SensorRingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_RINGBUFFER_H */
