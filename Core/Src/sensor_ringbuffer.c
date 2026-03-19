/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_ringbuffer.c
  * @brief          : 多传感器同步数据环形缓冲区实现
  ******************************************************************************
  * @attention
  *
  * 并发模型：
  * - 生产者：中断/DMA 回调上下文调用 RingBuffer_Push()
  * - 消费者：主循环调用 RingBuffer_Pop()
  *
  * 满载策略：
  * - 当缓冲区已满时，主动丢弃最老数据（tail 前移），再写入最新帧。
  * - 这样可保证系统尽量输出“最近时刻”的传感信息，适合实时监测场景。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sensor_ringbuffer.h"

#include "stm32g4xx_hal.h"
#include <string.h>

/**
  * @brief  环形缓冲区初始化。
  * @param  rb: 环形缓冲区对象指针
  * @retval None
  */
void RingBuffer_Init(SensorRingBuffer_t *rb)
{
  if (rb == NULL)
  {
    return;
  }

  /*
   * 统一清零，确保首次使用前数据区与索引均处于已知状态。
   * 这里不需要关中断：通常在系统初始化阶段调用。
   */
  (void)memset(rb, 0, sizeof(SensorRingBuffer_t));
}

/**
  * @brief  入队一帧数据（中断上下文友好）。
  * @param  rb:    环形缓冲区对象指针
  * @param  frame: 待写入帧指针
  * @retval true: 写入成功
  * @retval false: 参数非法，未写入
  */
bool RingBuffer_Push(SensorRingBuffer_t *rb, const SensorDataFrame_t *frame)
{
  uint16_t next_head;

  if ((rb == NULL) || (frame == NULL))
  {
    return false;
  }

  /*
   * 写入当前 head 位置。
   * 由于 frame 为结构体，编译器会生成定长拷贝；数据布局固定、可预测。
   */
  rb->buffer[rb->head] = *frame;

  next_head = (uint16_t)(rb->head + 1U);
  if (next_head >= RING_BUFFER_SIZE)
  {
    next_head = 0U;
  }

  if (rb->count >= RING_BUFFER_SIZE)
  {
    /*
     * 满载：丢弃最老数据（tail 前移），count 维持满值。
     * 该策略保证“最新帧优先”。
     */
    uint16_t next_tail = (uint16_t)(rb->tail + 1U);
    if (next_tail >= RING_BUFFER_SIZE)
    {
      next_tail = 0U;
    }
    rb->tail = next_tail;
    rb->count = RING_BUFFER_SIZE;
  }
  else
  {
    rb->count++;
  }

  rb->head = next_head;

  return true;
}

/**
  * @brief  出队一帧数据（主循环调用）。
  * @param  rb:    环形缓冲区对象指针
  * @param  frame: 接收出队数据的目标指针
  * @retval true: 读出成功
  * @retval false: 队列为空或参数非法
  */
bool RingBuffer_Pop(SensorRingBuffer_t *rb, SensorDataFrame_t *frame)
{
  uint16_t local_tail;
  uint16_t next_tail;

  if ((rb == NULL) || (frame == NULL))
  {
    return false;
  }

  /*
   * 极短临界区保护出队关键步骤：
   * 1) 判空
   * 2) 读取当前 tail 对应帧
   * 3) 更新 tail/count
   *
   * 防止在主循环出队过程中被中断生产者打断，导致索引/计数竞争。
   */
  __disable_irq();

  if (rb->count == 0U)
  {
    __enable_irq();
    return false;
  }

  local_tail = rb->tail;
  *frame = rb->buffer[local_tail];

  next_tail = (uint16_t)(local_tail + 1U);
  if (next_tail >= RING_BUFFER_SIZE)
  {
    next_tail = 0U;
  }

  rb->tail = next_tail;
  rb->count--;

  __enable_irq();

  return true;
}

/**
  * @brief  获取当前未读帧数。
  * @param  rb: 环形缓冲区对象指针
  * @retval 当前队列长度；参数非法时返回 0
  */
uint16_t RingBuffer_GetCount(SensorRingBuffer_t *rb)
{
  uint16_t count_snapshot;

  if (rb == NULL)
  {
    return 0U;
  }

  /*
   * 读取 count 使用短临界区，避免与 ISR 修改并发时获得不一致快照。
   */
  __disable_irq();
  count_snapshot = rb->count;
  __enable_irq();

  return count_snapshot;
}
