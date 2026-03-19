#include "ble.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USART1 句柄由 usart.c 提供；此处再次声明用于强调依赖。 */
extern UART_HandleTypeDef huart1;

/* 示例动态参数：上位机命令 "SET_PULSE_T=xxxx" 将更新该值。 */
volatile uint32_t g_ble_pulse_t_us = 10000U;

/* 模块内部状态 */
static uint32_t s_ble_tx_dma_drop_count = 0U;
static uint8_t *s_ble_rx_dma_buffer = NULL;
static uint16_t s_ble_rx_dma_max_len = 0U;
static char s_ble_last_rx_command[BLE_LAST_CMD_MAX_LEN] = {0};

/* 初始化阶段临时接收缓冲：用于阻塞式等待应答。 */
static uint8_t s_ble_init_rx_buffer[BLE_LAST_CMD_MAX_LEN] = {0};

/* 固定唤醒序列：连续 5 个 0xAA。 */
static const uint8_t s_ble_wake_bytes[BLE_WAKE_BYTES_COUNT] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

/*
 * @brief 初始化 BLE 专用 GPIO：RST(PE1) 与 STATE(PE0)。
 */
static void BLE_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* RST 输出：默认保持非复位电平，避免上电瞬间反复复位。 */
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, BLE_RST_INACTIVE_LEVEL);
  gpio_init.Pin = BLE_RST_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_RST_PORT, &gpio_init);

  /* STATE 输入：读取连接状态，0=未连接，1=已连接。 */
  gpio_init.Pin = BLE_STATE_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_STATE_PORT, &gpio_init);
}

/*
 * @brief 重新配置 USART1 波特率。
 * @note  切换波特率前先 DeInit，再按原始格式参数重新 Init，避免寄存器遗留状态。
 */
static HAL_StatusTypeDef BLE_Reconfigure_UartBaud(uint32_t baudrate)
{
  HAL_StatusTypeDef ret;

  /* 防止重配期间存在活动 DMA/中断传输导致状态异常。 */
  (void)HAL_UART_Abort(&huart1);

  ret = HAL_UART_DeInit(&huart1);
  if (ret != HAL_OK)
  {
    return ret;
  }

  huart1.Init.BaudRate = baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;

  ret = HAL_UART_Init(&huart1);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return HAL_OK;
}

/*
 * @brief 发送固定唤醒序列。
 * @note  初始化阶段允许阻塞，明确保证模块 RX 从休眠态进入可接收指令态。
 */
static HAL_StatusTypeDef BLE_SendWakeBytes_Blocking(void)
{
  return HAL_UART_Transmit(&huart1,
               (uint8_t *)s_ble_wake_bytes,
               (uint16_t)BLE_WAKE_BYTES_COUNT,
               BLE_INIT_UART_TX_TIMEOUT_MS);
}

/*
 * @brief 阻塞接收并检查是否包含指定应答子串。
 * @param expected_resp 期望应答关键字，例如 "ok"。
 * @param timeout_ms    总超时。
 * @retval HAL_OK       匹配成功。
 * @retval HAL_TIMEOUT  超时未匹配。
 * @retval HAL_ERROR    接收过程异常。
 */
static HAL_StatusTypeDef BLE_WaitResponseContains(const char *expected_resp, uint32_t timeout_ms)
{
  uint16_t idx = 0U;
  uint8_t ch = 0U;
  uint32_t start_tick = HAL_GetTick();

  if (expected_resp == NULL)
  {
    return HAL_ERROR;
  }

  memset(s_ble_init_rx_buffer, 0, sizeof(s_ble_init_rx_buffer));

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    uint32_t elapsed = HAL_GetTick() - start_tick;
    uint32_t remain = timeout_ms - elapsed;
    HAL_StatusTypeDef rx_ret = HAL_UART_Receive(&huart1, &ch, 1U, remain);

    if (rx_ret == HAL_OK)
    {
      if (idx < (sizeof(s_ble_init_rx_buffer) - 1U))
      {
        s_ble_init_rx_buffer[idx++] = ch;
        s_ble_init_rx_buffer[idx] = '\0';
      }

      if (strstr((char *)s_ble_init_rx_buffer, expected_resp) != NULL)
      {
        return HAL_OK;
      }
    }
    else if (rx_ret == HAL_TIMEOUT)
    {
      /* 单字节等待超时属于常态，继续累计等到总超时。 */
    }
    else
    {
      return HAL_ERROR;
    }
  }

  return HAL_TIMEOUT;
}

/*
 * @brief 在指定当前波特率下执行一次 "唤醒 + 设置目标波特率 + 等待 ok"。
 * @param current_baud 当前 MCU 侧 UART 波特率（19200 或 115200）。
 */
static HAL_StatusTypeDef BLE_TrySyncToTargetBaud(uint32_t current_baud)
{
  HAL_StatusTypeDef ret;
  char cmd_set_baud[32] = {0};
  int cmd_len;

  ret = BLE_Reconfigure_UartBaud(current_baud);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BLE_SendWakeBytes_Blocking();
  if (ret != HAL_OK)
  {
    return ret;
  }
  HAL_Delay(BLE_WAKE_SETTLE_MS);

  cmd_len = snprintf(cmd_set_baud, sizeof(cmd_set_baud), BLE_CMD_SET_BAUD_FMT, BLE_TARGET_BAUD);
  if ((cmd_len <= 0) || ((uint32_t)cmd_len >= sizeof(cmd_set_baud)))
  {
    return HAL_ERROR;
  }

  ret = HAL_UART_Transmit(&huart1,
              (uint8_t *)cmd_set_baud,
              (uint16_t)cmd_len,
              BLE_INIT_UART_TX_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BLE_WaitResponseContains(BLE_RESP_OK, BLE_INIT_UART_RX_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  HAL_Delay(BLE_BAUD_APPLY_MS);
  return HAL_OK;
}

HAL_StatusTypeDef BLE_Init(void)
{
  HAL_StatusTypeDef ret;

  /* 步骤 0：先保证 RST/STATE 管脚处于已配置状态。 */
  BLE_GPIO_Init();

  /*
   * 步骤 1：硬件复位模块。
   * 手册要求 RST 高电平有效，维持 >1ms；这里用 10ms 增强鲁棒性。
   */
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, BLE_RST_ACTIVE_LEVEL);
  HAL_Delay(BLE_RST_PULSE_MS);
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, BLE_RST_INACTIVE_LEVEL);
  HAL_Delay(BLE_BOOT_WAIT_MS);

  /*
   * 步骤 2：按手册执行 "唤醒与波特率同步" 状态机。
   * 2.1 先按出厂 19200 尝试；失败则用 115200 再试一次。
   */
  ret = BLE_TrySyncToTargetBaud(BLE_DEFAULT_BAUD);
  if (ret != HAL_OK)
  {
    ret = BLE_TrySyncToTargetBaud(BLE_PREV_BAUD);
    if (ret != HAL_OK)
    {
      return ret;
    }
  }

  /* 步骤 3：MCU 本地 UART 切换到目标 460800，进入全速传输配置。 */
  ret = BLE_Reconfigure_UartBaud(BLE_TARGET_BAUD);
  if (ret != HAL_OK)
  {
    return ret;
  }
  HAL_Delay(BLE_BAUD_APPLY_MS);

  /*
   * 步骤 4：再次发送唤醒字节，随后下发 <ST_WAKE=FOREVER>。
   * 目的：确保模块 RX 在当前波特率下已激活，并锁定全速工作模式。
   */
  ret = BLE_SendWakeBytes_Blocking();
  if (ret != HAL_OK)
  {
    return ret;
  }
  HAL_Delay(BLE_WAKE_FOREVER_PRE_DELAY_MS);

  ret = HAL_UART_Transmit(&huart1,
              (uint8_t *)BLE_CMD_SET_WAKE_FOREVER,
              (uint16_t)strlen(BLE_CMD_SET_WAKE_FOREVER),
              BLE_INIT_UART_TX_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BLE_WaitResponseContains(BLE_RESP_OK, BLE_INIT_UART_RX_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return HAL_OK;
}

HAL_StatusTypeDef BLE_Transmit_Data_DMA(uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  /*
   * 高频发送保护：
   * 100Hz 定时触发下，如果上一次 DMA 还在发，绝不能覆盖同一发送缓冲区。
   * 因此这里直接返回 HAL_BUSY，并累积丢帧计数供上层监控。
   */
  if (huart1.gState != HAL_UART_STATE_READY)
  {
    s_ble_tx_dma_drop_count++;
    return HAL_BUSY;
  }

  return HAL_UART_Transmit_DMA(&huart1, data, len);
}

HAL_StatusTypeDef BLE_Start_Receive_DMA(uint8_t *rx_buffer, uint16_t max_len)
{
  HAL_StatusTypeDef ret;

  if ((rx_buffer == NULL) || (max_len == 0U))
  {
    return HAL_ERROR;
  }

  s_ble_rx_dma_buffer = rx_buffer;
  s_ble_rx_dma_max_len = max_len;

  ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_ble_rx_dma_buffer, s_ble_rx_dma_max_len);
  if (ret != HAL_OK)
  {
    return ret;
  }

  /*
   * 可选优化：关闭半传输中断，避免收到半包就进入回调。
   * 这里我们只关心 IDLE 或缓存满事件。
   */
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  return HAL_OK;
}

uint32_t BLE_Get_TxBusyDropCount(void)
{
  return s_ble_tx_dma_drop_count;
}

const char *BLE_Get_LastRxCommand(void)
{
  return s_ble_last_rx_command;
}

uint8_t BLE_IsConnected(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(BLE_STATE_PORT, BLE_STATE_PIN);
  return (pin_state == GPIO_PIN_SET) ? BLE_STATE_CONNECTED : BLE_STATE_DISCONNECTED;
}

/*
 * @brief UART IDLE + DMA 接收事件回调模板。
 *
 * 说明：
 * 1) 该函数是 HAL 提供的弱符号回调，这里给出 BLE 场景下的完整可用模板。
 * 2) 回调里只做轻量解析，避免耗时操作阻塞中断上下文。
 * 3) 每次处理完都要立刻重启 HAL_UARTEx_ReceiveToIdle_DMA，保证持续监听上位机命令。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t copy_len;

  if (huart != &huart1)
  {
    return;
  }

  if ((s_ble_rx_dma_buffer == NULL) || (s_ble_rx_dma_max_len == 0U))
  {
    return;
  }

  /* 限幅拷贝，构造 C 字符串。 */
  copy_len = Size;
  if (copy_len > (uint16_t)(BLE_LAST_CMD_MAX_LEN - 1U))
  {
    copy_len = (uint16_t)(BLE_LAST_CMD_MAX_LEN - 1U);
  }

  memcpy(s_ble_last_rx_command, s_ble_rx_dma_buffer, copy_len);
  s_ble_last_rx_command[copy_len] = '\0';

  /* 去掉尾部回车换行，便于字符串匹配。 */
  while (copy_len > 0U)
  {
    char tail = s_ble_last_rx_command[copy_len - 1U];
    if ((tail == '\r') || (tail == '\n'))
    {
      s_ble_last_rx_command[copy_len - 1U] = '\0';
      copy_len--;
    }
    else
    {
      break;
    }
  }

  /*
   * 指令解析示例：SET_PULSE_T=10000
   * 可在此继续扩展其它参数命令，例如 SET_GAIN=... / SET_RATE=...。
   */
  if (strncmp(s_ble_last_rx_command, "SET_PULSE_T=", 12U) == 0)
  {
    uint32_t new_pulse_t = (uint32_t)strtoul(&s_ble_last_rx_command[12], NULL, 10);

    /* 参数边界保护，避免异常值破坏控制环时序。 */
    if ((new_pulse_t >= BLE_PULSE_T_MIN_US) && (new_pulse_t <= BLE_PULSE_T_MAX_US))
    {
      g_ble_pulse_t_us = new_pulse_t;
    }
  }

  /* 重新启动下一轮不定长接收，保持持续监听。 */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_ble_rx_dma_buffer, s_ble_rx_dma_max_len);
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}