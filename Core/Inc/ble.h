#ifndef __BLE_H__
#define __BLE_H__

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================
 * HJ131IMH 模块基础配置宏
 * =========================== */

/* [目标波特率] 初始化成功后，MCU 与蓝牙模块都工作在该速率，用于高频数据链路。 */
#define BLE_TARGET_BAUD                 115200U

/* [默认波特率] 模块出厂 UART 波特率。 */
#define BLE_DEFAULT_BAUD                19200U

/* [备用波特率] 用于容错重试：当模块曾被改过波特率但未恢复出厂时尝试该速率。 */
#define BLE_PREV_BAUD                   460800U

/* [唤醒字节个数] 发送连续 0xAA 的次数。 */
#define BLE_WAKE_BYTES_COUNT            5U

/* [阻塞初始化超时] 初始化阶段允许阻塞式发送/接收，超时用于判定步骤失败。 */
#define BLE_INIT_UART_TX_TIMEOUT_MS     100U
#define BLE_INIT_UART_RX_TIMEOUT_MS     200U

/* [复位时序参数] RST 为高电平有效，维持 10ms 后释放，再等待模块启动。 */
#define BLE_RST_PULSE_MS                10U
#define BLE_BOOT_WAIT_MS                600U

/* [指令处理等待] 唤醒后与改波特率后，按手册留出模块内部处理时间。 */
#define BLE_WAKE_SETTLE_MS              20U
#define BLE_BAUD_APPLY_MS               50U
#define BLE_WAKE_FOREVER_PRE_DELAY_MS   5U

/* [命令字符串] 透传模块采用 <...> 指令格式。 */
#define BLE_CMD_SET_BAUD_FMT            "<ST_BAUD=%lu>"
#define BLE_CMD_SET_WAKE_FOREVER        "<ST_WAKE=FOREVER>"
#define BLE_CMD_SET_TX_POWER_MAX        "<ST_TX_POWER=+2.5>"

/* [关键应答关键字] 只做子串匹配，兼容可能包含前后缀的应答帧。 */
#define BLE_RESP_OK                     "ok"

/* [硬件引脚定义] 与原理图一致：RST=PE1, STATE=PE0。 */
#define BLE_RST_PORT                    GPIOE
#define BLE_RST_PIN                     GPIO_PIN_1
#define BLE_STATE_PORT                  GPIOE
#define BLE_STATE_PIN                   GPIO_PIN_0

/* [复位电平语义] RST 高电平有效。 */
#define BLE_RST_ACTIVE_LEVEL            GPIO_PIN_SET
#define BLE_RST_INACTIVE_LEVEL          GPIO_PIN_RESET

/* [连接状态语义] 读取 STATE 引脚判断连接状态。 */
#define BLE_STATE_CONNECTED             1U
#define BLE_STATE_DISCONNECTED          0U

/* [接收命令缓存上限] 回调中保存最近一条指令，便于调试/上层读取。 */
#define BLE_LAST_CMD_MAX_LEN            128U

/* [示例参数边界] 用于演示 SET_PULSE_T=... 的解析与参数保护。 */
#define BLE_PULSE_T_MIN_US              100U
#define BLE_PULSE_T_MAX_US              1000000U

/* ===========================
 * 全局变量与接口声明
 * =========================== */

/* USART1 句柄由 CubeMX/usart.c 提供，按系统约定在此使用 extern。 */
extern UART_HandleTypeDef huart1;

/*
 * 示例动态参数：
 * 回调模板中会解析 "SET_PULSE_T=xxxx" 并更新该变量。
 * 你的主循环/控制环可以直接读取该值，实现上位机动态调参。
 */
extern volatile uint32_t g_ble_pulse_t_us;

/*
 * @brief BLE 初始化入口。
 * @note  严格按手册执行：硬件复位 -> 19200 唤醒+改波特率 -> 115200 容错重试 ->
 *        MCU 切换 460800 -> 再唤醒并发送 <ST_WAKE=FOREVER>。
 * @retval HAL_OK       初始化流程关键步骤完成。
 * @retval HAL_TIMEOUT  命令应答超时。
 * @retval HAL_ERROR    串口重配置或发送失败。
 */
HAL_StatusTypeDef BLE_Init(void);

/*
 * @brief 100Hz 数据链路使用的 DMA 非阻塞发送接口。
 * @note  入口先检查 UART 状态，若发送忙返回 HAL_BUSY 并计数，避免覆盖缓冲区。
 * @param data 发送数据指针。
 * @param len  发送字节数。
 * @retval HAL_OK/HAL_BUSY/HAL_ERROR。
 */
HAL_StatusTypeDef BLE_Transmit_Data_DMA(uint8_t *data, uint16_t len);

/*
 * @brief 启动 IDLE + DMA 不定长接收，用于持续监听上位机参数指令。
 * @param rx_buffer 接收缓存。
 * @param max_len   缓存最大长度。
 * @retval HAL_OK/HAL_ERROR。
 */
HAL_StatusTypeDef BLE_Start_Receive_DMA(uint8_t *rx_buffer, uint16_t max_len);

/* @brief 获取 DMA 发送忙导致的丢帧计数。 */
uint32_t BLE_Get_TxBusyDropCount(void);

/* @brief 获取最近一次解析到的命令字符串。 */
const char *BLE_Get_LastRxCommand(void);

/* @brief 读取 BLE 连接状态引脚。 */
uint8_t BLE_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_H__ */