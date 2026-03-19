#ifndef __BLE_COMM_H__
#define __BLE_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "sensor_ringbuffer.h"

/* ===========================
 * 蓝牙数据链路协议定义（固定帧）
 * =========================== */

/* [单帧长度] 固定 32 字节。 */
#define BLE_COMM_SINGLE_FRAME_SIZE          32U

/* [批量帧数] 每次 DMA 发送 10 帧。 */
#define BLE_COMM_BATCH_FRAME_COUNT          10U

/* [批量发送字节数] 32 * 10 = 320 字节。 */
#define BLE_COMM_BATCH_TX_SIZE              (BLE_COMM_SINGLE_FRAME_SIZE * BLE_COMM_BATCH_FRAME_COUNT)

/* [帧头/帧尾] */
#define BLE_COMM_FRAME_HEADER_BYTE0         0xAAU
#define BLE_COMM_FRAME_HEADER_BYTE1         0xBBU
#define BLE_COMM_FRAME_TAIL_BYTE0           0xCCU
#define BLE_COMM_FRAME_TAIL_BYTE1           0xDDU

/* [字段索引定义] 便于校验边界与调试定位。 */
#define BLE_COMM_IDX_HEADER0                0U
#define BLE_COMM_IDX_HEADER1                1U

#define BLE_COMM_IDX_ADC_EARLY_L            2U
#define BLE_COMM_IDX_ADC_EARLY_M            3U
#define BLE_COMM_IDX_ADC_EARLY_H            4U

#define BLE_COMM_IDX_ADC_LATE_L             5U
#define BLE_COMM_IDX_ADC_LATE_M             6U
#define BLE_COMM_IDX_ADC_LATE_H             7U

#define BLE_COMM_IDX_PPG_START              8U
#define BLE_COMM_IDX_PPG_END                16U

#define BLE_COMM_IDX_IMU_START              17U
#define BLE_COMM_IDX_IMU_END                28U

#define BLE_COMM_IDX_CHECKSUM               29U
#define BLE_COMM_IDX_TAIL0                  30U
#define BLE_COMM_IDX_TAIL1                  31U

/* [校验区间] 按协议要求：仅对 [2..28] 异或，帧头不参与。 */
#define BLE_COMM_XOR_START_IDX              BLE_COMM_IDX_ADC_EARLY_L
#define BLE_COMM_XOR_END_IDX                BLE_COMM_IDX_IMU_END

/* ===========================
 * 接口声明
 * =========================== */

/*
 * @brief 将一帧传感器数据按协议打包为 32 字节。
 * @param frame      输入传感器融合帧。
 * @param out_buffer 输出缓存，长度至少为 BLE_COMM_SINGLE_FRAME_SIZE。
 *
 * @note
 * 1) ADC 采用 adc_data[0] 的 early/late 作为协议中的两路 24-bit 数据源。
 * 2) 24-bit 提取采用显式位掩码 + 右移，不使用指针强转。
 */
void BLE_PackSingleFrame(SensorDataFrame_t *frame, uint8_t *out_buffer);

/*
 * @brief 蓝牙发送任务：每次调用最多消费 1 帧并聚合到 10 帧批次后 DMA 发送。
 * @param rb 传感器帧环形缓冲区。
 * @retval true  本次调用成功消费了 1 帧（无论是否触发发送）。
 * @retval false 本次无帧可消费或参数非法。
 */
bool BLE_Task_Process(SensorRingBuffer_t *rb);

/* @brief 获取 "UART忙导致整批丢弃" 的计数。 */
uint32_t BLE_Comm_GetBatchDropBusyCount(void);

/* @brief 获取 "DMA调用失败" 的计数。 */
uint32_t BLE_Comm_GetBatchSendFailCount(void);

/* @brief 获取当前批次已累计帧数（0..9）。 */
uint8_t BLE_Comm_GetPendingFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_COMM_H__ */
