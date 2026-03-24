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

/*
 * [单帧长度] 固定 49 字节。
 * 帧结构严格对应：
 * - [0..1]   : 帧头
 * - [2..25]  : 4通道 ADC，每通道 early/late 各 3 字节，共 24 字节
 * - [26..34] : PPG 三路（G/R/IR），每路 3 字节，共 9 字节
 * - [35..46] : MIMU 六轴，每轴 2 字节，共 12 字节
 * - [47]     : XOR 校验（不含帧头）
 * - [48]     : 帧尾 0xCC
 */
#define BLE_COMM_SINGLE_FRAME_SIZE          49U

/* [批量帧数] 每次 DMA 发送 10 帧。 */
#define BLE_COMM_BATCH_FRAME_COUNT          10U

/* [批量发送字节数] 49 * 10 = 490 字节。 */
#define BLE_COMM_BATCH_TX_SIZE              (BLE_COMM_SINGLE_FRAME_SIZE * BLE_COMM_BATCH_FRAME_COUNT)

/* [帧头/帧尾] */
#define BLE_COMM_FRAME_HEADER_BYTE0         0xAAU
#define BLE_COMM_FRAME_HEADER_BYTE1         0xBBU
#define BLE_COMM_FRAME_TAIL_BYTE0           0xCCU

/* [字段索引定义] 便于校验边界与调试定位。 */
#define BLE_COMM_IDX_HEADER0                0U
#define BLE_COMM_IDX_HEADER1                1U

#define BLE_COMM_IDX_ADC_START              2U
#define BLE_COMM_ADC_CHANNEL_COUNT          4U
#define BLE_COMM_ADC_VALUES_PER_CHANNEL     2U
#define BLE_COMM_ADC_BYTES_PER_VALUE        3U
#define BLE_COMM_ADC_BYTES_PER_CHANNEL      (BLE_COMM_ADC_VALUES_PER_CHANNEL * BLE_COMM_ADC_BYTES_PER_VALUE)
#define BLE_COMM_IDX_ADC_END                25U

#define BLE_COMM_IDX_PPG_START              26U
#define BLE_COMM_PPG_CHANNEL_COUNT          3U
#define BLE_COMM_PPG_BYTES_PER_CHANNEL      3U
#define BLE_COMM_IDX_PPG_END                34U

#define BLE_COMM_IDX_IMU_START              35U
#define BLE_COMM_IMU_AXIS_COUNT             6U
#define BLE_COMM_IMU_BYTES_PER_AXIS         2U
#define BLE_COMM_IDX_IMU_END                46U

#define BLE_COMM_IDX_CHECKSUM               47U
#define BLE_COMM_IDX_TAIL0                  48U

/* [校验区间] 按协议要求：仅对 [2..46] 异或，帧头不参与。 */
#define BLE_COMM_XOR_START_IDX              BLE_COMM_IDX_ADC_START
#define BLE_COMM_XOR_END_IDX                BLE_COMM_IDX_IMU_END

/* ===========================
 * 接口声明
 * =========================== */

/*
 * @brief 将一帧传感器数据按协议打包为 49 字节。
 * @param frame      输入传感器融合帧。
 * @param out_buffer 输出缓存，长度至少为 BLE_COMM_SINGLE_FRAME_SIZE。
 *
 * @note
 * 1) ADC 区按 4 通道循环展开：ch0~ch3，每通道 early 后接 late。
 * 2) PPG 区按 Green/Red/IR 顺序写入 3 字节低位。
 * 3) MIMU 区按 Gx/Gy/Gz/Ax/Ay/Az 顺序写入 2 字节小端。
 * 4) 24-bit 提取采用显式位掩码 + 右移，不使用指针强转。
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
