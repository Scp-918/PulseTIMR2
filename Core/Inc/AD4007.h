#ifndef __AD4007_H__
#define __AD4007_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* ========================= 基础参数宏定义 ========================= */
/* AD4007 外部参考电压，单位 V。仅用于上层做码值到电压换算时参考。 */
#define AD4007_VREF                          (4.096f)

/* AD4007 配置寄存器访问命令
 * 按 AD4007 文档约定：
 * 0x14 = 写配置寄存器；0x54 = 读配置寄存器
 */
#define AD4007_CMD_WRITE_CONFIG              (0x14u)
#define AD4007_CMD_READ_CONFIG               (0x54u)

/* ========================= 配置位掩码定义 ========================= */
/* Bit4: Status 位输出使能（1: 在转换结果后附加状态位） */
#define AD4007_CFG_STATUS_EN_MASK            (1u << 4)
/* Bit3: Span Compression 量程压缩使能（1: 压缩到 ±0.8*VREF） */
#define AD4007_CFG_SPAN_COMP_MASK            (1u << 3)
/* Bit2: High-Z 模式使能（1: 输入高阻，减小输入反冲，低频高精度常用） */
#define AD4007_CFG_HIGH_Z_MASK               (1u << 2)
/* Bit1: Turbo 模式使能（1: 提高读取窗口，读到上一次转换数据） */
#define AD4007_CFG_TURBO_MASK                (1u << 1)
/* Bit0: 过压钳位标志位（只读粘滞位），软件写入时通常保持 0 */
#define AD4007_CFG_OV_CLAMP_FLAG_MASK        (1u << 0)

/* AD4007 默认初始化配置：
 * 关闭 Turbo、开启 High-Z、关闭 Span Compression
 */
#define AD4007_CFG_DEFAULT                   (AD4007_CFG_HIGH_Z_MASK)

/* 数据格式相关宏：24-bit 帧中高 18 位为 ADC 数据，低 6 位为状态/填充位 */
#define AD4007_FRAME_BYTES                   (3u)
#define AD4007_FRAME_RIGHT_SHIFT             (6u)
#define AD4007_CODE_MASK_18BIT               (0x3FFFFu)
#define AD4007_SIGN_BIT_18BIT                (0x20000u)
#define AD4007_SIGN_EXTEND_MASK              (0xFFFC0000u)

/* HRTIM ISR 内单次 LL 阻塞读取的总硬超时。 */
#define AD4007_BLOCKING_TIMEOUT_US            (5u)

/* LL 阻塞读取运行统计，供调试器只读观察。 */
typedef struct
{
    volatile uint32_t read_ok_count;
    volatile uint32_t read_busy_count;
    volatile uint32_t txe_timeout_count;
    volatile uint32_t rxne_timeout_count;
    volatile uint32_t bsy_timeout_count;
    volatile uint32_t spi_error_count;
} AD4007_RuntimeStats_t;

/* ========================= 对外接口声明 ========================= */

/*
 * @brief AD4007 初始化（核心初始化函数）
 * 1) 校验并确保 SPI3 为 CPOL=0, CPHA=0。
 * 2) 写默认配置：Turbo 关、High-Z 开、Span Compression 关。
 * 3) 执行一次上电后的空转换：软件拉高 CNV 等待 tCONV 后拉低并丢弃首帧数据。
 *
 * @retval HAL_OK      初始化成功
 * @retval HAL_ERROR   初始化失败
 */
HAL_StatusTypeDef AD4007_Init(void);

/*
 * @brief 上位机动态配置接口
 * @param enable_high_z      true: 开启 High-Z
 * @param enable_span_comp   true: 开启 Span Compression
 * @param enable_turbo       true: 开启 Turbo
 *
 * @note 内部通过发送 0x14 + cfg_byte 实时更新配置寄存器。
 */
HAL_StatusTypeDef AD4007_ConfigMode(bool enable_high_z, bool enable_span_comp, bool enable_turbo);

/*
 * @brief 使用 LL 寄存器内联接口阻塞读取一个 24-bit AD4007 帧。
 * @param out_code 输出解码并符号扩展后的 18-bit ADC 码值。
 *
 * @note 整笔事务由 DWT 周期计数器限制在 AD4007_BLOCKING_TIMEOUT_US 内，
 *       适合在 CNV 下降沿对应的 HRTIM ISR 中调用。
 */
HAL_StatusTypeDef AD4007_ReadBlocking_LL(int32_t *out_code);

/*
 * @brief 只对 valid_mask 标记有效的 slot 求平均。
 * @param slot_codes  六个原始 slot 码值。
 * @param valid_mask  bit0..bit5 分别对应 slot0..slot5。
 * @param first_slot  平均区间首 slot，范围 0..5。
 * @param slot_count  平均区间长度，且 first_slot+slot_count <= 6。
 * @param out_avg_code 输出有效 slot 的整型平均；无有效项时输出 0。
 */
HAL_StatusTypeDef AD4007_AverageValidSlots(const int32_t slot_codes[6],
                                           uint8_t valid_mask,
                                           uint8_t first_slot,
                                           uint8_t slot_count,
                                           int32_t *out_avg_code);

/* 返回 LL 阻塞读取的只读运行统计。 */
const AD4007_RuntimeStats_t *AD4007_GetRuntimeStats(void);

/*
 * 历史调试接口归档：
 * - 当前 HRTIM 主路径使用 AD4007_ReadBlocking_LL；原 DMA 接口继续保留；
 * - 单次手动 CNV 的测试接口保留在 #if 0 中，便于后续联调快速回滚。
 */
#if 0
HAL_StatusTypeDef AD4007_test_Rx(int32_t *out_code);
HAL_StatusTypeDef AD4007_test_DMA_Rx(int32_t *out_code, uint32_t timeout_ms);
#endif

/*
 * @brief 启动 SPI3 + DMA 异步接收
 * @param rx_buffer    DMA 接收缓存（长度至少为 sample_count*3 字节）
 * @param sample_count 采样点数（每点 3 字节）
 *
 * @note 本函数仅启动 DMA 接收，不翻转 CNV。正常工作下 CNV 由 HRTIM 硬件自动驱动。
 */
HAL_StatusTypeDef AD4007_Start_DMA_Rx(uint8_t *rx_buffer, uint16_t sample_count);

/*
 * @brief DMA 原始数据处理：解包 + 18 位补码符号扩展 + 整型均值
 * @param dma_buffer   原始 DMA 缓冲区（按 3 字节/点排列）
 * @param sample_count 样本数量
 * @param out_avg_code 输出平均码值（int32）
 *
 * @note 全程使用整数运算，不使用 float。
 */
HAL_StatusTypeDef AD4007_ProcessRawData(uint8_t *dma_buffer, uint16_t sample_count, int32_t *out_avg_code);

/*
 * @brief 读取结束后强制 MOSI 拉高
 *
 * @note AD4007 在下一次 CNV 采样 SDI=1 可保持 CS 模式。
 *       该函数会临时把 PC12(MOSI) 切为 GPIO 输出高，再恢复 AF6_SPI3。
 */
void AD4007_ForceMOSIHigh(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD4007_H__ */
