#ifndef __LSM9DS1_H
#define __LSM9DS1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensor_ringbuffer.h"

/*
 * LSM9DS1 A/G 片区寄存器地址定义
 * 说明：本驱动仅使用 A/G 相关寄存器（不涉及磁力计片区）。
 */
#define LSM9DS1_REG_INT1_CTRL           0x0CU
#define LSM9DS1_REG_WHO_AM_I            0x0FU
#define LSM9DS1_REG_CTRL_REG1_G         0x10U
#define LSM9DS1_REG_CTRL_REG3_G         0x12U
#define LSM9DS1_REG_CTRL_REG6_XL        0x20U
#define LSM9DS1_REG_CTRL_REG7_XL        0x21U
#define LSM9DS1_REG_CTRL_REG8           0x22U
#define LSM9DS1_REG_CTRL_REG9           0x23U
#define LSM9DS1_REG_FIFO_CTRL           0x2EU

/* SPI 命令/位定义 */
#define LSM9DS1_SPI_RW_READ_BIT          0x80U /* bit7=1: 读寄存器 */
#define LSM9DS1_SPI_RW_WRITE_BIT         0x00U /* bit7=0: 写寄存器 */
#define LSM9DS1_SPI_REG_ADDR_MASK        0x7FU /* 写命令需确保 bit7=0 */

/* WHO_AM_I 常量 */
#define LSM9DS1_BURST_READ_CMD          0x98U
#define LSM9DS1_WHO_AM_I_VALUE          0x68U

/*
 * 关键位掩码（便于后续扩展按位改写）
 * CTRL_REG8 (0x22)
 */
#define LSM9DS1_CTRL_REG8_BDU_BIT       0x40U /* 防撕裂：高低字节完整读取后再更新 */
#define LSM9DS1_CTRL_REG8_IF_ADD_INC    0x04U /* 地址自动递增 */
#define LSM9DS1_CTRL_REG8_SW_RESET      0x01U /* 软件复位触发位 */

/* CTRL_REG9 (0x23) */
#define LSM9DS1_CTRL_REG9_FIFO_EN       0x02U /* FIFO 使能位（本方案应关闭） */

/* FIFO_CTRL (0x2E) */
#define LSM9DS1_FIFO_CTRL_FMODE_MASK    0xE0U /* FIFO 模式位域 [7:5] */
#define LSM9DS1_FIFO_CTRL_BYPASS        0x00U /* Bypass 模式：彻底旁路 FIFO */

/* CTRL_REG1_G / CTRL_REG6_XL 的量程位掩码 */
#define LSM9DS1_FS_FIELD_MASK           0x18U /* [4:3] */

/*
 * 初始化目标寄存器值（严格对应文档）
 * - 0x05: 触发 SW_RESET + IF_ADD_INC
 * - 0x44: BDU + IF_ADD_INC
 */
#define LSM9DS1_INIT_CTRL_REG8_RESET    0x05U
#define LSM9DS1_INIT_CTRL_REG8_NORMAL   0x44U
#define LSM9DS1_INIT_CTRL_REG9          0x00U
#define LSM9DS1_INIT_FIFO_CTRL          0x00U
#define LSM9DS1_INIT_INT1_CTRL          0x00U
#define LSM9DS1_INIT_CTRL_REG1_G        0x68U
#define LSM9DS1_INIT_CTRL_REG3_G        0x46U
/* 0x60: ODR_XL=119Hz, FS_XL=+-2g */
#define LSM9DS1_INIT_CTRL_REG6_XL       0x60U
#define LSM9DS1_INIT_CTRL_REG7_XL       0xC4U

/* 阻塞读写默认超时（毫秒） */
#define LSM9DS1_SPI_BLOCKING_TIMEOUT_MS 20U

/*
 * 量程输入参数（uint8_t 选择码语义）：
 * - accel_fs 取值：0/1/2/3 分别对应 ±2g/±4g/±8g/±16g
 * - gyro_fs  取值：0/1/2 分别对应 ±245dps/±500dps/±2000dps
 */
#define LSM9DS1_ACCEL_FS_2G             0U
#define LSM9DS1_ACCEL_FS_4G             1U
#define LSM9DS1_ACCEL_FS_8G             2U
#define LSM9DS1_ACCEL_FS_16G            3U

#define LSM9DS1_GYRO_FS_245DPS          0U
#define LSM9DS1_GYRO_FS_500DPS          1U
#define LSM9DS1_GYRO_FS_2000DPS         2U

/* 对应寄存器 FS 位编码（写入 [4:3]） */
#define LSM9DS1_ACCEL_FS_BITS_2G        0x00U
#define LSM9DS1_ACCEL_FS_BITS_4G        0x02U
#define LSM9DS1_ACCEL_FS_BITS_8G        0x03U
#define LSM9DS1_ACCEL_FS_BITS_16G       0x01U

#define LSM9DS1_GYRO_FS_BITS_245DPS     0x00U
#define LSM9DS1_GYRO_FS_BITS_500DPS     0x01U
#define LSM9DS1_GYRO_FS_BITS_2000DPS    0x03U

/* 灵敏度（单位：mg/LSB 与 mdps/LSB） */
#define LSM9DS1_ACCEL_SENS_2G_MG_LSB    0.061f
#define LSM9DS1_ACCEL_SENS_4G_MG_LSB    0.122f
#define LSM9DS1_ACCEL_SENS_8G_MG_LSB    0.244f
#define LSM9DS1_ACCEL_SENS_16G_MG_LSB   0.732f

#define LSM9DS1_GYRO_SENS_245_MDPS_LSB  8.75f
#define LSM9DS1_GYRO_SENS_500_MDPS_LSB  17.50f
#define LSM9DS1_GYRO_SENS_2000_MDPS_LSB 70.0f

/* SPI 全双工 DMA 长度：1字节命令 + 12字节数据 */
#define LSM9DS1_SPI_DMA_FRAME_LEN       13U
#define LSM9DS1_SPI_PAYLOAD_BYTES        12U

/* DMA 接收缓冲区中各字段偏移（rx[0] 为命令期回传废位） */
#define LSM9DS1_RX_DISCARD_INDEX         0U
#define LSM9DS1_RX_GX_L_INDEX            1U
#define LSM9DS1_RX_GX_H_INDEX            2U
#define LSM9DS1_RX_GY_L_INDEX            3U
#define LSM9DS1_RX_GY_H_INDEX            4U
#define LSM9DS1_RX_GZ_L_INDEX            5U
#define LSM9DS1_RX_GZ_H_INDEX            6U
#define LSM9DS1_RX_AX_L_INDEX            7U
#define LSM9DS1_RX_AX_H_INDEX            8U
#define LSM9DS1_RX_AY_L_INDEX            9U
#define LSM9DS1_RX_AY_H_INDEX            10U
#define LSM9DS1_RX_AZ_L_INDEX            11U
#define LSM9DS1_RX_AZ_H_INDEX            12U

typedef struct
{
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
} LSM9DS1_RawData_t;

/* 运行时状态快照，用于调试 DMA 采样链路是否稳定 */
typedef struct
{
    volatile uint8_t dma_busy;            /* 1=当前 DMA 事务进行中 */
    volatile uint32_t dma_ok_count;       /* DMA 完成次数 */
    volatile uint32_t dma_error_count;    /* DMA 错误次数 */
    volatile uint32_t trigger_busy_count; /* 触发时因 busy 被拒绝次数 */
} LSM9DS1_RuntimeState_t;

/* 1) 核心初始化：GPIO、通信校验、Bypass/FIFO/中断配置、基础 ODR 与滤波 */
HAL_StatusTypeDef LSM9DS1_Init(void);

/* 绑定外部中转帧：DMA 完成后会直接覆盖 imu_data[0..5] */
void LSM9DS1_AttachFrameBuffer(SensorDataFrame_t *frame);

/* 绑定外部环形缓冲区：DMA 完成后会额外把 imu_data 入队。 */
void LSM9DS1_AttachRingBuffer(SensorRingBuffer_t *rb);

/* 2) 外部 10ms 主定时器触发入口：发起 13 字节 SPI DMA 读取 */
HAL_StatusTypeDef LSM9DS1_TriggerRead_IT(void);

/* 3) 回调分发处理：可在用户集中回调中调用 */
void LSM9DS1_SPI_TxRxCpltHandler(SPI_HandleTypeDef *hspi);
void LSM9DS1_SPI_ErrorHandler(SPI_HandleTypeDef *hspi);

/* 4) 上位机动态配置量程，并同步更新灵敏度系数 */
HAL_StatusTypeDef LSM9DS1_SetFullScale(uint8_t accel_fs, uint8_t gyro_fs);

/* 读出最近一次 DMA 解包得到的原始六轴数据 */
void LSM9DS1_GetLatestRaw(LSM9DS1_RawData_t *out_raw);

/* 物理量换算接口：严格遵循文档灵敏度公式 */
float LSM9DS1_AccelRawToG(int16_t raw_val);
float LSM9DS1_GyroRawToDps(int16_t raw_val);

/* 运行状态读取（只读指针） */
const LSM9DS1_RuntimeState_t *LSM9DS1_GetRuntimeState(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM9DS1_H */
