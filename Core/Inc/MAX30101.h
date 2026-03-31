#ifndef __MAX30101_H
#define __MAX30101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensor_ringbuffer.h"

/*
 * MAX30101 7-bit 从机地址为 0x57。
 * STM32 HAL 的 DevAddress 参数需传入左移 1 位后的地址（bit0 由硬件自动填充读写位）。
 * 因此这里定义为 0xAE。
 */
#define MAX30101_I2C_ADDR               (0x57U << 1)

/*
 * 寄存器地址定义
 * 说明：下述注释保留了关键位语义，便于后续动态配置时按位组合。
 */
#define MAX30101_REG_INTR_STATUS_1      0x00U /* 中断状态1，读后清除：A_FULL/PPG_RDY/ALC_OVF/PWR_RDY */
#define MAX30101_REG_INTR_STATUS_2      0x01U /* 中断状态2，读后清除：DIE_TEMP_RDY */
#define MAX30101_REG_INTR_ENABLE_1      0x02U /* 中断使能1：与 0x00 中断源对应（当前方案可不启用） */
#define MAX30101_REG_INTR_ENABLE_2      0x03U /* 中断使能2：温度中断等（当前方案可不启用） */

#define MAX30101_REG_FIFO_WR_PTR        0x04U /* FIFO 写指针，仅 bit[4:0] 有效，范围 0~31，硬件写入后自增 */
#define MAX30101_REG_OVF_COUNTER        0x05U /* FIFO 溢出计数，仅 bit[4:0] 有效，饱和到 0x1F */
#define MAX30101_REG_FIFO_RD_PTR        0x06U /* FIFO 读指针，仅 bit[4:0] 有效，主机读取 FIFO_DATA 后自增 */
#define MAX30101_REG_FIFO_DATA          0x07U /* FIFO 数据窗口：连续读取同一地址即可顺序弹出数据 */

#define MAX30101_REG_FIFO_CONFIG        0x08U /* [7:5]SMP_AVE [4]ROLLOVER_EN [3:0]A_FULL */
#define MAX30101_REG_MODE_CONFIG        0x09U /* [6]RESET [7]SHDN [2:0]MODE(0x2/0x3/0x7) */
#define MAX30101_REG_SPO2_CONFIG        0x0AU /* [6:5]RGE [4:2]SR [1:0]LED_PW */

#define MAX30101_REG_LED1_PA            0x0CU /* RED LED 电流寄存器，电流约 = reg * 0.2mA */
#define MAX30101_REG_LED2_PA            0x0DU /* IR LED 电流寄存器，电流约 = reg * 0.2mA */
#define MAX30101_REG_LED3_PA            0x0EU /* GREEN LED 电流寄存器，电流约 = reg * 0.2mA */

#define MAX30101_REG_MULTI_LED_CTRL1    0x11U /* [2:0]SLOT1 [6:4]SLOT2，编码：001=RED,010=IR,011=GREEN */
#define MAX30101_REG_MULTI_LED_CTRL2    0x12U /* [2:0]SLOT3 [6:4]SLOT4，编码同上，000=Disabled */

#define MAX30101_REG_REV_ID             0xFEU /* 芯片 Revision ID，可用于版本排查 */
#define MAX30101_REG_PART_ID            0xFFU /* 芯片 Part ID，MAX30101 固定为 0x15 */

/*
 * 固定 Part ID 期望值。
 * 初始化阶段先读 0xFF 并校验该值，可快速定位 I2C 连线/地址错误。
 */
#define MAX30101_PART_ID_VALUE          0x15U

/*
 * FIFO 与帧宽参数
 * MAX30101 FIFO 深度固定 32 样本。
 * 本工程使用三光路 Green/Red/IR，每路 3 字节，所以单样本固定 9 字节。
 */
#define MAX30101_FIFO_DEPTH             32U
#define MAX30101_CHANNEL_COUNT          3U  /* 通道顺序与时隙配置对应：Green, Red, IR */
#define MAX30101_BYTES_PER_LED          3U  /* 每个 LED 通道输出 3-byte 原始码 */
#define MAX30101_SAMPLE_BYTES           (MAX30101_CHANNEL_COUNT * MAX30101_BYTES_PER_LED)

/* LED 类型定义：用于运行时调节 LED 电流 */
typedef enum
{
	MAX30101_LED_RED = 0U,
	MAX30101_LED_IR,
	MAX30101_LED_GREEN
} MAX30101_LedType_t;

/* 运行时模式定义 */
typedef enum
{
	MAX30101_LED_MODE_GREEN_ONLY = 0U, /* 仅绿光心率 */
	MAX30101_LED_MODE_SPO2,            /* 红光+红外血氧 */
	MAX30101_LED_MODE_MULTI_G_R_IR     /* 绿-红-红外轮切 */
} MAX30101_LedMode_t;

/*
 * 初始化默认配置：目标 100Hz ODR + 8x 平均 + 17-bit
 * 关系说明：f_ADC = f_ODR * N_AVG = 100 * 8 = 800sps，因此 SR 选 800sps。
 */
#define MAX30101_INIT_FIFO_CONFIG       0x7FU /* 0b011_1_1111: SMP_AVE=8, ROLLOVER_EN=1, A_FULL=15 */
#define MAX30101_INIT_SPO2_CONFIG       0x52U /* 0b10_100_10: RGE=8192nA, SR=800sps, LED_PW=215us(17-bit) */
#define MAX30101_INIT_MODE_CONFIG       0x07U /* Multi-LED 模式，按 SLOT1~SLOT4 顺序出数 */
#define MAX30101_INIT_SLOT_CTRL1        0x13U /* SLOT1=GREEN(011), SLOT2=RED(001) */
#define MAX30101_INIT_SLOT_CTRL2        0x02U /* SLOT3=IR(010), SLOT4=Disabled(000) */

/*
 * 默认 LED 电流（启动值）
 * 0x3F 对应约 12.6mA（典型值，I_LED ~= reg * 0.2mA）。
 * 若出现 ALC_OVF，可在运行时下调（例如 0x28~0x30）。
 */
#define MAX30101_DEFAULT_LED_RED_PA     0x3FU
#define MAX30101_DEFAULT_LED_IR_PA      0x3FU
#define MAX30101_DEFAULT_LED_GREEN_PA   0x3FU

/* 对外状态结构，用于调试与上位机监视 */
typedef struct
{
	volatile uint8_t busy;                 /* 1=正在进行 I2C 异步流程 */
	volatile uint8_t ptr_stage;            /* 指针读取阶段：0空闲/1读WR/2读RD */
	volatile uint8_t last_wr_ptr;          /* 最近一次读取到的写指针 */
	volatile uint8_t last_rd_ptr;          /* 最近一次读取到的读指针 */
	volatile uint8_t pending_samples;      /* 最近一次 DMA 请求样本数 */
	volatile uint32_t ptr_read_ok_count;   /* 指针读取成功次数 */
	volatile uint32_t dma_read_ok_count;   /* DMA 读取成功次数 */
	volatile uint32_t skip_empty_count;    /* 可用样本为0的跳过次数 */
	volatile uint32_t i2c_error_count;     /* I2C 错误计数 */
} MAX30101_RuntimeState_t;

/* 基础寄存器访问 */
uint8_t MAX30101_WriteReg(uint8_t reg, uint8_t data);
uint8_t MAX30101_ReadReg(uint8_t reg, uint8_t *p_data);
uint8_t MAX30101_GetPartID(void);

/* 1) 核心初始化 */
uint8_t MAX30101_Init(void);

/* 外部环形缓冲区绑定：DMA 完成回调会将解析结果压入该缓冲区 */
void MAX30101_AttachRingBuffer(SensorRingBuffer_t *rb);

/* 2) 非阻塞状态机入口与回调 */
HAL_StatusTypeDef MAX30101_TriggerPointerRead_IT(void);
void MAX30101_PointerRxCpltCallback(void);
void MAX30101_DataRxCpltCallback(uint8_t *dma_buffer, uint8_t sample_count);

/* 供主工程在 HAL 全局回调中转发（若直接使用本文件中的 HAL 回调实现，则无需显式调用） */
void MAX30101_I2CMemRxCpltHandler(I2C_HandleTypeDef *hi2c);
void MAX30101_I2CErrorHandler(I2C_HandleTypeDef *hi2c);

/* 3) 运行时动态配置接口 */
uint8_t MAX30101_SetLEDCurrent(uint8_t led_type, uint8_t current_val);
uint8_t MAX30101_SetLEDMode(MAX30101_LedMode_t mode,
														uint8_t green_current,
														uint8_t red_current,
														uint8_t ir_current,
														uint8_t spo2_sr_code,
														uint8_t smp_ave_code);

/* 按 BLE 13 字节协议直接下发 MAX30101 组合配置。 */
uint8_t MAX30101_ApplyBleParamConfig(uint8_t mode_code,
															 uint8_t multi_mode_code,
															 uint8_t green_current,
															 uint8_t red_current,
															 uint8_t ir_current,
															 uint8_t adc_rge_code,
															 uint8_t led_pw_code,
															 uint8_t spo2_sr_code,
															 uint8_t smp_ave_code);

/* 状态查询 */
const MAX30101_RuntimeState_t *MAX30101_GetRuntimeState(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30101_H */