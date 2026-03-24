#include "AD4007.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi3;

/* 统一使用 0xFF 作为读取时的发送填充值，确保 MOSI 在最后一位保持为高。 */
static const uint8_t AD4007_SPI_TX_DUMMY[AD4007_FRAME_BYTES] = {0xFFu, 0xFFu, 0xFFu};

/*
 * 通过空转 NOP 提供一个亚微秒级短延时。
 * 该延时用于满足 tCONV/tQUIET 的数量级要求，不依赖定时器。
 */
static void AD4007_DelayCycles(volatile uint32_t cycles)
{
    while (cycles-- > 0u)
    {
        __NOP();
    }
}

/*
 * AD4007 tCONV 最大值约 320ns。
 * 在 STM32G474 常见主频 (170MHz) 下，60~100 个周期已明显覆盖 320ns。
 */
static void AD4007_Wait_tCONV_Max(void)
{
    AD4007_DelayCycles(120u);
}

/* 将 PA9 临时切换为普通 GPIO 输出，供软件手动控制 CNV。 */
static void AD4007_PA9_ToGPIO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* 将 PA9 恢复为 HRTIM1_CHA2 对应的 AF13 复用模式。 */
static void AD4007_PA9_ToAF13_HRTIM(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * 软件触发一次 CNV 脉冲：
 * 1) CNV 拉高触发转换；
 * 2) 等待 tCONV；
 * 3) 拉低进入串行读取窗口。
 */
static void AD4007_GenerateCnvPulse_SW(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    AD4007_Wait_tCONV_Max();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

/*
 * 把 3 字节原始帧解包为 18-bit 补码并做符号扩展。
 * - 原始 24-bit: [D17...D0][x x x x x x]
 * - 右移 6 位后取低 18 位
 * - 若 bit17=1，则把高 14 位补 1，得到 int32 负数
 */
static int32_t AD4007_DecodeOneSample(const uint8_t frame[AD4007_FRAME_BYTES])
{
    uint32_t raw24 = ((uint32_t)frame[0] << 16)
                   | ((uint32_t)frame[1] << 8)
                   | ((uint32_t)frame[2]);

    uint32_t code18 = (raw24 >> AD4007_FRAME_RIGHT_SHIFT) & AD4007_CODE_MASK_18BIT;

    if ((code18 & AD4007_SIGN_BIT_18BIT) != 0u)
    {
        return (int32_t)(code18 | AD4007_SIGN_EXTEND_MASK);
    }

    return (int32_t)code18;
}

/*
 * 强制 MOSI 拉高：
 * 读取结束后把 PC12(MOSI) 短暂切到 GPIO 输出高，再恢复 SPI3 AF6。
 * 这样可确保 AD4007 在下一次 CNV 采样到 SDI=1，维持 CS 模式。
 */
void AD4007_ForceMOSIHigh(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* 组合配置字节并写入 ADC 配置寄存器（0x14 + 1Byte 配置）。 */
HAL_StatusTypeDef AD4007_ConfigMode(bool enable_high_z, bool enable_span_comp, bool enable_turbo)
{
    HAL_StatusTypeDef ret;
    uint8_t tx_buf[2];
    uint8_t cfg = 0u;

    if (enable_high_z)
    {
        cfg |= (uint8_t)AD4007_CFG_HIGH_Z_MASK;
    }
    if (enable_span_comp)
    {
        cfg |= (uint8_t)AD4007_CFG_SPAN_COMP_MASK;
    }
    if (enable_turbo)
    {
        cfg |= (uint8_t)AD4007_CFG_TURBO_MASK;
    }

    tx_buf[0] = AD4007_CMD_WRITE_CONFIG;
    tx_buf[1] = cfg;

    /* 配置寄存器写入需要软件控制 CNV/CS 时序，这里临时接管 PA9。 */
    AD4007_PA9_ToGPIO_Output();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    ret = HAL_SPI_Transmit(&hspi3, tx_buf, 2u, 10u);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    AD4007_DelayCycles(40u); /* 预留 tQUIET2 余量 */
    AD4007_PA9_ToAF13_HRTIM();

    AD4007_ForceMOSIHigh();
    return ret;
}

/* 核心初始化：校验 SPI 模式、写默认配置、执行空转换并丢弃首帧。 */
HAL_StatusTypeDef AD4007_Init(void)
{
    HAL_StatusTypeDef ret;
    uint8_t discard_rx[AD4007_FRAME_BYTES] = {0};

    /* 必须是 SPI Mode 0：CPOL=0, CPHA=0。若不满足则重配 SPI3。 */
    if ((hspi3.Init.CLKPolarity != SPI_POLARITY_LOW) || (hspi3.Init.CLKPhase != SPI_PHASE_1EDGE))
    {
        hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
        if (HAL_SPI_Init(&hspi3) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    /* 默认配置：Turbo 关 + High-Z 开 + Span Compression 关 */
    ret = AD4007_ConfigMode(true, false, false);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /*
     * 上电后的初始空转换流程：
     * 1) 软件拉高 CNV，等待 tCONV；
     * 2) 拉低后读取 1 帧并丢弃，避免首帧无效数据影响后续计算。
     */
    AD4007_PA9_ToGPIO_Output();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    AD4007_GenerateCnvPulse_SW();

    ret = HAL_SPI_TransmitReceive(&hspi3,
                                  (uint8_t *)AD4007_SPI_TX_DUMMY,
                                  discard_rx,
                                  AD4007_FRAME_BYTES,
                                  10u);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    AD4007_PA9_ToAF13_HRTIM();

    AD4007_ForceMOSIHigh();
    return ret;
}

/*
 * 历史调试接口归档：
 * - 下方两组函数用于“软件手动 CNV + 单点读取”联调；
 * - 当前正式路径改为 HRTIM 驱动 + AD4007_Start_DMA_Rx()；
 * - 为减少正式固件符号暴露与误调用风险，此处默认不参与编译。
 */
#if 0
HAL_StatusTypeDef AD4007_test_Rx(int32_t *out_code)
{
    HAL_StatusTypeDef ret;
    uint8_t rx_frame[AD4007_FRAME_BYTES] = {0};

    if (out_code == NULL)
    {
        return HAL_ERROR;
    }

    AD4007_PA9_ToGPIO_Output();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    AD4007_GenerateCnvPulse_SW();

    ret = HAL_SPI_TransmitReceive(&hspi3,
                                  (uint8_t *)AD4007_SPI_TX_DUMMY,
                                  rx_frame,
                                  AD4007_FRAME_BYTES,
                                  10u);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    AD4007_PA9_ToAF13_HRTIM();

    AD4007_ForceMOSIHigh();

    if (ret != HAL_OK)
    {
        return ret;
    }

    *out_code = AD4007_DecodeOneSample(rx_frame);
    return HAL_OK;
}

HAL_StatusTypeDef AD4007_test_DMA_Rx(int32_t *out_code, uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret;
    uint8_t rx_frame[AD4007_FRAME_BYTES] = {0};
    uint32_t tick_begin;

    if (out_code == NULL)
    {
        return HAL_ERROR;
    }

    AD4007_PA9_ToGPIO_Output();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    AD4007_GenerateCnvPulse_SW();

    ret = HAL_SPI_TransmitReceive_DMA(&hspi3,
                                      (uint8_t *)AD4007_SPI_TX_DUMMY,
                                      rx_frame,
                                      AD4007_FRAME_BYTES);
    if (ret != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        AD4007_PA9_ToAF13_HRTIM();
        AD4007_ForceMOSIHigh();
        return ret;
    }

    tick_begin = HAL_GetTick();
    while (hspi3.State != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tick_begin) >= timeout_ms)
        {
            (void)HAL_SPI_Abort(&hspi3);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            AD4007_PA9_ToAF13_HRTIM();
            AD4007_ForceMOSIHigh();
            return HAL_TIMEOUT;
        }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    AD4007_PA9_ToAF13_HRTIM();

    *out_code = AD4007_DecodeOneSample(rx_frame);
    return HAL_OK;
}
#endif

/*
 * 启动 SPI3 + DMA 接收：
 * 每个样本固定 3 字节，因此 DMA 长度 = sample_count * 3。
 * 正常工作时 CNV 由 HRTIM 自动脉冲，本函数不进行任何 CNV 翻转。
 */
HAL_StatusTypeDef AD4007_Start_DMA_Rx(uint8_t *rx_buffer, uint16_t sample_count)
{
    uint32_t dma_len_bytes;

    if ((rx_buffer == NULL) || (sample_count == 0u))
    {
        return HAL_ERROR;
    }

    dma_len_bytes = (uint32_t)sample_count * AD4007_FRAME_BYTES;

    /* HAL SPI DMA 长度参数为 uint16_t，需防止溢出截断。 */
    if (dma_len_bytes > 0xFFFFu)
    {
        return HAL_ERROR;
    }

    /*
     * SPI 主机模式下，为了保证每一位都有 SCK，
     * 采用 TxRx DMA 并发送 0xFF dummy 字节流。
     */
    if (sample_count == 1u)
    {
        return HAL_SPI_TransmitReceive_DMA(&hspi3,
                                           (uint8_t *)AD4007_SPI_TX_DUMMY,
                                           rx_buffer,
                                           (uint16_t)dma_len_bytes);
    }

    return HAL_ERROR;
}

/*
 * 原始数据处理：
 * 1) 每 3 字节解包为 24-bit；
 * 2) 右移 6 位得到 18-bit 原始码；
 * 3) 执行 18-bit 补码符号扩展到 int32_t；
 * 4) 采用 int64 累加并取整型平均值。
 */
HAL_StatusTypeDef AD4007_ProcessRawData(uint8_t *dma_buffer, uint16_t sample_count, int32_t *out_avg_code)
{
    uint16_t i;
    int64_t sum = 0;

    if ((dma_buffer == NULL) || (out_avg_code == NULL) || (sample_count == 0u))
    {
        return HAL_ERROR;
    }

    for (i = 0u; i < sample_count; i++)
    {
        const uint8_t *frame = &dma_buffer[(uint32_t)i * AD4007_FRAME_BYTES];
        int32_t sample_code = AD4007_DecodeOneSample(frame);
        sum += (int64_t)sample_code;
    }

    *out_avg_code = (int32_t)(sum / (int64_t)sample_count);
    return HAL_OK;
}

/*
 * SPI3 接收完成回调（轻量化版本）：
 *
 * 设计意图：
 * 1) 高速触发场景下，DMA 完成回调必须尽量短，避免阻塞后续中断。
 * 2) 当前读取链路使用 HAL_SPI_TransmitReceive_DMA + 0xFF dummy；
 *    在 SPI 空闲后，MOSI 末位保持高电平（0xFF 的最后一位为 1），
 *    可满足大多数 CS 模式保持需求。
 *
 * 回滚说明：
 * - 若后续实验确认必须“每次 DMA 完成后强制 MOSI 拉高”，
 *   可恢复下方 #if 0 中旧逻辑。
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != NULL) && (hspi->Instance == SPI3))
    {
        /* 轻量路径：不在高频回调里做 GPIO 模式重配置。 */

#if 0
        /* 旧逻辑保留用于快速回滚对照。 */
        AD4007_ForceMOSIHigh();
#endif
    }
}