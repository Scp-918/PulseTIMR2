#include "MAX30101.h"

#include "i2c.h"

/*
 * 本驱动运行模型：
 * 1) 外部 10ms 主定时器调用 MAX30101_TriggerPointerRead_IT()，只发起非阻塞 IT 读指针。
 * 2) 在 I2C MemRx 完成回调中先完成 WR/RD 指针读取，再根据可用样本数启动 DMA 读 FIFO。
 * 3) DMA 完成后解析 Green/Red/IR 并压入外部 Ring Buffer。
 */

typedef enum
{
    MAX30101_ASYNC_IDLE = 0, /* 空闲态：允许外部触发下一次 10ms 轮询 */
    MAX30101_ASYNC_PTR_WR,   /* 已发起 WR_PTR(0x04) 的 IT 读取 */
    MAX30101_ASYNC_PTR_RD,   /* 已发起 RD_PTR(0x06) 的 IT 读取 */
    MAX30101_ASYNC_FIFO_DMA  /* 已发起 FIFO_DATA(0x07) 的 DMA 突发读取 */
} MAX30101_AsyncState_t;

static MAX30101_RuntimeState_t s_runtime = {0};
static MAX30101_AsyncState_t s_async_state = MAX30101_ASYNC_IDLE;

/*
 * 指针读取使用独立字节缓存，避免栈变量生命周期问题。
 * DMA 缓冲区必须为静态存储期，确保 HAL DMA 在异步阶段访问地址有效。
 */
static uint8_t s_wr_ptr_raw = 0U;
static uint8_t s_rd_ptr_raw = 0U;
static uint8_t s_fifo_dma_buffer[MAX30101_FIFO_DEPTH * MAX30101_SAMPLE_BYTES] = {0};
static SensorRingBuffer_t *s_ring_buffer = NULL;

uint8_t MAX30101_WriteReg(uint8_t reg, uint8_t data)
{
    /*
     * 寄存器写（阻塞式）：
     * - 用于初始化和运行时配置更新，写长度固定 1 byte。
     * - 超时 20ms，失败返回 0，调用者据此中止后续步骤。
     */
    if (HAL_I2C_Mem_Write(&hi2c3,
                                                MAX30101_I2C_ADDR,
                                                reg,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &data,
                                                1U,
                                                20U) != HAL_OK)
    {
        return 0U;
    }
    return 1U;
}

uint8_t MAX30101_ReadReg(uint8_t reg, uint8_t *p_data)
{
    /* 参数保护：避免空指针导致 HardFault */
    if (p_data == NULL)
    {
        return 0U;
    }

    /*
     * 寄存器读（阻塞式）：
     * - 仅用于初始化/配置类低频路径（例如读 Part ID、读复位位）。
     * - 运行态采样链路不走本函数，不会在 10ms 任务内死等 FIFO 数据。
     */
    if (HAL_I2C_Mem_Read(&hi2c3,
                                             MAX30101_I2C_ADDR,
                                             reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             p_data,
                                             1U,
                                             20U) != HAL_OK)
    {
        return 0U;
    }
    return 1U;
}

uint8_t MAX30101_GetPartID(void)
{
    uint8_t id = 0U;

    /* 读取失败返回 0，用于上层快速识别 I2C/连线异常 */
    if (MAX30101_ReadReg(MAX30101_REG_PART_ID, &id) == 0U)
    {
        return 0U;
    }
    return id;
}

void MAX30101_AttachRingBuffer(SensorRingBuffer_t *rb)
{
    /*
     * 绑定外部环形缓冲区实例：
     * DMA 完成后，解析结果将写入 rb->buffer[*].ppg_data[0..2]。
     */
    s_ring_buffer = rb;
}

const MAX30101_RuntimeState_t *MAX30101_GetRuntimeState(void)
{
    /* 只读状态快照接口：用于调试串口/上位机查看链路健康度 */
    return &s_runtime;
}

uint8_t MAX30101_Init(void)
{
    uint8_t id = 0U;
    uint8_t mode_cfg = 0U;
    uint32_t wait_cnt = 0U;

    /*
     * 1) 读取 Part ID，必须为 0x15
     * 若不是 0x15，一般说明器件型号不符、地址错误或总线异常。
     */
    if (MAX30101_ReadReg(MAX30101_REG_PART_ID, &id) == 0U)
    {
        return 0U;
    }
    if (id != MAX30101_PART_ID_VALUE)
    {
        return 0U;
    }

    /*
     * 2) 软复位：置位 RESET(bit6)，并轮询等待自动清零。
     * 复位后寄存器回到默认态，避免上电毛刺或历史状态影响配置。
     */
    if (MAX30101_WriteReg(MAX30101_REG_MODE_CONFIG, 0x40U) == 0U)
    {
        return 0U;
    }

    for (wait_cnt = 0U; wait_cnt < 100U; wait_cnt++)
    {
        /* 每 1ms 读取一次 RESET 位，最多等待约 100ms */
        if (MAX30101_ReadReg(MAX30101_REG_MODE_CONFIG, &mode_cfg) == 0U)
        {
            return 0U;
        }
        if ((mode_cfg & 0x40U) == 0U)
        {
            break;
        }
        HAL_Delay(1U);
    }
    if ((mode_cfg & 0x40U) != 0U)
    {
        return 0U;
    }

    /*
     * 3) 清空 FIFO 写/读指针及溢出计数。
     * 这一步是后续“(WR+32-RD)%32”回绕计算正确的基础。
     */
    if ((MAX30101_WriteReg(MAX30101_REG_FIFO_WR_PTR, 0x00U) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_OVF_COUNTER, 0x00U) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_FIFO_RD_PTR, 0x00U) == 0U))
    {
        return 0U;
    }

    /* 4) 配置 FIFO: SMP_AVE=8, FIFO_ROLLOVER_EN=1, A_FULL=0x0F */
    if (MAX30101_WriteReg(MAX30101_REG_FIFO_CONFIG, MAX30101_INIT_FIFO_CONFIG) == 0U)
    {
        return 0U;
    }

    /*
     * 5) 配置 SPO2：
     * - SR=800sps 与 100Hz 输出 + 8x 平均匹配。
     * - LED_PW=215us 对应 17-bit 分辨率。
     */
    if (MAX30101_WriteReg(MAX30101_REG_SPO2_CONFIG, MAX30101_INIT_SPO2_CONFIG) == 0U)
    {
        return 0U;
    }

    /* 默认 LED 电流：上电后先进入可用发光强度，可再由上位机动态调节 */
    if ((MAX30101_WriteReg(MAX30101_REG_LED1_PA, MAX30101_DEFAULT_LED_RED_PA) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_LED2_PA, MAX30101_DEFAULT_LED_IR_PA) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_LED3_PA, MAX30101_DEFAULT_LED_GREEN_PA) == 0U))
    {
        return 0U;
    }

    /*
     * 6) 多光路时隙：
     * FIFO 内样本顺序固定为 Green -> Red -> IR（每路 3 字节）。
     */
    if ((MAX30101_WriteReg(MAX30101_REG_MULTI_LED_CTRL1, MAX30101_INIT_SLOT_CTRL1) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_MULTI_LED_CTRL2, MAX30101_INIT_SLOT_CTRL2) == 0U))
    {
        return 0U;
    }

    /* 7) 进入 Multi-LED 模式，开始按时隙循环采样 */
    if (MAX30101_WriteReg(MAX30101_REG_MODE_CONFIG, MAX30101_INIT_MODE_CONFIG) == 0U)
    {
        return 0U;
    }

    /*
     * 清运行状态：
     * 避免重新初始化后保留旧统计/旧阶段，导致外部误判驱动忙状态。
     */
    s_async_state = MAX30101_ASYNC_IDLE;
    s_runtime.busy = 0U;
    s_runtime.ptr_stage = 0U;
    s_runtime.last_wr_ptr = 0U;
    s_runtime.last_rd_ptr = 0U;
    s_runtime.pending_samples = 0U;

    return 1U;
}

HAL_StatusTypeDef MAX30101_TriggerPointerRead_IT(void)
{
    HAL_StatusTypeDef ret;

    /*
     * 外部 10ms 定时器触发入口：
     * 仅发起“指针读取第一步”，绝不在此处阻塞等待数据。
     */

    /* 仅允许在空闲态发起，防止重入导致 I2C 总线状态错乱 */
    if (s_async_state != MAX30101_ASYNC_IDLE)
    {
        return HAL_BUSY;
    }

    if (hi2c3.State != HAL_I2C_STATE_READY)
    {
        /* 总线忙时让出本周期，下一拍再尝试 */
        return HAL_BUSY;
    }

    s_async_state = MAX30101_ASYNC_PTR_WR;
    s_runtime.busy = 1U;
    s_runtime.ptr_stage = 1U;

    /*
     * 按需求仅用 HAL_I2C_Mem_Read_IT 读取指针寄存器。
     * 第一步先读 0x04 (FIFO_WR_PTR)。完成后在回调里继续读 0x06 (FIFO_RD_PTR)。
     */
    ret = HAL_I2C_Mem_Read_IT(&hi2c3,
                                                        MAX30101_I2C_ADDR,
                                                        MAX30101_REG_FIFO_WR_PTR,
                                                        I2C_MEMADD_SIZE_8BIT,
                                                        &s_wr_ptr_raw,
                                                        1U);
    if (ret != HAL_OK)
    {
        /* 发起失败时必须完整回滚状态，避免卡死在 busy */
        s_async_state = MAX30101_ASYNC_IDLE;
        s_runtime.busy = 0U;
        s_runtime.ptr_stage = 0U;
        s_runtime.i2c_error_count++;
    }

    return ret;
}

void MAX30101_PointerRxCpltCallback(void)
{
    uint8_t wr_ptr;
    uint8_t rd_ptr;
    uint8_t available_samples;
    uint16_t dma_len;
    HAL_StatusTypeDef ret;

    /*
     * 指针有效位仅 [4:0]：
     * 先做掩码，再参与回绕计算，避免高位脏数据引入错误样本数。
     */
    wr_ptr = (uint8_t)(s_wr_ptr_raw & 0x1FU);
    rd_ptr = (uint8_t)(s_rd_ptr_raw & 0x1FU);
    s_runtime.last_wr_ptr = wr_ptr;
    s_runtime.last_rd_ptr = rd_ptr;
    s_runtime.ptr_read_ok_count++;

    /*
     * FIFO 环形回绕计算：
     * 可用样本 = (WR_PTR + 32 - RD_PTR) % 32
     * 说明：当两端时钟存在漂移且本次尚未产生新样本时，可能得到 0，应直接退出等待下个 10ms 周期。
     */
    available_samples = (uint8_t)((wr_ptr + MAX30101_FIFO_DEPTH - rd_ptr) % MAX30101_FIFO_DEPTH);
    if (available_samples == 0U)
    {
        s_runtime.skip_empty_count++;
        s_runtime.pending_samples = 0U;
        s_async_state = MAX30101_ASYNC_IDLE;
        s_runtime.busy = 0U;
        s_runtime.ptr_stage = 0U;
        return;
    }

    /*
     * 长度保护：
     * 理论上 available_samples 最大 31；这里仍做一次缓冲区上限钳制，
     * 防止未来修改宏后出现长度越界。
     */
    if (available_samples > (uint8_t)(sizeof(s_fifo_dma_buffer) / MAX30101_SAMPLE_BYTES))
    {
        available_samples = (uint8_t)(sizeof(s_fifo_dma_buffer) / MAX30101_SAMPLE_BYTES);
    }

    dma_len = (uint16_t)available_samples * (uint16_t)MAX30101_SAMPLE_BYTES;
    s_runtime.pending_samples = available_samples;
    s_async_state = MAX30101_ASYNC_FIFO_DMA;

    ret = HAL_I2C_Mem_Read_DMA(&hi2c3,
                                                         MAX30101_I2C_ADDR,
                                                         MAX30101_REG_FIFO_DATA,
                                                         I2C_MEMADD_SIZE_8BIT,
                                                         s_fifo_dma_buffer,
                                                         dma_len);
    if (ret != HAL_OK)
    {
        /* DMA 启动失败：回到空闲态，等待下一次 10ms 触发 */
        s_runtime.i2c_error_count++;
        s_runtime.pending_samples = 0U;
        s_async_state = MAX30101_ASYNC_IDLE;
        s_runtime.busy = 0U;
    }
}

void MAX30101_DataRxCpltCallback(uint8_t *dma_buffer, uint8_t sample_count)
{
    uint8_t i;

    /* 容错保护：空缓冲或空样本直接返回 */
    if ((dma_buffer == NULL) || (sample_count == 0U))
    {
        return;
    }

    if (s_ring_buffer == NULL)
    {
        /* 未绑定外部 Ring Buffer 时仅丢弃数据，不做非法写访问 */
        return;
    }

    for (i = 0U; i < sample_count; i++)
    {
        /* 每帧 9 字节：G[0..2], R[3..5], IR[6..8] */
        uint8_t base = (uint8_t)(i * MAX30101_SAMPLE_BYTES);
        uint32_t green_raw;
        uint32_t red_raw;
        uint32_t ir_raw;
        SensorDataFrame_t frame;

        /*
         * 每个通道 3 字节拼接为 24bit 左对齐原始码，
         * 在 LED_PW=215us(17-bit) 配置下，按需求统一右移 1 位得到真实 ADC 值。
         */
        green_raw = ((uint32_t)dma_buffer[base] << 16) |
                                ((uint32_t)dma_buffer[base + 1U] << 8) |
                                ((uint32_t)dma_buffer[base + 2U]);
        red_raw = ((uint32_t)dma_buffer[base + 3U] << 16) |
                            ((uint32_t)dma_buffer[base + 4U] << 8) |
                            ((uint32_t)dma_buffer[base + 5U]);
        ir_raw = ((uint32_t)dma_buffer[base + 6U] << 16) |
                         ((uint32_t)dma_buffer[base + 7U] << 8) |
                         ((uint32_t)dma_buffer[base + 8U]);

        /*
         * 当前工程 RingBuffer 存储的是“整帧传感器融合结构体”。
         * 这里先清零其余字段，仅填充 ppg_data，确保结构体内容确定。
         */
        frame = (SensorDataFrame_t){0};
        frame.ppg_data[0] = (green_raw >> 1) & 0x1FFFFU;
        frame.ppg_data[1] = (red_raw >> 1) & 0x1FFFFU;
        frame.ppg_data[2] = (ir_raw >> 1) & 0x1FFFFU;

        /* 入队失败（极端情况下）无需阻塞；RingBuffer 内部已实现满载覆盖策略 */
        (void)RingBuffer_Push(s_ring_buffer, &frame);
    }
}

uint8_t MAX30101_SetLEDCurrent(uint8_t led_type, uint8_t current_val)
{
    uint8_t reg;

    /* LED 类型 -> 电流寄存器地址映射 */
    switch (led_type)
    {
        case MAX30101_LED_RED:
            reg = MAX30101_REG_LED1_PA;
            break;
        case MAX30101_LED_IR:
            reg = MAX30101_REG_LED2_PA;
            break;
        case MAX30101_LED_GREEN:
            reg = MAX30101_REG_LED3_PA;
            break;
        default:
            return 0U;
    }

    /* current_val 取值 0x00~0xFF，典型换算 I_LED ~= reg * 0.2mA */
    return MAX30101_WriteReg(reg, current_val);
}

uint8_t MAX30101_SetLEDMode(MAX30101_LedMode_t mode,
                                                        uint8_t green_current,
                                                        uint8_t red_current,
                                                        uint8_t ir_current,
                                                        uint8_t spo2_sr_code,
                                                        uint8_t smp_ave_code)
{
    uint8_t spo2_cfg;
    uint8_t fifo_cfg;
    uint8_t mode_cfg;
    uint8_t slot1_slot2;
    uint8_t slot3_slot4;

    /*
     * 运行时组合寄存器：
     * - 固定 RGE=8192nA(0b10)
     * - SR 由上位机给出（3bit）
     * - 固定 LED_PW=215us(17-bit, 0b10)
     */
    /*
     * spo2_cfg: [6:5]=RGE=0b10 [4:2]=SR [1:0]=PW=0b10
     * fifo_cfg: [7:5]=SMP_AVE [4]=ROLLOVER_EN=1 [3:0]=A_FULL=0x0F
     */
    spo2_cfg = (uint8_t)((0x02U << 5) | ((spo2_sr_code & 0x07U) << 2) | 0x02U);
    fifo_cfg = (uint8_t)(((smp_ave_code & 0x07U) << 5) | (1U << 4) | 0x0FU);

    switch (mode)
    {
        case MAX30101_LED_MODE_GREEN_ONLY:
            /* 使用 Multi-LED + 仅 SLOT1=GREEN 实现“仅绿光” */
            mode_cfg = 0x07U;
            slot1_slot2 = 0x03U;
            slot3_slot4 = 0x00U;
            break;

        case MAX30101_LED_MODE_SPO2:
            /* 原生 SpO2 模式：RED + IR */
            mode_cfg = 0x03U;
            slot1_slot2 = 0x00U;
            slot3_slot4 = 0x00U;
            break;

        case MAX30101_LED_MODE_MULTI_G_R_IR:
            /* 三光路轮切：Green -> Red -> IR */
            mode_cfg = 0x07U;
            slot1_slot2 = 0x13U;
            slot3_slot4 = 0x02U;
            break;

        default:
            return 0U;
    }

    /* 先更新 LED 电流，保证模式切换后立即使用新电流参数 */
    if ((MAX30101_SetLEDCurrent(MAX30101_LED_GREEN, green_current) == 0U) ||
            (MAX30101_SetLEDCurrent(MAX30101_LED_RED, red_current) == 0U) ||
            (MAX30101_SetLEDCurrent(MAX30101_LED_IR, ir_current) == 0U))
    {
        return 0U;
    }

    /* 再更新采样/平均寄存器 */
    if (MAX30101_WriteReg(MAX30101_REG_FIFO_CONFIG, fifo_cfg) == 0U)
    {
        return 0U;
    }

    if (MAX30101_WriteReg(MAX30101_REG_SPO2_CONFIG, spo2_cfg) == 0U)
    {
        return 0U;
    }

    /* 先写 slots 再写 mode，避免切换瞬间 FIFO 通道顺序与预期不一致 */
    if ((MAX30101_WriteReg(MAX30101_REG_MULTI_LED_CTRL1, slot1_slot2) == 0U) ||
            (MAX30101_WriteReg(MAX30101_REG_MULTI_LED_CTRL2, slot3_slot4) == 0U))
    {
        return 0U;
    }

    if (MAX30101_WriteReg(MAX30101_REG_MODE_CONFIG, mode_cfg) == 0U)
    {
        return 0U;
    }

    return 1U;
}

void MAX30101_I2CMemRxCpltHandler(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;

    /* 仅处理本驱动绑定的 I2C3 回调，其它 I2C 设备直接忽略 */
    if (hi2c != &hi2c3)
    {
        return;
    }

    if (s_async_state == MAX30101_ASYNC_PTR_WR)
    {
        /*
         * 阶段A完成：已拿到 WR_PTR，继续发起 RD_PTR 读取。
         * 该步骤仍使用 IT，满足“指针读取必须 IT 非阻塞”的约束。
         */
        s_async_state = MAX30101_ASYNC_PTR_RD;
        s_runtime.ptr_stage = 2U;
        ret = HAL_I2C_Mem_Read_IT(&hi2c3,
                                                            MAX30101_I2C_ADDR,
                                                            MAX30101_REG_FIFO_RD_PTR,
                                                            I2C_MEMADD_SIZE_8BIT,
                                                            &s_rd_ptr_raw,
                                                            1U);
        if (ret != HAL_OK)
        {
            /* 发起失败时立即回空闲，避免后续状态悬挂 */
            s_runtime.i2c_error_count++;
            s_async_state = MAX30101_ASYNC_IDLE;
            s_runtime.busy = 0U;
            s_runtime.ptr_stage = 0U;
        }
        return;
    }

    if (s_async_state == MAX30101_ASYNC_PTR_RD)
    {
        /* 阶段B完成：WR/RD 都已到位，进入样本数计算与 DMA 启动 */
        s_runtime.ptr_stage = 0U;
        MAX30101_PointerRxCpltCallback();
        return;
    }

    if (s_async_state == MAX30101_ASYNC_FIFO_DMA)
    {
        uint8_t samples = s_runtime.pending_samples;

        /* 阶段C完成：DMA 数据已到内存，先收尾状态，再做数据解包 */
        s_runtime.pending_samples = 0U;
        s_runtime.dma_read_ok_count++;
        s_async_state = MAX30101_ASYNC_IDLE;
        s_runtime.busy = 0U;

        MAX30101_DataRxCpltCallback(s_fifo_dma_buffer, samples);
        return;
    }
}

void MAX30101_I2CErrorHandler(I2C_HandleTypeDef *hi2c)
{
    /* HAL I2C 错误统一落到这里，保证状态机可恢复 */
    if (hi2c != &hi2c3)
    {
        return;
    }

    s_runtime.i2c_error_count++;
    s_runtime.pending_samples = 0U;
    s_runtime.ptr_stage = 0U;
    s_async_state = MAX30101_ASYNC_IDLE;
    s_runtime.busy = 0U;
}

/*
 * 若工程中尚未实现 HAL I2C 全局回调，可直接使用以下桥接实现。
 * 如果后续有多个 I2C 设备共用回调，可将逻辑汇总后在其中调用 MAX30101_I2CMemRxCpltHandler/MAX30101_I2CErrorHandler。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    MAX30101_I2CMemRxCpltHandler(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    MAX30101_I2CErrorHandler(hi2c);
}