#include "LSM9DS1.h"

#include "spi.h"

#include <string.h>

/*
 * LSM9DS1 A/G 片选硬件定义：
 * - CS_A/G = PD7，低电平有效。
 * - 本驱动在每次 SPI 事务前后显式控制 CS，确保 DMA 时序可控。
 */
#define LSM9DS1_CS_GPIO_PORT            GPIOD
#define LSM9DS1_CS_PIN                  GPIO_PIN_7
#define LSM9DS1_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

/* SPI DMA 通信缓冲区：固定 13 字节 */
static uint8_t s_spi_tx_buf[LSM9DS1_SPI_DMA_FRAME_LEN] = {0};
static uint8_t s_spi_rx_buf[LSM9DS1_SPI_DMA_FRAME_LEN] = {0};

/* 运行态数据与状态 */
static volatile uint8_t s_dma_busy = 0U;
static volatile LSM9DS1_RawData_t s_latest_raw = {0};
static SensorDataFrame_t *s_bound_frame = NULL;
static SensorRingBuffer_t *s_ring_buffer = NULL;
static LSM9DS1_RuntimeState_t s_runtime = {0};

/*
 * 记录当前寄存器镜像值：
 * 运行时动态改量程时在此基础上改位，避免误改 ODR/BW 等非目标字段。
 */
static uint8_t s_ctrl_reg1_g_shadow = LSM9DS1_INIT_CTRL_REG1_G;
static uint8_t s_ctrl_reg6_xl_shadow = LSM9DS1_INIT_CTRL_REG6_XL;

/* 灵敏度系数（单位：mg/LSB 与 mdps/LSB），默认对应初始化量程 ±4g、±500dps */
static float s_accel_sens_mg_lsb = LSM9DS1_ACCEL_SENS_4G_MG_LSB;
static float s_gyro_sens_mdps_lsb = LSM9DS1_GYRO_SENS_500_MDPS_LSB;

static void LSM9DS1_CS_High(void)
{
    /* CS 拉高：SPI 总线空闲，器件不再响应时钟 */
    HAL_GPIO_WritePin(LSM9DS1_CS_GPIO_PORT, LSM9DS1_CS_PIN, GPIO_PIN_SET);
}

static void LSM9DS1_CS_Low(void)
{
    /* CS 拉低：选通 A/G 器件，后续 SCK 才会被视为有效传输 */
    HAL_GPIO_WritePin(LSM9DS1_CS_GPIO_PORT, LSM9DS1_CS_PIN, GPIO_PIN_RESET);
}

static void LSM9DS1_GPIO_InitCS(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    LSM9DS1_CS_GPIO_CLK_ENABLE();

    gpio_init.Pin = LSM9DS1_CS_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LSM9DS1_CS_GPIO_PORT, &gpio_init);

    /* 空闲态保持高电平，防止上电后误选中 */
    LSM9DS1_CS_High();
}

static HAL_StatusTypeDef LSM9DS1_WriteReg_Blocking(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];

    /*
     * LSM9DS1 SPI 写寄存器帧：
     * - 第 1 字节: [bit7=0 | reg_addr(6:0)]
     * - 第 2 字节: value
     * 这里强制清 bit7，避免误发读命令。
     */
    tx_buf[0] = (uint8_t)(reg & LSM9DS1_SPI_REG_ADDR_MASK);
    tx_buf[1] = value;

    LSM9DS1_CS_Low();
    if (HAL_SPI_Transmit(&hspi1, tx_buf, 2U, LSM9DS1_SPI_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        LSM9DS1_CS_High();
        return HAL_ERROR;
    }
    LSM9DS1_CS_High();

    return HAL_OK;
}

/*
 * 阻塞式读单寄存器：
 * - 通过 2 字节全双工完成（命令 + 占位时钟）。
 * - 第二字节回读值才是目标寄存器数据。
 */
static HAL_StatusTypeDef LSM9DS1_ReadReg_Blocking(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * LSM9DS1 SPI 读寄存器帧：
     * - 第 1 字节: [bit7=1 | reg_addr(6:0)]
     * - 第 2 字节: dummy(0x00)，仅用于继续提供时钟
     * 返回数据在 rx_buf[1]，rx_buf[0] 为命令阶段回传废位。
     */
    tx_buf[0] = (uint8_t)(reg | LSM9DS1_SPI_RW_READ_BIT);
    tx_buf[1] = 0x00U;

    LSM9DS1_CS_Low();
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2U, LSM9DS1_SPI_BLOCKING_TIMEOUT_MS) != HAL_OK)
    {
        LSM9DS1_CS_High();
        return HAL_ERROR;
    }
    LSM9DS1_CS_High();

    *value = rx_buf[1];
    return HAL_OK;
}

static HAL_StatusTypeDef LSM9DS1_ReadWhoAmI(uint8_t *who_am_i)
{
    /*
     * 明确按需求发送 0x8F：
     * WHO_AM_I(0x0F) | 读位(0x80) = 0x8F。
     * 返回值应为 0x68；该值不受量程/ODR配置影响，
     * 是判断 SPI 线、CS 时序、供电是否正确的首要门槛。
     */
    return LSM9DS1_ReadReg_Blocking(LSM9DS1_REG_WHO_AM_I, who_am_i);
}

static int16_t LSM9DS1_AssembleInt16LE(uint8_t low_byte, uint8_t high_byte)
{
    /* 输出寄存器采用 little-endian: OUT_X_L 在前，OUT_X_H 在后。 */
    return (int16_t)(((uint16_t)high_byte << 8) | (uint16_t)low_byte);
}

static void LSM9DS1_UpdateSensitivity(uint8_t accel_fs, uint8_t gyro_fs)
{
    /*
     * 此函数只维护“原始计数 -> 物理单位”换算系数，
     * 与寄存器写入动作解耦，避免在中断或热路径里做重复判断。
     */
    switch (accel_fs)
    {
        case LSM9DS1_ACCEL_FS_2G:
            s_accel_sens_mg_lsb = LSM9DS1_ACCEL_SENS_2G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_4G:
            s_accel_sens_mg_lsb = LSM9DS1_ACCEL_SENS_4G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_8G:
            s_accel_sens_mg_lsb = LSM9DS1_ACCEL_SENS_8G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_16G:
            s_accel_sens_mg_lsb = LSM9DS1_ACCEL_SENS_16G_MG_LSB;
            break;
        default:
            break;
    }

    switch (gyro_fs)
    {
        case LSM9DS1_GYRO_FS_245DPS:
            s_gyro_sens_mdps_lsb = LSM9DS1_GYRO_SENS_245_MDPS_LSB;
            break;
        case LSM9DS1_GYRO_FS_500DPS:
            s_gyro_sens_mdps_lsb = LSM9DS1_GYRO_SENS_500_MDPS_LSB;
            break;
        case LSM9DS1_GYRO_FS_2000DPS:
            s_gyro_sens_mdps_lsb = LSM9DS1_GYRO_SENS_2000_MDPS_LSB;
            break;
        default:
            break;
    }
}

/*
 * DMA 回包解析：
 * - 严格按小端序组包。
 * - 解析顺序固定为 Gx,Gy,Gz,Ax,Ay,Az。
 */
static void LSM9DS1_ProcessDmaRxData(void)
{
    LSM9DS1_RawData_t raw;

    /*
     * 数据格式：
     * - s_spi_rx_buf[0] 为命令阶段回传废位。
     * - s_spi_rx_buf[1..12] 为 6 组小端 16bit：Gx,Gy,Gz,Ax,Ay,Az。
     */
    raw.gx = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_GX_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_GX_H_INDEX]);
    raw.gy = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_GY_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_GY_H_INDEX]);
    raw.gz = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_GZ_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_GZ_H_INDEX]);
    raw.ax = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_AX_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_AX_H_INDEX]);
    raw.ay = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_AY_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_AY_H_INDEX]);
    raw.az = LSM9DS1_AssembleInt16LE(s_spi_rx_buf[LSM9DS1_RX_AZ_L_INDEX], s_spi_rx_buf[LSM9DS1_RX_AZ_H_INDEX]);

    s_latest_raw = raw;

    /*
     * 若绑定了外部“最新帧/中转缓存”，则直接覆盖 imu_data，
     * 由主循环在统一时序点完成整帧打包或入环形缓冲。
     */
    if (s_bound_frame != NULL)
    {
        s_bound_frame->imu_data[0] = raw.gx;
        s_bound_frame->imu_data[1] = raw.gy;
        s_bound_frame->imu_data[2] = raw.gz;
        s_bound_frame->imu_data[3] = raw.ax;
        s_bound_frame->imu_data[4] = raw.ay;
        s_bound_frame->imu_data[5] = raw.az;
    }

    if (s_ring_buffer != NULL)
    {
        SensorDataFrame_t frame = {0};

        frame.imu_data[0] = raw.gx;
        frame.imu_data[1] = raw.gy;
        frame.imu_data[2] = raw.gz;
        frame.imu_data[3] = raw.ax;
        frame.imu_data[4] = raw.ay;
        frame.imu_data[5] = raw.az;

        (void)RingBuffer_Push(s_ring_buffer, &frame);
    }
}

/*
 * 驱动初始化（阻塞路径，仅上电阶段调用）：
 * 1) 初始化 CS GPIO；
 * 2) WHO_AM_I 通信自检；
 * 3) 软复位 + 延时；
 * 4) 打开 BDU/自增地址，关闭 FIFO/中断；
 * 5) 写入陀螺仪/加速度计工作参数。
 */
HAL_StatusTypeDef LSM9DS1_Init(void)
{
    uint8_t who_am_i = 0U;

    LSM9DS1_GPIO_InitCS();

    /*
     * 新增核心校验：阻塞读取 WHO_AM_I。
     * 返回值必须为 0x68，否则直接判定初始化失败。
     */
    if (LSM9DS1_ReadWhoAmI(&who_am_i) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (who_am_i != LSM9DS1_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    /*
     * Step-1 软复位：CTRL_REG8(0x22)=0x05
     * - bit2 IF_ADD_INC=1：即使复位阶段也允许后续地址自增行为一致
     * - bit0 SW_RESET=1：触发内部数字逻辑复位
     * 复位后必须等待寄存器重装完成。
     */
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG8, LSM9DS1_INIT_CTRL_REG8_RESET) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(20U);

    /*
     * Step-2 工作总线配置：CTRL_REG8(0x22)=0x44
     * - bit6 BDU=1：高低字节需成对读取后才更新，防止“撕裂数据”
     * - bit2 IF_ADD_INC=1：突发读取地址自动递增，支持一次读完 12 字节
     */
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG8, LSM9DS1_INIT_CTRL_REG8_NORMAL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Step-3 关闭 FIFO 与硬件中断，采用“主控主动轮询”架构。
     * - CTRL_REG9(0x23)=0x00：FIFO_EN=0，禁用 FIFO 缓冲路径
     * - FIFO_CTRL(0x2E)=0x00：FMODE=Bypass，完全旁路 FIFO
     * - INT1_CTRL(0x0C)=0x00：关闭 DRDY/FTH/OVR 等硬件中断输出
     */
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG9, LSM9DS1_INIT_CTRL_REG9) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_FIFO_CTRL, LSM9DS1_INIT_FIFO_CTRL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_INT1_CTRL, LSM9DS1_INIT_INT1_CTRL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Step-4 陀螺仪配置
     * - CTRL_REG1_G(0x10)=0x68:
     *   ODR_G=119Hz, FS_G=±500dps, BW_G=14Hz
     * - CTRL_REG3_G(0x12)=0x46:
     *   HP_EN=1 + HPCF_G=0b0110，抑制慢变零偏漂移
     */
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG1_G, LSM9DS1_INIT_CTRL_REG1_G) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG3_G, LSM9DS1_INIT_CTRL_REG3_G) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Step-5 加速度计配置
        * - CTRL_REG6_XL(0x20)=0x60:
        *   ODR_XL=119Hz, FS_XL=±2g, 带宽自动由 ODR 决定
     * - CTRL_REG7_XL(0x21)=0xC4:
     *   HR=1 高分辨率, DCF=00(ODR/9), FDS=1 输出滤波后数据
     */
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG6_XL, LSM9DS1_INIT_CTRL_REG6_XL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG7_XL, LSM9DS1_INIT_CTRL_REG7_XL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 初始化“寄存器镜像 + 换算系数 + 运行态统计” */
    s_ctrl_reg1_g_shadow = LSM9DS1_INIT_CTRL_REG1_G;
    s_ctrl_reg6_xl_shadow = LSM9DS1_INIT_CTRL_REG6_XL;
    LSM9DS1_UpdateSensitivity(LSM9DS1_ACCEL_FS_2G, LSM9DS1_GYRO_FS_500DPS);

    s_dma_busy = 0U;
    s_runtime = (LSM9DS1_RuntimeState_t){0};
    (void)memset((void *)s_spi_rx_buf, 0, sizeof(s_spi_rx_buf));

    return HAL_OK;
}

/*
 * 绑定外部中转帧：
 * DMA 解包后会直接写入 frame->imu_data[0..5]。
 */
void LSM9DS1_AttachFrameBuffer(SensorDataFrame_t *frame)
{
    s_bound_frame = frame;
}

void LSM9DS1_AttachRingBuffer(SensorRingBuffer_t *rb)
{
    s_ring_buffer = rb;
}

/*
 * 10ms 主循环触发入口（非阻塞）：
 * - 若 SPI 或驱动正忙，返回 HAL_BUSY；
 * - 成功时仅发起 DMA，不在此函数等待完成。
 */
HAL_StatusTypeDef LSM9DS1_TriggerRead_IT(void)
{
    HAL_StatusTypeDef ret;

    if (s_dma_busy != 0U)
    {
        s_runtime.trigger_busy_count++;
        return HAL_BUSY;
    }

    /* 防止 SPI 外设正被其他事务占用（包括未完成的 DMA）。 */
    if (hspi1.State != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    /* 清缓冲保证本帧数据可追踪，便于调试 DMA 对齐问题 */
    (void)memset((void *)s_spi_tx_buf, 0, sizeof(s_spi_tx_buf));
    (void)memset((void *)s_spi_rx_buf, 0, sizeof(s_spi_rx_buf));
    /*
     * 0x98 = 0x18 | 0x80：
     * 从 OUT_X_L_G 起始做连续读，依赖 IF_ADD_INC 自动递增。
     */
    s_spi_tx_buf[0] = LSM9DS1_BURST_READ_CMD;

    /*
     * 必须先拉低 CS，再启动 DMA。
     * 这样 DMA 第一拍发送读命令时器件已被选中，避免首字节丢失。
     */
    LSM9DS1_CS_Low();
    /*
     * 13 字节 DMA 事务结构：
     * - tx[0] 命令 + tx[1..12] dummy
     * - rx[0] 废位 + rx[1..12] 六轴数据
     */
    ret = HAL_SPI_TransmitReceive_DMA(&hspi1, s_spi_tx_buf, s_spi_rx_buf, LSM9DS1_SPI_DMA_FRAME_LEN);
    if (ret != HAL_OK)
    {
        /* DMA 发起失败时立刻释放 CS，防止总线被长时间占用 */
        LSM9DS1_CS_High();
        s_dma_busy = 0U;
        s_runtime.dma_error_count++;
        s_runtime.dma_busy = 0U;
        return ret;
    }

    /* 置忙后由 DMA 完成/错误回调负责清 busy。 */
    s_dma_busy = 1U;
    s_runtime.dma_busy = 1U;
    return HAL_OK;
}

/*
 * SPI DMA 完成处理：
 * - 第一动作就是拉高 CS；
 * - 第二动作更新状态；
 * - 最后进行数据解包。
 */
void LSM9DS1_SPI_TxRxCpltHandler(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
    {
        return;
    }

    /*
     * DMA 完成后第一时间拉高 CS，结束本次 13 字节事务。
     * 该动作放在解包之前，最大限度缩短片选有效时间。
     */
    LSM9DS1_CS_High();
    s_dma_busy = 0U;
    s_runtime.dma_busy = 0U;
    /* 先记完成计数，再解包，便于上层按计数边沿判断本次事务已落地。 */
    s_runtime.dma_ok_count++;

    LSM9DS1_ProcessDmaRxData();
}

/*
 * SPI 错误处理：
 * 无论错误发生在何阶段，都必须强制释放 CS，
 * 保证后续轮询周期能够恢复。
 */
void LSM9DS1_SPI_ErrorHandler(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
    {
        return;
    }

    /* SPI 异常兜底：确保 CS 回到空闲态，避免后续事务被锁死 */
    LSM9DS1_CS_High();
    s_dma_busy = 0U;
    s_runtime.dma_busy = 0U;
    /* 任何 SPI 错误都计入错误计数，上位机可用其判断链路健康度。 */
    s_runtime.dma_error_count++;
}

/*
 * 运行时切换量程：
 * - 仅修改 FS 位，不改 ODR/带宽等配置；
 * - 两步写寄存器，第二步失败时回滚第一步；
 * - 成功后同步更新物理量换算灵敏度。
 */
HAL_StatusTypeDef LSM9DS1_SetFullScale(uint8_t accel_fs, uint8_t gyro_fs)
{
    uint8_t accel_fs_bits;
    uint8_t gyro_fs_bits;
    float new_accel_sens;
    float new_gyro_sens;
    uint8_t new_ctrl_reg6_xl;
    uint8_t new_ctrl_reg1_g;
    uint8_t old_ctrl_reg6_xl;

    /* 配置寄存器期间禁止与 DMA 读并发，避免读到跨量程混合帧。 */
    if (s_dma_busy != 0U)
    {
        return HAL_BUSY;
    }

    switch (accel_fs)
    {
        case LSM9DS1_ACCEL_FS_2G:
            accel_fs_bits = LSM9DS1_ACCEL_FS_BITS_2G;
            new_accel_sens = LSM9DS1_ACCEL_SENS_2G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_4G:
            accel_fs_bits = LSM9DS1_ACCEL_FS_BITS_4G;
            new_accel_sens = LSM9DS1_ACCEL_SENS_4G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_8G:
            accel_fs_bits = LSM9DS1_ACCEL_FS_BITS_8G;
            new_accel_sens = LSM9DS1_ACCEL_SENS_8G_MG_LSB;
            break;
        case LSM9DS1_ACCEL_FS_16G:
            accel_fs_bits = LSM9DS1_ACCEL_FS_BITS_16G;
            new_accel_sens = LSM9DS1_ACCEL_SENS_16G_MG_LSB;
            break;
        default:
            return HAL_ERROR;
    }

    switch (gyro_fs)
    {
        case LSM9DS1_GYRO_FS_245DPS:
            gyro_fs_bits = LSM9DS1_GYRO_FS_BITS_245DPS;
            new_gyro_sens = LSM9DS1_GYRO_SENS_245_MDPS_LSB;
            break;
        case LSM9DS1_GYRO_FS_500DPS:
            gyro_fs_bits = LSM9DS1_GYRO_FS_BITS_500DPS;
            new_gyro_sens = LSM9DS1_GYRO_SENS_500_MDPS_LSB;
            break;
        case LSM9DS1_GYRO_FS_2000DPS:
            gyro_fs_bits = LSM9DS1_GYRO_FS_BITS_2000DPS;
            new_gyro_sens = LSM9DS1_GYRO_SENS_2000_MDPS_LSB;
            break;
        default:
            return HAL_ERROR;
    }

    /*
     * 仅修改 FS 位：
     * - CTRL_REG6_XL[4:3] = FS_XL
     * - CTRL_REG1_G [4:3] = FS_G
     * 其余位保持初始化配置，避免误改 ODR/BW/滤波。
     */
    /*
     * new_ctrl_reg*_x 只改 FS 位，其余位继承 shadow，
     * 这样不会破坏 ODR、滤波、BDU 等初始化设定。
     */
    old_ctrl_reg6_xl = s_ctrl_reg6_xl_shadow;
    new_ctrl_reg6_xl = (uint8_t)((s_ctrl_reg6_xl_shadow & (uint8_t)(~LSM9DS1_FS_FIELD_MASK)) | ((accel_fs_bits & 0x03U) << 3));
    new_ctrl_reg1_g = (uint8_t)((s_ctrl_reg1_g_shadow & (uint8_t)(~LSM9DS1_FS_FIELD_MASK)) | ((gyro_fs_bits & 0x03U) << 3));

    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG6_XL, new_ctrl_reg6_xl) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG1_G, new_ctrl_reg1_g) != HAL_OK)
    {
        /* 若第二步失败，回滚第一步，降低配置半成功带来的系统不一致风险 */
        (void)LSM9DS1_WriteReg_Blocking(LSM9DS1_REG_CTRL_REG6_XL, old_ctrl_reg6_xl);
        return HAL_ERROR;
    }

    /* 提交 shadow 与物理量换算系数，确保“寄存器状态”和“换算公式”一致。 */
    s_ctrl_reg6_xl_shadow = new_ctrl_reg6_xl;
    s_ctrl_reg1_g_shadow = new_ctrl_reg1_g;
    s_accel_sens_mg_lsb = new_accel_sens;
    s_gyro_sens_mdps_lsb = new_gyro_sens;

    return HAL_OK;
}

/* 读取最近一次 DMA 解析后的六轴原始值快照 */
void LSM9DS1_GetLatestRaw(LSM9DS1_RawData_t *out_raw)
{
    if (out_raw == NULL)
    {
        return;
    }

    *out_raw = (LSM9DS1_RawData_t)s_latest_raw;
}

float LSM9DS1_AccelRawToG(int16_t raw_val)
{
    /* 文档公式：acc_g = raw * (mg/LSB) / 1000 */
    return ((float)raw_val * s_accel_sens_mg_lsb) / 1000.0f;
}

float LSM9DS1_GyroRawToDps(int16_t raw_val)
{
    /* 文档公式：gyro_dps = raw * (mdps/LSB) / 1000 */
    return ((float)raw_val * s_gyro_sens_mdps_lsb) / 1000.0f;
}

const LSM9DS1_RuntimeState_t *LSM9DS1_GetRuntimeState(void)
{
    /* 返回内部状态只读指针，便于上位机/串口观测链路健康度 */
    return &s_runtime;
}

/*
 * 若工程中尚未实现 SPI 全局回调，可直接使用以下桥接实现。
 * 如果后续存在多个 SPI 设备，可在统一回调中分发到对应设备处理器。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* 当前工程由 LSM9DS1 接管 SPI1 的 TxRx 完成分发。 */
    LSM9DS1_SPI_TxRxCpltHandler(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* 当前工程由 LSM9DS1 接管 SPI1 的错误分发。 */
    LSM9DS1_SPI_ErrorHandler(hspi);
}
