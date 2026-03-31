# PulseTIMR2 - 高精度多传感器同步采集系统

PulseTIMR2 是基于 **STM32G474** 微控制器开发的高频多传感器同步采集与无线传输系统。项目利用高级定时器（HRTIM）的皮秒级特性驱动模拟开关（TMUX1108）与高速 ADC（AD4007），同时基于无阻塞异步状态机同步采集 PPG（MAX30101）和六轴 IMU（LSM9DS1）数据，并通过 BLE 模块以 DMA 方式全速透传至上位机。

---

## 📑 目录

1. 整个数据的采集-入队-发送流程
2. 项目整体初始化与主循环流程
3. 定时器-传感器通信与参数配置流程
4. 关键数据结构与核心函数
5. 通信协议帧结构
6. 上位机调节参数功能流程与状态机说明
7. 调参与发送策略设计要点

---

## 1. 整个数据的采集-入队-发送流程

系统的数据流采用了 **“中断生产者 -> 环形缓冲区 (RingBuffer) -> 主循环消费者 -> DMA发送”** 的非阻塞管线架构，极大程度避免了高频采样时的数据积压与线程竞争。

```mermaid
graph TD
    subgraph isr ["硬件触发与中断采集 (ISR)"]
        HRTIM[HRTIM Master CMP4] -->|触发| S1(AD4007: 相位内早晚窗求均值)
        HRTIM -->|Phase 1触发| S2(MAX30101 I2C-DMA)
        HRTIM -->|Phase 2触发| S3(LSM9DS1 SPI-DMA)
        
        S2 -->|DMA完成解包| RB[(全局 Sensor RingBuffer)]
        S3 -->|DMA完成解包| RB
    end

    subgraph main_loop ["主循环处理 (Main Loop)"]
        S1 -->|ISR结算直接写入| GF[当前组帧 g_group_frame]
        RB -->|出队合并| GF
        
        GF -->|Phase 4结束提交| PF[一帧延迟缓存 g_tx_prev_frame]
        PF -->|零值修补算法| Pck[帧打包 BLE_PackSingleFrame]
    end

    subgraph dma_tx ["异步发送 (DMA TX)"]
        Pck -->|UART1_DMA 发送| BLE((BLE 模块 460800 Baud))
    end
```

---

## 2. 项目整体初始化与主循环流程

系统在启动时严格控制了外设的上电时序和初始化顺序，进入主循环后保持极简设计，仅负责帧拼接与异步发送。

### 2.1 整体流程图

```mermaid
graph TD
    Start((系统启动)) --> HAL[HAL库及系统时钟初始化]
    HAL --> Peri[配置 GPIO/DMA/I2C/SPI/UART]
    Peri --> Mux[TMUX_Global_Init: 初始化模拟开关]
    Mux --> RB[RingBuffer_Init: 初始化传感器环形缓冲]
    RB --> Pwr[传感器电源上电 E5V -> E3.3V -> E4V]
    Pwr --> BLE_Init[BLE_Init: 模块波特率同步与唤醒]
    BLE_Init --> SensorInit[MAX30101 / LSM9DS1 初始化并绑定同一 RingBuffer]
    SensorInit --> ADC_Init[AD4007_Init: 初始化并丢弃首帧]
    ADC_Init --> Tim_Start[启动 HRTIM 与 TIM1 开启硬件触发]
    
    Tim_Start --> Loop_Start{进入 while 1 主循环}
    Loop_Start --> C1[ADC_TryHarvestPendingSample 尝试回收残留ADC DMA]
    C1 --> C2{g_group_frame_ready_for_send == 1?}
    C2 -- Yes --> C2_1[PrepareAndCommitGroupFrame: 环形缓冲出队与帧融合]
    C2 -- No --> C3
    C2_1 --> C3{g_ble_send_pending == 1?}
    C3 -- Yes --> C3_1[BLE_PackSingleFrame: 组包 49 Bytes]
    C3_1 --> C3_2[BLE_Transmit_Data_DMA: 发送至上位机]
    C3 -- No --> Loop_Start
    C3_2 --> Loop_Start
```

### 2.2 流程说明
1. **上电时序控制**：确保 GPIO 控制的传感器电源（PE7/PE8/PE10）按顺序稳定建立。
2. **传感器异步绑定**：将 `MAX30101` 和 `LSM9DS1` 的数据目标指针挂载到全局 `g_sensor_rb` 环形缓冲，使驱动层与业务层完全解耦。
3. **极简主循环**：主循环被设计为“纯消费者”，依赖 `g_group_frame_ready_for_send` 标志位由 HRTIM 中断驱动。组帧时执行**1帧延迟策略**，当当前帧 PPG 偶尔丢包时，利用相邻历史帧计算均值进行平滑修补。
4. **参数接收与延迟生效**：主循环通过 `BLE_FetchRxFrame()` 从 UART DMA 缓冲提取上位机参数帧，只做校验与排队；真正寄存器写入由 HRTIM 固定相位（Phase1/Phase2）执行，避免异步写寄存器破坏采样节拍。

---

## 3. 定时器-传感器通信与参数配置流程

该项目核心突破在于使用 **TIM1 级联 HRTIM1 构造的微秒级 4 相位状态机**，确保了 ADC 高速采集与低速 PPG/IMU 采集的节拍完全对齐。

### 3.1 传感器参数与时序工作流

```mermaid
sequenceDiagram
    participant T1 as TIM1 (400Hz)
    participant HR as HRTIM1 (Master)
    participant HA as HRTIM1 (Timer A)
    participant MUX as TMUX1108
    participant ADC as AD4007 (SPI3)
    participant PPG as MAX30101 (I2C3)
    participant IMU as LSM9DS1 (SPI1)

    Note over PPG: 初始化: Multi-LED轮切 (G->R->IR)<br/>SMP_AVE=4, LED_PW=215us, 100Hz
    Note over IMU: 初始化: ODR=119Hz, ±2g, ±500dps<br/>关闭中断, 开启BDU
    Note over ADC: 初始化: Turbo Off, High-Z On
    
    loop 4-Phase State Machine
        T1->>HR: 同步节拍触发
        HR->>MUX: 切换桥臂 (Phase 1/2/3/4)
        
        rect rgb(240, 248, 255)
            Note over HR, HA: 相位内部操作 (每相位均发生)
            HA->>ADC: CMP1/3 产生 3+3 (Early/Late) CNV 下降沿
            ADC-->>HA: SPI3 DMA 单次读取 3 字节
        end
        
        alt 切换前是 Phase 1
            HR->>PPG: I2C_IT 发起读写指针
            PPG-->>RB: 指针回调验证样本数 -> I2C_DMA 读取压入队列
        else 切换前是 Phase 2
            HR->>IMU: SPI1_DMA 发起 13 字节连续读取
            IMU-->>RB: SPI 回调解包压入队列
        else 切换前是 Phase 4
            HR->>Main: 置位 g_group_frame_ready_for_send
        end
    end
```

### 3.2 传感器配置参数提炼
* **MAX30101 (PPG)**: 
  * 工作模式：`MULTI_LED_MODE`（时隙为 Green -> Red -> IR）。
  * 核心指标：SPO2 SR=800sps 结合 8x 平均（等效100Hz），LED 脉宽 `215us`（17位分辨率）。
  * 读策略：中断非阻塞触发 (`TriggerPointerRead_IT`) -> 判断 `(WR+32-RD)%32` 算得可用样本 -> DMA 直接抽取 FIFO 压入 `RingBuffer`。
* **LSM9DS1 (IMU)**: 
  * 工作模式：关闭 FIFO 与硬件 DRDY 中断，开启 `BDU`（区块数据更新）和地址自增。
  * 核心指标：ODR=`119Hz`，加速度 `±2g`，角速度 `±500dps`。
  * 读策略：在 Phase 2 通过 SPI1 DMA 发送 1 字节命令读取 12 字节六轴数据。
* **AD4007 (ADC)**: 
    * 核心指标：18-bit，SPI **Mode0**（CPOL=0/CPHA=0，代码中对应 `SPI_PHASE_1EDGE`），SPI 时钟高分频 (`DIV4`)。
  * 读策略：每次产生 CNV 下降沿后触发 SPI3 RxTx DMA，`AD4007_DecodeOneSample` 提取低 18 位并做符号位扩展 (`int32_t`)。

### 3.3 新增：上位机参数下发到传感器生效的时序

新增调参能力采用“两段式提交”策略：
1. **接收与校验（主循环）**：在 BLE RX DMA 回调产生的新帧中，主循环滑窗查找 `AB CD ... EF FA`，并逐字段做范围校验。
2. **挂起待应用（主循环）**：校验通过后仅置位 `g_sensor_param_pending`，并缓存参数到 `g_sensor_param_pending_buf`。
3. **Phase1 应用 PPG 参数（HRTIM 回调）**：在 `HAL_HRTIM_Compare4EventCallback()` 的 phase1 分支写 MAX30101 寄存器，同时进入“配置应用周期”。
4. **Phase2 应用 IMU 参数（HRTIM 回调）**：在 phase2 分支写 LSM9DS1 量程，随后清 `pending` 并启动 `SENSOR_SEND_HOLD_CYCLES=100` 禁发倒计时。
5. **Phase4 提交零帧（HRTIM -> 主循环）**：配置应用周期内强制本组帧为全零，并通过一次性流水线清空，确保旧参数残留不会被发出。
6. **禁发窗口结束后恢复输出**：禁发窗口内继续正常采样和缓存刷新，仅暂停对外发送；倒计时归零后自动恢复 BLE 发包。

---

## 4. 关键数据结构与核心函数

项目对数据流内存做了精心设计，避免 ISR 与主循环争抢资源。

### 4.1 核心数据结构

**1. 传感器数据融合帧 (`SensorDataFrame_t`)**
承载大周期一帧的完整快照，包含四个相位的早期/晚期 ADC 均值及辅助传感器数据。
```c
typedef struct {
    /* 4 个相位的 ADC early 与 late 采样均值 */
    struct {
        int32_t early_code;
        int32_t late_code;
    } adc_data[4];

    /* PPG 三路光数据：Green, Red, IR */
    uint32_t ppg_data[3];

    /* IMU 六轴原始数据：Gx, Gy, Gz, Ax, Ay, Az */
    int16_t imu_data[6];
} SensorDataFrame_t;
```

**2. 线程安全环形缓冲区 (`SensorRingBuffer_t`)**
专为中断生产者和主循环消费者打造。当满载时主动丢弃老数据，保证实时性 (`next_tail` 推进)。
```c
typedef struct {
    SensorDataFrame_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} SensorRingBuffer_t;
```

### 4.2 核心业务函数

| 文件 | 函数名 | 职责 |
| :--- | :--- | :--- |
| `main.c` | `ADC_FinalizeStateAndRotateBridge()` | HRTIM 节拍关键点：结算本相位 3 个早窗/晚窗的 ADC 累加平均值，更新给 `g_group_frame`，并切至下一 TMUX 桥臂状态。 |
| `main.c` | `PrepareAndCommitGroupFrame()` | 出队 RingBuffer 中的最新 PPG/IMU，执行一帧延迟机制及**相邻帧补零算法**（`PatchZeroPPGWithNeighborAverage`），确保蓝牙吐出的时序平滑不断层。 |
| `main.c` | `SensorParam_ParseFrame()` | 对上位机 13 字节参数帧执行帧头帧尾检测、字段范围校验与模式一致性校验（非 Multi 模式强制 `sub-mode=0x01`）。 |
| `main.c` | `SensorParam_ApplyPPG()/SensorParam_ApplyIMU()` | 将协议字段映射为 MAX30101/LSM9DS1 寄存器配置，按固定相位原子生效。 |
| `AD4007.c` | `AD4007_DecodeOneSample()` | 解析 3 Byte 原始流：取低 18 位，若 `bit17==1` 则用 `AD4007_SIGN_EXTEND_MASK` 执行**补码符号扩展**。 |
| `MAX30101.c` | `MAX30101_PointerRxCpltCallback()` | 利用 FIFO 环形回绕计算可用样本，动态设置 DMA 长度，彻底告别阻塞拉取，不卡死系统主时钟。 |
| `LSM9DS1.c` | `LSM9DS1_SetFullScale()` | 仅更新 FS 位，保留 ODR/BW/滤波配置，并在失败时回滚，确保运行时调参一致性。 |

---

## 5. 通信协议帧结构

上位机通信使用 `USART1` DMA 全速透传，波特率 `460800`。单帧协议定长 **49 字节**，结构紧凑且含校验。

| 偏移 (Byte) | 长度 | 字段名称 | 序列化说明 (全部为小端序) |
| :--- | :--- | :--- | :--- |
| `0-1` | 2 | 帧头 | 固定标识 `0xAA 0x55` |
| `2-25` | 24 | ADC 区 | 4通道 × (Early 3B + Late 3B)。18位有符号数值使用 24 位小端传输 |
| `26-34` | 9 | PPG 区 | 3通道 (Green, Red, IR) × 3B。原始无符号数值，右移 1 位映射 |
| `35-46` | 12 | IMU 区 | 6通道 (Gx, Gy, Gz, Ax, Ay, Az) × 2B (`int16_t`) |
| `47` | 1 | 校验和 | 从 Byte 2 开始至 Byte 46 的逐字节**异或和 (XOR)** |
| `48` | 1 | 帧尾 | 固定标识 `0xCC` |

### 5.1 新增：上位机参数配置帧（13 Bytes）

系统支持上位机通过 BLE/UART 发送 13 字节控制帧动态调参：

| 偏移 | 长度 | 字段 | 说明 |
| :--- | :--- | :--- | :--- |
| `0-1` | 2 | 帧头 | 固定 `0xAB 0xCD` |
| `2` | 1 | PPG 模式 | `01=MultiLED`, `02=HR`, `03=SpO2` |
| `3` | 1 | Multi 子模式 | `01=G-R-IR`, `02=G`, `03=R`, `04=IR`, `05=R-IR` |
| `4-6` | 3 | LED 档位 | Green/Red/IR，范围 `00~09`，线性映射到 `0x00~0xFF` 电流码 |
| `7` | 1 | PPG ADC Range | `01~04`，映射 MAX30101 `RGE` |
| `8` | 1 | PPG 脉宽 | `01~04`，映射 `LED_PW`，并联动 SR/SMP_AVE 组合 |
| `9` | 1 | Gyro FS | `01=245dps`, `02=500dps`, `03=2000dps` |
| `10` | 1 | Accel FS | `01=2g`, `02=4g`, `03=8g`, `04=16g` |
| `11-12` | 2 | 帧尾 | 固定 `0xEF 0xFA` |

补充约束：
- 当 `mode != 0x01`（非 Multi 模式）时，`sub-mode` 必须为 `0x01`，否则丢弃该帧。
- 参数写入失败不会阻塞主状态机，只会置 `g_sensor_cfg_apply_error` 错误标记，保证采样节拍连续。

---

## 6. 上位机调节参数功能流程与状态机说明

```mermaid
flowchart TD
    RX[BLE UART DMA 收到字节流] --> F1[主循环 BLE_FetchRxFrame]
    F1 --> F2{SensorParam_ParseFrame 校验通过?}
    F2 -- No --> Drop[丢弃无效帧]
    F2 -- Yes --> Q[写入 pending_buf 并置 g_sensor_param_pending]

    Q --> P1[Phase1: 应用 MAX30101 参数]
    P1 --> P2[置 g_sensor_cfg_apply_cycle_active=1\n清空发送流水线]
    P2 --> P3[Phase2: 应用 LSM9DS1 FS]
    P3 --> P4[清 pending 并置禁发倒计时=100]
    P4 --> P5[Phase4: 提交全零组帧]
    P5 --> Hold[禁发窗口: 仅采样不发送]
    Hold --> Resume[倒计时归零后自动恢复发送]
```

流程解读：
1. **解析与应用解耦**：参数解析放在主循环，寄存器写入放在 HRTIM 固定相位，避免 I2C/SPI 写寄存器插入随机时刻。
2. **一次性清流水线**：`g_sensor_drop_pipeline_once` 会清空历史一帧缓存，防止切参前历史帧穿透到新参数窗口。
3. **配置应用周期全零化**：设置 `g_sensor_cfg_apply_cycle_active` 后，该大周期内 ADC/PPG/IMU 均可被强制记零，确保时域边界清晰。
4. **禁发不禁采**：`g_sensor_send_hold_countdown` > 0 时暂停 BLE 输出，但内部仍持续采样并刷新缓存，待稳定后恢复发包。

---

## 7. 调参与发送策略设计要点

1. **先稳态再输出**：传感器参数切换后保留 100 个组周期禁发，避免将过渡态样本直接送上位机。
2. **节拍优先**：即便写寄存器失败，也不打断 HRTIM 4 相位推进，实时性优先于单次调参成功率。
3. **防跨配置污染**：通过清 ringbuffer + 清一帧延迟管线，保证“旧参数数据不会混入新参数窗口”。
4. **协议可扩展**：13 字节帧采用固定头尾和字段化编码，后续可在中间字段扩展滤波、增益、输出率等策略。
