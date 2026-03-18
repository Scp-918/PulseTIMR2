# HJ131数据手册
# 1. 通信基础接口 (Communication Interface)
- **协议类型**: UART (透明传输模式/AT指令配置模式)
- **默认波特率**: 19200 bps (出厂默认，宏 `BLE_DEFAULT_BAUD`)
- **目标工作波特率**: 460800 bps (宏 `BLE_TARGET_BAUD`)
- **UART 参数配置**: 8-bit 数据位, 1-bit 停止位, 无校验 (Parity None), 无硬件流控。
- **硬件引脚连接 (高功耗模式 + 外接天线)**:
  - **VCC_HIGH (Pin 2)**: 供电输入，范围 1.8V ~ 3.6V (典型 3.3V)。
  - **VCC_LOW (Pin 1)**: 悬空 (不使用低压模式)。
  - **TX (Pin 8 - P05/W)**: BLE 串口发送引脚，连接 MCU RX。
  - **RX (Pin 9 - P02/C)**: BLE 串口接收引脚 / 唤醒引脚。
  - **RST (Pin 10 - P00/R)**: 硬件复位引脚 (高电平有效，宏 `BLE_RST_PIN`)。
  - **STATE (Pin 11 - P01)**: 连接状态引脚 (宏 `BLE_STATE_PIN`)。
  - **ANT_OUT (Pin 12)**: 射频输出，连接外置天线。
  - **BOARD_ANT (Pin 13)**: 悬空 (外接天线模式下不可与 Pin 12 短接)。

# 2. 核心配置指令集映射表 (Core Command Map)
/* 注：本设备为UART透明传输蓝牙模块，无传统I2C/SPI物理寄存器，使用类AT字符串指令集进行配置 */
#define BLE_CMD_WAKE_BYTES       {0xAA, 0xAA, 0xAA, 0xAA, 0xAA} // [默认唤醒序列] : 连续发送5个0xAA唤醒处于休眠的RX
#define BLE_CMD_SET_BAUD_FMT     "<ST_BAUD=%d>"                 // [波特率设置] : 配置模块通信波特率
#define BLE_CMD_RD_BAUD          "<RD_BAUD>"                    // [波特率读取] : 读取当前波特率
#define BLE_CMD_SET_WAKE_FOREVER "<ST_WAKE=FOREVER>"            // [工作模式设置] : 配置为全速工作模式不休眠
#define BLE_CMD_SET_TX_POWER_MAX "<ST_TX_POWER=+2.5>"           // [发射功率设置] : 设置发射功率为最大值 +2.5dBm
#define BLE_CMD_SET_NAME_FMT     "<ST_NAME=%s>"                 // [广播名称设置] : 设置蓝牙广播名称

#define BLE_RESP_OK              "ok"                           // [标准回复] : 成功配置时的回复后缀/子串
#define BLE_RESP_RD_BAUD         "rd_baud"                      // [波特率读取回复] : 读取波特率时的应答标识

#define BLE_TARGET_BAUD          460800                         // [目标波特率]
#define BLE_DEFAULT_BAUD         19200                          // [出厂波特率]
#define BLE_PREV_BAUD            115200                         // [备用/上一次波特率]

#define BLE_RST_PORT             GPIOE                          // [硬件复位端口]
#define BLE_RST_PIN              GPIO_PIN_1                     // [硬件复位引脚] (对应模块 Pin 10)
#define BLE_STATE_PORT           GPIOE                          // [状态读取端口]
#define BLE_STATE_PIN            GPIO_PIN_0                     // [状态读取引脚] (对应模块 Pin 11)

#define BLE_RST_ACTIVE_LEVEL     GPIO_PIN_SET                   // [复位有效电平] : 高电平复位
#define BLE_RST_INACTIVE_LEVEL   GPIO_PIN_RESET                 // [正常运行电平] : 低电平或悬空

#define BLE_STATE_CONNECTED      1                              // [连接状态] : 蓝牙已连接
#define BLE_STATE_DISCONNECTED   0                              // [连接状态] : 蓝牙未连接

# 4. 状态机与初始化流程 (Initialization Sequence)
1. **硬件复位 (Hardware Reset)**:
   - 动作: 拉高 `BLE_RST_PIN` (Pin 10)。
   - 时序: 保持高电平至少 1ms (驱动中采用 10ms 确保稳定)。
   - 动作: 拉低 `BLE_RST_PIN`。
   - 延时: 等待模块启动，至少延时 600ms。
2. **唤醒与波特率同步 (Baudrate Synchronization)**:
   - 动作: MCU UART 切换至 `BLE_DEFAULT_BAUD` (19200)。
   - 发送: `0xAA 0xAA 0xAA 0xAA 0xAA` 唤醒模块。
   - 延时: 20ms。
   - 发送: `<ST_BAUD=460800>`。
   - 延时: 等待模块处理 (50ms)。
   - 容错: 若无响应，切换 MCU UART 至 `BLE_PREV_BAUD` (115200) 并在该波特率下重复上述唤醒与设置指令。
   - 切换: 将 MCU 本地 UART 切换至 `BLE_TARGET_BAUD` (460800)，延时 50ms。
3. **配置全速模式 (Configure Wake Mode)**:
   - 发送: 唤醒序列 `0xAA` * 5，延时 5ms 确保 RX 就绪。
   - 发送: `<ST_WAKE=FOREVER>`。
   - 校验: 等待超时内(200ms)接收到 "ok"。
4. **校验波特率 (Verify Baudrate)**:
   - 发送: 换行/回车或其他探测报文，等待 "ok"；若失败，发送 `<RD_BAUD>`。
   - 校验: 等待超时内(200ms)接收到 "rd_baud" 确认当前处于正确波特率。
5. **配置射频参数 (Configure RF Parameters)**:
   - 发送: `<ST_TX_POWER=+2.5>`。
   - 校验: 等待超时内(200ms)接收到 "ok"。
6. **配置设备名称 (Configure Device Name)**:
   - 发送: `<ST_NAME=HJ-G474-TEST>` (按需替换名称)。
   - 校验: 等待超时内(200ms)接收到 "ok"。

# 5. 传感参数控制、数据读取与转换逻辑
- **数据透传模式 (Transparent Transmission)**: 
  模块默认采用 SPPv2 固件，所有非指令包裹格式（非 `<...>`）的数据会被直接通过蓝牙链路透传给主机/手机。发送数据调用 `HAL_UART_Transmit` 直接输出字符串或字节流即可。
- **状态监控触发 (State Monitoring)**: 
  无需轮询寄存器。直接读取硬件 GPIO (PE0 / `BLE_STATE_PIN`) 的电平状态判定连接状态。高电平(1)代表已连接，低电平(0)代表断开连接。
- **低功耗唤醒逻辑 (Wakeup Logic)**: 
  如果在后续开发中关闭了 `FOREVER` 模式开启了休眠，唤醒模块需通过拉高 RX 引脚（Pin 9）大于 1ms 或发送 Dummy Bytes (`0xAA`)，模块唤醒后此引脚自动恢复为普通 UART RX 功能。
- **射频性能说明**:
  当采用外接天线且发射功率设置为 +2.5dBm 时，在空旷区域下，无线信号传输距离可达 40~80 米。峰值发射电流约为 3.5mA，峰值接收电流约为 2.2mA。