# dvc_MVR2TB

`dvc_MVR2TB` 是面向 STM32 HAL 与现有 `drv_uart` 的 MOEVISION MVR2TB TOF 激光测距模组驱动。接口组织方式与项目中的 `dvc_motor_dm`、`dvc_motor_IK`、`dvc_motor_RS` 类似：设备类负责协议收发、数据换算、在线检测和状态保存，底层 UART 初始化、DMA 接收及发送继续复用 `drv_uart`。

## 文件

- `dvc_MVR2TB.h`：类型、类接口、Getter、命令定义
- `dvc_MVR2TB.cpp`：流式解析、命令组帧、数据换算和在线检测
- `dvc_MVR2TB_README.md`：接入与使用说明

主类：

```cpp
Class_MVR2TB
```

## 通信配置

在 CubeMX 中将连接模组的 UART 配置为：

```text
Baud rate : 115200 bit/s
Data bits : 8
Stop bits : 1
Parity    : None
Level     : LVTTL 3.3V
```

接线时需要交叉连接串口：

```text
MVR2TB UART_TX -> MCU UART_RX
MVR2TB UART_RX -> MCU UART_TX
MVR2TB GND     -> MCU GND
MVR2TB VCC     -> +5V
```

模组 UART 引脚是 3.3V LVTTL，不能直接接入高于 3.3V 的 UART 信号。

## 已实现功能

驱动覆盖数据手册中的全部用户命令：

- 单次/连续测量模式选择
- 开始连续测量、停止连续测量、单次测量
- HEX/ASCII 输出格式切换
- 进入待机和单字节串口唤醒
- 读取固件版本、读取硬件版本
- 上电自动测量开关
- 10Hz、20Hz、50Hz、100Hz 测量频率设置
- 读取 IO 距离阈值、设置 IO 距离阈值
- 自定义命令发送

接收侧支持：

- 固定 11 字节 HEX 测量帧
- 通用 `0x89 + CMD + LEN + CONTENT + XOR` 响应帧
- 逗号分隔、回车结尾的 ASCII 测量数据
- DMA 空闲中断造成的任意分包
- 一次回调中包含多帧数据的粘包
- 校验失败后的帧头重新同步
- HEX 与 ASCII 数据自动识别，不依赖 UART 回调边界

## 最小接入示例

```cpp
#include "dvc_MVR2TB.h"

Class_MVR2TB MVR2TB;

void UART6_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
    MVR2TB.UART_RxCpltCallback(Buffer, Length);
}

void MVR2TB_Init()
{
    // drv_uart 使用 ReceiveToIdle DMA，并在空闲中断中调用上面的回调。
    UART_Init(&huart6, UART6_Rx_Callback);

    // 参数：UART 句柄、离线超时的 100ms 周期数、上电输出格式先验值。
    // 默认 5 个周期，即连续 500ms 左右无有效帧后判定离线。
    MVR2TB.Init(&huart6, 5, MVR2TB_Output_Format_HEX);

    // 模组出厂默认通常会自动连续测量，驱动初始化后即可解析数据。
    // 需要主动启动时可发送：
    MVR2TB.UART_Send_Start_Continuous_Measurement();
}
```

不要把 `UART_RxCpltCallback()` 当作“一次回调恰好一帧”。驱动内部已经保存未完成数据，应用层应把 `drv_uart` 提供的整段 `Buffer` 和 `Length` 原样传入。

## 获取测量数据

```cpp
void Sensor_Task()
{
    if (MVR2TB.Get_Measurement_Updated())
    {
        uint16_t Distance_MM = MVR2TB.Get_Now_Distance_MM();
        float Distance_M = MVR2TB.Get_Now_Distance();
        uint16_t Amplitude = MVR2TB.Get_Now_Signal_Amplitude();
        float Temperature_C = MVR2TB.Get_Now_Temperature_Celsius();
        float Temperature_K = MVR2TB.Get_Now_Temperature();
        uint16_t Ambient = MVR2TB.Get_Now_Ambient_Light();
        uint16_t Illumination = MVR2TB.Get_Now_Illumination_DAC();

        // 处理完本次更新后清除软件标志。
        MVR2TB.Clear_Measurement_Updated();
    }
}
```

也可以直接读取完整结构体：

```cpp
const Struct_MVR2TB_Measurement_Data &Data =
    MVR2TB.Get_Measurement_Data();
```

主要字段：

```cpp
Data.Valid
Data.Source_Format
Data.Distance_MM
Data.Distance_Meter
Data.Signal_Amplitude
Data.Temperature_Celsius
Data.Temperature_Kelvin
Data.Ambient_Light
Data.Illumination_DAC
Data.Illumination_Status
Data.Module_ID
Data.Sequence
```

`Sequence` 每收到一帧有效测量数据递增。对不能丢更新的任务，建议记录上一次 `Sequence`，而不是只依赖布尔更新标志。

## HEX 测量帧

协议的固定测量帧为：

```text
Byte0  0x89
Byte1  0x81
Byte2  DIST_L
Byte3  DIST_H
Byte4  AMP_L
Byte5  AMP_H
Byte6  TEMP
Byte7  AMB_L
Byte8  AMB_H
Byte9  ILLB
Byte10 XOR
```

驱动按小端解析距离、信号幅度和环境光；HEX 温度按有符号 8 位摄氏度解析，以覆盖数据手册给出的负温工作区间。

## ASCII 测量帧

数据手册表格给出的格式带模组 ID，例如：

```text
M11422,02439,05000,04737,075,00000\r
```

驱动也兼容 GUI 截图中出现的不带 ID 五字段格式：

```text
01220,03759,41,0068,1,\r
```

兼容规则：

- 首字段以英文字母开头时，将其保存为 `Module_ID`。
- 温度字段带小数点时直接按小数解析。
- 温度为 4 位及以上固定数字时按 `0.01°C/LSB` 解析，例如 `04737 -> 47.37°C`。
- 较短温度字段按整摄氏度解析，例如 `41 -> 41°C`。
- 行尾支持 `CR` 或 `CRLF`。

## 常用命令

### 开始和停止连续测量

```cpp
MVR2TB.UART_Send_Start_Continuous_Measurement();
MVR2TB.UART_Send_Stop_Continuous_Measurement();
```

### 单次测量

```cpp
MVR2TB.UART_Send_Single_Measurement();
```

单次模式下，模组以测量数据帧作为本次命令的完成结果，不另外发送普通 ACK。驱动收到下一帧有效测量数据后，会将：

```cpp
MVR2TB.Get_Last_Acknowledged_Command()
```

更新为 `0x02`。

### 设置初始化测量模式

```cpp
MVR2TB.UART_Send_Select_Measurement_Mode(
    MVR2TB_Measurement_Mode_CONTINUOUS
);
```

可选值：

```cpp
MVR2TB_Measurement_Mode_SINGLE
MVR2TB_Measurement_Mode_CONTINUOUS
```

该设置会使模组自动重启。`0x20` 和 `0x21` 的响应命令字都固定为 `0x50`，驱动会根据最近待确认命令反向匹配并更新测量模式。

### 切换 HEX/ASCII 输出

```cpp
MVR2TB.UART_Send_Set_Output_Format(MVR2TB_Output_Format_HEX);
MVR2TB.UART_Send_Set_Output_Format(MVR2TB_Output_Format_ASCII);
```

该设置只对当前运行有效，重启后模组恢复默认 HEX 输出。接收解析器始终同时支持两种格式，因此即使应用层保存的格式状态与模组暂时不一致，也可以重新同步。

### 待机和唤醒

```cpp
MVR2TB.UART_Send_Enter_Standby();

// 待机模组收到任意一个串口字节即可唤醒，默认发送 0x00。
MVR2TB.UART_Send_Wake_Up();
```

待机命令仅在单次测量模式下有效。唤醒后需要留出模组恢复时间，再发送单次测量命令；驱动不在中断或发送函数中加入阻塞延时。

### 读取版本号

```cpp
MVR2TB.UART_Send_Read_Firmware_Version();
MVR2TB.UART_Send_Read_Hardware_Version();
```

读取结果：

```cpp
const char *Firmware = MVR2TB.Get_Firmware_Version();
const char *Firmware_Raw = MVR2TB.Get_Firmware_Version_Raw();

const char *Hardware = MVR2TB.Get_Hardware_Version();
const char *Hardware_Raw = MVR2TB.Get_Hardware_Version_Raw();
```

例如内容字节为 `M107` 时，驱动格式化为 `M1.0.7`，同时保留原始字符串 `M107`。数据手册的表格、十六进制样例和文字解析之间存在版本号书写差异，现场判断时应优先查看 `Raw`。

### 设置测量频率

```cpp
MVR2TB.UART_Send_Set_Measurement_Frequency(
    MVR2TB_Measurement_Frequency_50HZ
);
```

可选值：

```cpp
MVR2TB_Measurement_Frequency_10HZ
MVR2TB_Measurement_Frequency_20HZ
MVR2TB_Measurement_Frequency_50HZ
MVR2TB_Measurement_Frequency_100HZ
```

频率设置重启后生效。

### 上电自动测量

```cpp
MVR2TB.UART_Send_Set_Power_On_Auto_Measurement(true);
MVR2TB.UART_Send_Set_Power_On_Auto_Measurement(false);
```

设置在重新上电或重启后生效，并且在单次测量模式下无效。

### IO 距离阈值

读取：

```cpp
MVR2TB.UART_Send_Read_IO_Threshold();

if (MVR2TB.Get_IO_Threshold_Valid())
{
    uint16_t Threshold_MM = MVR2TB.Get_IO_Threshold_MM();
}
```

设置为 2500mm：

```cpp
MVR2TB.UART_Send_Set_IO_Threshold(2500);
```

驱动生成的帧为：

```text
56 DA 04 C4 09 00 00 45
```

距离小于阈值时 IO 为高电平，距离大于阈值时 IO 为低电平。

## 命令返回值与 DMA 缓冲区

所有发送函数返回 `drv_uart` 中 `UART_Transmit_Data()` 的 HAL 状态值：

```cpp
HAL_OK
HAL_ERROR
HAL_BUSY
HAL_TIMEOUT
```

`HAL_UART_Transmit_DMA()` 在发送完成前会持续访问传入缓冲区。驱动没有把临时栈数组交给 DMA，而是使用两个类内持久发送缓冲区轮换发送，因此不会因函数返回造成悬空指针。

同一个 UART 上仍然只允许一个 DMA 发送事务。当前发送未结束时再次调用命令函数，底层通常返回 `HAL_BUSY`；应用层应稍后重试，不要高频无间隔发送配置命令。

## 在线检测

在 100ms 定时器中调用：

```cpp
void TIM_100ms_Callback()
{
    MVR2TB.TIM_100ms_Alive_PeriodElapsedCallback();
}
```

读取状态：

```cpp
if (MVR2TB.Get_Status() == MVR2TB_Status_ENABLE)
{
    // 最近收到过通过校验的测量帧或命令响应。
}
```

默认离线超时为 5 个 100ms 周期。单次测量、停止测量或待机状态下没有周期数据，在线状态会自然超时；这表示“最近没有有效串口帧”，不等同于模组硬件一定断电。

## 响应和解析统计

最近一次通用响应：

```cpp
const Struct_MVR2TB_Response_Data &Response =
    MVR2TB.Get_Response_Data();
```

字段包括：

```cpp
Response.Valid
Response.Result
Response.Command
Response.Matched_Tx_Command
Response.Content_Length
Response.Content
Response.Checksum
Response.Sequence
```

解析统计：

```cpp
const Struct_MVR2TB_Parser_Statistics &Statistics =
    MVR2TB.Get_Parser_Statistics();
```

可用于排查：

- 有效 HEX 测量帧数量
- 有效 ASCII 测量行数量
- 有效命令响应数量
- 兼容数据手册测量样例校验方式的数量
- XOR 校验错误
- 帧格式错误
- 二进制或 ASCII 重组缓冲区溢出

清零统计：

```cpp
MVR2TB.Clear_Parser_Statistics();
```

## 数据手册中的不一致及处理方式

### 1. HEX 测量样例校验

手册第 7 页样例：

```text
89 81 74 0A AB 0C 2F 67 00 01 99
```

若对 Byte0 到 Byte9 全部异或，结果是 `0x98`；样例中的 `0x99` 等于只异或 Byte0 到 Byte8，未包含 Byte9。驱动对 HEX 测量帧同时接受这两种校验方式：

- 标准方式：Byte0 到 Byte9 全部 XOR
- 手册样例兼容方式：Byte0 到 Byte8 XOR

通用命令响应仍严格校验全部前置字节。使用样例兼容方式收到的帧会累计到：

```cpp
Statistics.Legacy_Measurement_Checksum_Count
```

### 2. 固件版本响应长度

固件版本表格将 `LEN` 写为 `0x03`，但同页响应样例为：

```text
89 0E 04 4D 31 30 37 F8
```

且确实包含 4 个版本内容字节。驱动按实际帧中的 `LEN` 重组，并在内容长度至少为 4 时生成格式化版本号。

### 3. 设置 IO 阈值命令

手册第 12 页“用户命令”表格中的帧头和长度与第 6 页命令汇总及 2500mm 样例不一致。驱动采用命令汇总和完整样例：

```text
56 DA 04 XL XH 00 00 XOR
```

### 4. 读取阈值响应保留字节

表格称内容后两个字节为保留且为零，但示例响应为：

```text
89 03 04 F4 01 DC 05 A2
```

驱动校验完整帧，但只用内容前两个字节计算阈值，不要求后两个字节必须为零。

## 测试情况

本驱动已完成主机端 C++11 编译和单元测试，编译选项：

```text
-Wall -Wextra -Werror
```

并使用 AddressSanitizer 与 UndefinedBehaviorSanitizer 检查。测试覆盖：

- 所有主要命令的组帧和 XOR
- 2500mm 阈值命令字节序
- DMA 发送缓冲区生命周期及 `HAL_BUSY` 情况
- HEX 测量样例解析
- 两种 HEX 测量校验方式
- 错误校验拒绝和帧头重新同步
- UART 分包与粘包
- 固件、硬件版本解析
- IO 阈值读取和设置确认
- 模式选择特殊响应匹配
- 带 ID 与不带 ID 的 ASCII 数据
- ASCII 整摄氏度和 0.01°C 温度格式
- 单次测量完成确认
- 未知命令响应
- 在线超时

测试结果：

```text
All dvc_MVR2TB tests passed
```

目前尚未在真实 MVR2TB 模组和实际 STM32 UART 总线上完成硬件联调。首次接入建议同时观察 `Parser_Statistics`、原始串口数据和版本号原始字符串。
