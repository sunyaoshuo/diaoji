# dvc_motor_IK 使用说明

`dvc_motor_IK.h/.cpp` 是按上海瓴控/KMTECH《电机 CAN 总线通讯协议 V2.36》实现的单电机 CAN 驱动，接口风格参考 `dvc_motor_dm`，直接复用项目中的 `drv_can`。

## 1. 协议和默认约定

- 标准 CAN 数据帧，DLC 固定 8 字节。
- 发送与回复 ID 均为 `0x140 + Motor_ID`，`Motor_ID` 范围 `1~32`。
- 常规模式默认波特率为 1 Mbps；总线实际波特率仍由 CubeMX/FDCAN 初始化和电机参数共同决定。
- 对外物理量统一为：角度 `rad`、角速度 `rad/s`、电流 `A`、电压 `V`、温度 `K`。
- 协议把顺时针定义为正方向，驱动不额外翻转符号。
- `Control_Enable` 默认关闭，驱动不会在初始化或掉线时自动使能电机；需要显式调用 `CAN_Send_Motor_Run()` 和 `Set_Control_Enable(true)`。

电流分辨率默认值：

- MF/MH/MHF：`33/4096 A/LSB`
- MG：`66/4096 A/LSB`
- MS：状态 2 的对应字段按输出功率原始值解析，不换算为电流

协议只明确给出了 MF、MG 的电流分辨率，因此 MH/MHF 暂按 MF 处理。若具体型号不同，可通过 `Init()` 的第五个参数传入自定义 `A/LSB`。

## 2. 最小接入示例

```cpp
#include "1_Middleware/Device/Motor/dvc_motor_IK.h"

Class_Motor_IK Motor_IK;

void CAN1_Rx_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    // 可直接把同一总线的所有帧交给对象；对象内部会按 0x140 + ID 过滤。
    Motor_IK.CAN_RxCpltCallback(Header, Buffer);
}

void Motor_Init()
{
    CAN_Init(&hfdcan1, CAN1_Rx_Callback);

    // ID=1，MF 系列，14 bit 编码器。
    Motor_IK.Init(&hfdcan1, 1, Motor_IK_Series_MF, 14);

    Motor_IK.CAN_Send_Motor_Run();

    // 速度闭环：5 rad/s，转矩电流限制 4 A。
    Motor_IK.Set_Control_Method(Motor_IK_Control_Method_SPEED);
    Motor_IK.Set_Control_Omega(5.0f);
    Motor_IK.Set_Control_Current_Limit(4.0f);
    Motor_IK.Set_Control_Enable(true);
}

void TIM_1ms_Motor_Callback()
{
    Motor_IK.TIM_Send_PeriodElapsedCallback();
}

void TIM_100ms_Motor_Callback()
{
    Motor_IK.TIM_100ms_Alive_PeriodElapsedCallback();
}
```

同一条 CAN 总线上有多个电机时，可在总回调中依次调用多个对象的 `CAN_RxCpltCallback(Header, Buffer)`；每个对象会自行检查 ID。

## 3. 常用控制

### 转矩电流控制

```cpp
Motor_IK.Set_Control_Method(Motor_IK_Control_Method_TORQUE);
Motor_IK.Set_Control_Current(3.0f);  // A
Motor_IK.Set_Control_Enable(true);
```

也可一次性发送：

```cpp
Motor_IK.CAN_Send_Torque_Control(3.0f);
```

### 多圈位置控制

```cpp
Motor_IK.Set_Control_Method(Motor_IK_Control_Method_MULTI_TURN_ANGLE_2);
Motor_IK.Set_Control_Angle(3.1415926f);       // rad
Motor_IK.Set_Control_Max_Omega(6.0f);         // rad/s
Motor_IK.Set_Control_Enable(true);
```

### 单圈位置控制

```cpp
Motor_IK.Set_Control_Method(Motor_IK_Control_Method_SINGLE_TURN_ANGLE_2);
Motor_IK.Set_Control_Angle(1.5707963f);       // rad
Motor_IK.Set_Control_Max_Omega(4.0f);         // rad/s
Motor_IK.Set_Control_Spin_Direction(Motor_IK_Spin_Direction_CLOCKWISE);
Motor_IK.Set_Control_Enable(true);
```

### 增量位置控制

增量命令表示“每收到一帧就再移动一次”，因此驱动把周期输出实现为一次性触发：每次调用 `Set_Control_Angle_Increment()` 后，只会发送一帧。

```cpp
Motor_IK.Set_Control_Method(Motor_IK_Control_Method_INCREMENT_ANGLE_2);
Motor_IK.Set_Control_Max_Omega(3.0f);
Motor_IK.Set_Control_Angle_Increment(0.5f);   // rad，同时置位一次发送请求
Motor_IK.Set_Control_Enable(true);
```

需要再次增量运动时，再调用一次 `Set_Control_Angle_Increment()`。

## 4. 状态读取

控制命令 `0xA0~0xA8` 的回复均按“状态 2”解析，因此持续控制时可直接通过以下接口读取最新反馈：

```cpp
float omega = Motor_IK.Get_Now_Omega();
float current = Motor_IK.Get_Now_Current();
float temperature_k = Motor_IK.Get_Now_Temperature();
float temperature_c = Motor_IK.Get_Now_Temperature_Celsius();
uint16_t encoder = Motor_IK.Get_Now_Encoder();
```

状态 1 包含母线电压、母线电流、开启/关闭状态和错误位：

```cpp
Motor_IK.CAN_Send_Read_Status_1();

bool over_current = Motor_IK.Get_Error_Flag(Motor_IK_Error_OVERCURRENT);
float bus_voltage = Motor_IK.Get_Bus_Voltage();
```

完整接收数据可通过 `Get_Rx_Data()` 获取。

## 5. 参数读写

RAM 控制参数写入后立即生效，断电失效：

```cpp
Motor_IK.CAN_Send_Write_Control_PID(
    Motor_IK_Control_Parameter_SPEED_PID,
    100, 20, 0);

Motor_IK.CAN_Send_Read_Control_Parameter(
    Motor_IK_Control_Parameter_SPEED_PID);

const Struct_Motor_IK_Control_Parameter_Data &p =
    Motor_IK.Get_Control_Parameter_Data();
```

设定参数写入后还需要保存并重启：

```cpp
Motor_IK.CAN_Send_Write_Single_Setting_U8(
    Motor_IK_Setting_Parameter_CAN_BAUDRATE,
    Motor_IK_CAN_Baudrate_1M);

Motor_IK.CAN_Send_Save_Setting_Parameters();
// 等收到保存成功回复后：
Motor_IK.CAN_Send_Restart();
```

更改电机 ID 后，重启前后的回复 ID 可能发生变化。建议确认保存成功，重启后用新 ID 重新调用 `Init()`。

## 6. 协议细节处理

- 多圈角度回复 `0x92` 虽然文档写为 `int64_t`，但 CAN 负载中只有 `DATA[1:7]` 七个角度字节。本驱动按 56 位有符号数解析并做符号扩展，原始值保存在 `Multi_Turn_Angle_Raw`。
- 增量位置命令 2 的正文把 `maxSpeed` 写为 `uint32_t`，但报文只分配了 `DATA[2:3]` 两字节。本驱动按报文表实现为 `uint16_t`、单位 `1 dps/LSB`。
- 校准编码器、永久零点和保存设定参数会写 ROM，不应高频调用。
- 编码器校准可能持续数秒，这段时间 100 ms 在线检测可能暂时显示离线；应以校准回复和实际流程为准。
