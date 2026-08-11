# dvc_motor_RS

`dvc_motor_RS` 是面向 RobStride/灵足 EDULITE 05（EL05）电机的 C++11 驱动库，代码组织和调用方式仿照工程中的 `dvc_motor_dm`、`dvc_motor_IK`。本库实现《EL05 使用说明书 260713》第 4 章的 RobStride 私有 CAN 协议。

## 1. 协议范围

驱动按以下协议格式实现：

- CAN 2.0 Classic CAN
- 默认波特率：1 Mbps
- 29 位扩展数据帧
- DLC：8 字节
- 扩展 ID：`bit28~24 = 通信类型`、`bit23~8 = 数据区 2`、`bit7~0 = 目标地址`

已实现通信类型：

- `0x00` 获取设备 ID 和 64 位 MCU UID
- `0x01` 运控模式五参数控制
- `0x02` 电机反馈和版本号应答
- `0x03` 电机使能
- `0x04` 停止、清故障、版本读取
- `0x06` 设置机械零位
- `0x07` 修改电机 CAN ID
- `0x11` 单参数读取
- `0x12` 单参数写入
- `0x15` 详细故障读取与反馈
- `0x16` 参数保存
- `0x17` 波特率修改
- `0x18` 主动上报控制和主动上报反馈
- `0x19` 协议切换

本库不实现 CANopen 和 MIT 控制栈，只提供从私有协议切换到对应协议的命令。

## 2. 文件

- `dvc_motor_RS.h`
- `dvc_motor_RS.cpp`
- `drv_can_RS_extended_frame.patch`
- `drv_can_RS_compatible.cpp`
- `test/test_dvc_motor_RS.cpp`

## 3. 必须处理扩展帧接收

用户提供的 `drv_can.cpp` 中，全局过滤器会拒绝所有未匹配的扩展帧，因此电机可以发送但无法接收。推荐直接应用补丁：

```bash
patch -p1 < drv_can_RS_extended_frame.patch
```

补丁只把未匹配扩展数据帧改为进入 FIFO0，标准帧和现有标准帧电机逻辑不变，远程帧仍然拒绝。

也可以直接用 `drv_can_RS_compatible.cpp` 替换原 `drv_can.cpp`。

另一种方式是不修改 `drv_can.cpp`，在 CubeMX 中把对应 FDCAN 的 `Ext Filters Nbr` 设置为至少 1，并在 `CAN_Init()` 之前调用：

```cpp
Class_Motor_RS::CAN_Config_Extended_Filter(&hfdcan1);
CAN_Init(&hfdcan1, CAN1_Rx_Callback);
```

两种方式选择一种即可。应用全局过滤器补丁后，不需要再调用 `CAN_Config_Extended_Filter()`。

## 4. CAN 回调和初始化

```cpp
#include "dvc_motor_RS.h"

Class_Motor_RS Motor_RS;

void CAN1_Rx_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    // 同一 CAN 总回调可以继续分发给其他电机对象
    Motor_RS.CAN_RxCpltCallback(Header, Buffer);
}

void Motor_RS_Init()
{
    // 使用补丁后的 drv_can
    CAN_Init(&hfdcan1, CAN1_Rx_Callback);

    // 电机 CAN ID = 1，主机 ID = 0xFD
    Motor_RS.Init(&hfdcan1, 1, 0xFD);
}
```

`Init()` 默认电机 ID 为 `1`、主机 ID 为 `0xFD`。说明书报文示例中常用电机 ID `0x7F`，实际值应与电机配置一致。

## 5. 周期回调

```cpp
void TIM_1ms_Motor_Callback()
{
    Motor_RS.TIM_Send_PeriodElapsedCallback();
}

void TIM_100ms_Motor_Callback()
{
    Motor_RS.TIM_100ms_Alive_PeriodElapsedCallback();
}
```

只要 100 ms 检测窗口内收到该电机的有效应答或反馈，`Get_Status()` 就返回 `Motor_RS_Status_ENABLE`。

## 6. 运控模式

运控模式的指令为：

`torque_ref = Kd * (velocity_set - velocity_actual) + Kp * (position_set - position_actual) + torque_feedforward`

```cpp
Motor_RS.Set_Control_Method(Motor_RS_Control_Method_MOTION);
Motor_RS.Set_Control_Angle(0.0f);   // rad
Motor_RS.Set_Control_Omega(1.0f);   // rad/s
Motor_RS.Set_Control_K_P(0.0f);     // 0~500
Motor_RS.Set_Control_K_D(1.0f);     // 0~5
Motor_RS.Set_Control_Torque(0.0f);  // N.m，-6~6

// 内部依次发送：停止 -> 写 run_mode -> 使能
Motor_RS.CAN_Send_Start_Control();
```

此后在发送定时器中周期调用 `TIM_Send_PeriodElapsedCallback()`。

## 7. 电流、速度和位置模式

### 电流模式

```cpp
Motor_RS.Set_Control_Method(Motor_RS_Control_Method_CURRENT);
Motor_RS.Set_Control_Current(2.0f); // A，驱动会限制到 -11~11 A
Motor_RS.CAN_Send_Start_Control();
```

### 速度模式

```cpp
Motor_RS.Set_Control_Method(Motor_RS_Control_Method_SPEED);
Motor_RS.Set_Control_Current_Limit(5.0f); // A
Motor_RS.Set_Control_Acceleration(20.0f); // rad/s^2
Motor_RS.Set_Control_Omega(4.0f);         // rad/s
Motor_RS.CAN_Send_Start_Control();
```

### CSP 位置模式

```cpp
Motor_RS.Set_Control_Method(Motor_RS_Control_Method_POSITION_CSP);
Motor_RS.Set_Control_Speed_Limit(5.0f);   // rad/s
Motor_RS.Set_Control_Current_Limit(5.0f); // A
Motor_RS.Set_Control_Angle(1.5f);         // rad
Motor_RS.CAN_Send_Start_Control();
```

### PP 位置模式

```cpp
Motor_RS.Set_Control_Method(Motor_RS_Control_Method_POSITION_PP);
Motor_RS.Set_Control_Speed_Limit(5.0f);   // vel_max，rad/s
Motor_RS.Set_Control_Acceleration(10.0f); // acc_set，rad/s^2
Motor_RS.Set_Control_Deceleration(10.0f); // 0x702E，可选
Motor_RS.Set_Control_Current_Limit(5.0f); // A
Motor_RS.Set_Control_Angle(3.0f);         // rad
Motor_RS.CAN_Send_Start_Control();
```

PP 目标只发送一次。再次调用 `Set_Control_Angle()` 会重新挂起一个目标，下一次 `TIM_Send_PeriodElapsedCallback()` 发送该目标，避免每个周期重复启动轨迹规划。

## 8. 模式切换

说明书要求电机运行中不可直接切换控制方式。推荐流程：

```cpp
Motor_RS.Set_Control_Enable(false);
Motor_RS.CAN_Send_Stop();

Motor_RS.Set_Control_Method(Motor_RS_Control_Method_SPEED);
Motor_RS.Set_Control_Current_Limit(5.0f);
Motor_RS.Set_Control_Acceleration(20.0f);
Motor_RS.Set_Control_Omega(2.0f);

Motor_RS.CAN_Send_Apply_Control_Configuration();
Motor_RS.CAN_Send_Enable();
Motor_RS.Set_Control_Enable(true);
```

`CAN_Send_Start_Control()` 是上述停止、配置、使能流程的便捷封装。它会连续压入多帧；若发送 FIFO 很小或总线负载高，可以按上面的步骤在不同周期分时调用。

## 9. 反馈数据

```cpp
float cyclic_angle = Motor_RS.Get_Now_Angle();       // rad，约 -4pi~4pi 周期循环
float total_angle  = Motor_RS.Get_Total_Angle();     // rad，驱动做跨边界展开后的累计角度
float omega        = Motor_RS.Get_Now_Omega();       // rad/s
float torque       = Motor_RS.Get_Now_Torque();      // N.m
float temperature  = Motor_RS.Get_Now_Temperature(); // K
float temperature_c = Motor_RS.Get_Now_Temperature_Celsius();
```

简要故障示例：

```cpp
if (Motor_RS.Get_Feedback_Fault_Flag(Motor_RS_Feedback_Fault_OVERTEMPERATURE))
{
    // 过温处理
}
```

## 10. 参数读写

```cpp
Motor_RS.CAN_Send_Read_Parameter(Motor_RS_Parameter_POSITION_KP);

const Struct_Motor_RS_Parameter_Data &Param = Motor_RS.Get_Parameter_Data();
if (Param.Valid && Param.Success &&
    Param.Index == Motor_RS_Parameter_POSITION_KP)
{
    float position_kp = Param.Value_Float;
}
```

写参数：

```cpp
Motor_RS.CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_POSITION_KP, 40.0f);
Motor_RS.CAN_Send_Write_Parameter_U8(Motor_RS_Parameter_ZERO_RANGE,
                                     Motor_RS_Zero_Range_NEGATIVE_PI_TO_PI);
```

常用快捷接口还包括：

```cpp
Motor_RS.CAN_Send_Set_Active_Report_Period_MS(10);
Motor_RS.CAN_Send_Set_CAN_Timeout_Seconds(1.0f);
Motor_RS.CAN_Send_Set_Zero_Offset(0.0f);
Motor_RS.CAN_Send_Set_Cogging_Compensation(true);
Motor_RS.CAN_Send_Set_Initial_Calibration(true);
```

`0x7019`、`0x701A`、`0x701B`、`0x701C` 的成功读取结果会同步写入 `Get_Rx_Data()` 中的机械位置、Iq、机械速度和母线电压字段。

需要掉电保存的参数，在写入后调用：

```cpp
Motor_RS.CAN_Send_Save_Parameters();
```

## 11. 主动上报和故障读取

```cpp
Motor_RS.CAN_Send_Set_Active_Report_Period_MS(10);
Motor_RS.CAN_Send_Set_Active_Report(true);

Motor_RS.CAN_Send_Read_Fault_State();

if (Motor_RS.Get_Fault_Flag(Motor_RS_Fault_STALL_OVERLOAD))
{
    // 堵转过载处理
}
```

驱动兼容说明书中两种主动上报描述：通信类型 `0x18` 的反馈以及开启后周期发送的通信类型 `0x02` 反馈。

## 12. 设置 CAN ID

```cpp
Motor_RS.CAN_Send_Set_CAN_ID(3);
```

电机 ID 立即生效，电机随后以通信类型 0 返回 UID。驱动收到来自新 ID 的应答后才更新本地 `Motor_ID`：

```cpp
if (!Motor_RS.Get_CAN_ID_Change_Pending())
{
    uint8_t new_id = Motor_RS.Get_Motor_ID();
}
```

如电机已经改过 ID，但对象尚未同步，可使用 `Set_Motor_ID_Local()` 只修改软件侧 ID。

## 13. 波特率和协议切换

```cpp
Motor_RS.CAN_Send_Set_Baudrate(Motor_RS_CAN_Baudrate_500K);
Motor_RS.CAN_Send_Set_Protocol(Motor_RS_Protocol_CANOPEN);
```

波特率或协议改变后，主控端必须同步修改 FDCAN 配置。协议切换重新上电后生效。

## 14. 测试状态

源码已使用 C++11、`-Wall -Wextra -Werror` 编译，并通过本地协议单元测试，覆盖扩展 ID、报文大小端、运控打包、参数读写、反馈解析、角度跨边界展开、版本号、UID、CAN ID 修改、故障位、主动上报和 PP 单次发送。

测试基于 HAL/FDCAN 桩函数，尚未在真实 EL05 电机和实际 CAN 总线上完成硬件联调。首次上电应限制电流、速度和运动范围，并准备急停。
