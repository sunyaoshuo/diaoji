# DM-IMU-L1 CAN 驱动

本目录提供达妙 `DM-IMU-L1` 的经典 CAN 驱动，只依赖工程已有的
`1_Middleware/Driver/CAN`，不包含 USB、UART 或 RS485 实现。

协议依据：

- 《达妙科技 DM-IMU-L1 六轴 IMU 模块使用说明书 V1.2》
- 达妙官方 `MC02_CAN收发例程`

## 默认通信参数

- 经典 CAN，11 位标准数据帧，DLC = 8
- 默认波特率：1 Mbps
- 默认 `CAN_ID = 0x01`：主控向 IMU 发送请求/配置
- 默认 `Master_ID = 0x11`：IMU 向主控发送数据/应答

驱动采用手册“CAN 通信”章节及当前官方 MC02 例程中的 8 字节命令格式：

```text
请求：ID=CAN_ID
DATA = CC RID READ/WRITE DD DATA_U32_LE

应答：ID=Master_ID
DATA = CC RID DD ACK DATA_U32_LE
```

传感器数据帧同样由 `Master_ID` 返回，`DATA[0]` 分别为：

- `0x01`：加速度与温度
- `0x02`：角速度
- `0x03`：欧拉角
- `0x04`：四元数

## 接入示例

```cpp
#include "2_Device/IMU/IMU_DM/dvc_imu_dm.h"

Class_IMU_DM IMU_DM;

void CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    // 同一条总线上还可以继续分发给电机等其他设备。
    IMU_DM.CAN_RxCpltCallback(Header, Buffer);
}

void Device_Init()
{
    IMU_DM.Init(&hfdcan1, 0x01, 0x11);
    CAN_Init(&hfdcan1, CAN1_RxCpltCallback);
}
```

请求模式下，由任务按需要轮询：

```cpp
IMU_DM.CAN_Send_Request_Acceleration();
IMU_DM.CAN_Send_Request_Gyroscope();
IMU_DM.CAN_Send_Request_Euler_Angle();
IMU_DM.CAN_Send_Request_Quaternion();
```

主动模式下，IMU 会自行发送已启用的数据，不需要发送上述轮询命令：

```cpp
IMU_DM.CAN_Send_Set_Output_Mode(IMU_DM_Output_Mode_ACTIVE);
IMU_DM.CAN_Send_Save_Parameters();
```

每 100 ms 调用一次在线检测：

```cpp
IMU_DM.TIM_100ms_Alive_PeriodElapsedCallback();
```

读取数据：

```cpp
const Struct_IMU_DM_Acceleration_Data &accel = IMU_DM.Get_Acceleration_Data();
const Struct_IMU_DM_Gyroscope_Data &gyro = IMU_DM.Get_Gyroscope_Data();
const Struct_IMU_DM_Euler_Angle_Data &euler = IMU_DM.Get_Euler_Angle_Data();
const Struct_IMU_DM_Quaternion_Data &quat = IMU_DM.Get_Quaternion_Data();
```

单位如下：

- 加速度：`m/s^2`
- 角速度：`rad/s`
- 欧拉角：`degree`
- 温度：`degree Celsius`
- 四元数：无量纲，顺序 `W, X, Y, Z`

每组数据都有 `Valid` 和 `Sequence`。`Sequence` 变化表示该类数据收到了一帧新值。

## 配置注意事项

- 修改参数后，需要调用 `CAN_Send_Save_Parameters()` 才能防止掉电丢失。
- 修改 CAN 波特率后，主控 FDCAN 必须切换到相同波特率才能继续通信。
- 修改 CAN ID 或 Master ID 时，驱动会暂存新 ID，并在成功应答后更新本地 ID。
  如果设备已由上位机配置过，也可用 `Set_Local_CAN_ID()` 和
  `Set_Local_Master_ID()` 直接同步驱动侧配置。
- `IMU_DM_Register_OUTPUT_SELECTION (0x0f)` 在 V1.2 手册标注为当前固件不支持，
  驱动仅保留通用寄存器接口，不提供专用封装。
- 校准、重启、恢复出厂设置会改变设备状态，按手册要求操作，并在修改参数后重新确认
  CAN ID、Master ID 和波特率。
