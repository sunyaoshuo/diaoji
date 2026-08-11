# 吊机 Web Serial 上位机

打开 `crane_control.html` 后，可通过 USART1 控制吊机的三个 CAN 电机和俯仰直线电机。

## 连接

- USART1：115200 baud、8 data bits、no parity、1 stop bit，无流控。
- STM32 USART1 TX（PA9）接 USB-TTL RX；USART1 RX（PA10）接 USB-TTL TX；两端共地。
- 浏览器使用新版 Chrome 或 Edge。Web Serial 需要安全上下文，建议在本目录运行：

  ```powershell
  python -m http.server 8000
  ```

  然后访问 `http://localhost:8000/crane_control.html`。

## 电机配置

| 机构 | 类型 | 总线 | ID / 模式 |
|---|---|---|---|
| 伸缩 | Motor_RS | CAN2 | Motor ID 1，PP 位置模式，Master ID 0xFD |
| 收放线 | Motor_RS | CAN2 | Motor ID 2，PP 位置模式，Master ID 0xFD |
| YAW | Motor_DM | CAN2 | CAN_ID 0x01，MASTER_ID 0x00，位置-速度模式 |
| 俯仰 | 直线电机驱动器 | GPIO | PE13/PE09 互补方向电平 |

用户没有指定 Motor_RS 的 Master ID，程序采用该驱动的默认值 `0xFD`。若电机参数不是 0xFD，请修改 `RS_MASTER_ID`。

## 安全逻辑

- MCU 和网页上电后均默认关闭输出。
- 网页打开安全开关后，固件分时配置两台 RS 电机，避免一次写满 CAN2 的 8 帧发送 FIFO。
- 解锁时先用当前反馈角度作为目标，避免仅因解锁就突然回零。
- 网页每 250 ms 发送 `PING`；固件超过 1000 ms 未收到有效命令会停止/失能全部电机。
- 俯仰是点动控制，超过 300 ms 未续传方向命令会自动把 PE13、PE09 都拉低。
- “立即停止”、关闭安全开关或正常断开串口都会发送 `PITCH,STOP` 和 `SAFE,0`。

注意：Web Serial 和软件互锁不能替代独立的硬件急停、限位开关和驱动器 STO。PE13/PE09 只能连接驱动器逻辑输入，不能直接连接电机功率端。

## ASCII 协议

网页发送的每条命令以 `\n` 结束。角度单位为 `0.01 degree`，例如 `1234` 表示 `12.34°`。

```text
PING
SAFE,0
SAFE,1
SET,EXT,1234
SET,WINCH,-9000
SET,YAW,4500
PITCH,UP
PITCH,DOWN
PITCH,STOP
```

MCU 每 50 ms 返回：

```text
TEL,<请求安全>,<实际输出>,<伸缩角度>,<伸缩在线>,<收放线角度>,<收放线在线>,<YAW角度>,<YAW在线>,<俯仰方向>,<命令延迟ms>,<接收溢出数>,<发送错误数>
```

俯仰方向为 `1=抬升`、`0=停止`、`-1=下降`。直线电机没有位置传感器，因此页面只能显示运动状态，不能显示真实俯仰角。

## 上机前必须标定

固件文件 `User_File/3_Application/Crane/alg_crane_control.h` 中提供了软件角度限幅，`.cpp` 中提供速度、加速度和电流限幅。当前值仅是保守的通用初值，必须按真实减速比、卷筒直径、臂长、机械限位和负载重新计算；同时在硬件上接入上下限位与独立急停。
