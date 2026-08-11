# 五维摇杆 ADC 单端输入迁移说明

## 1. 修改目的

五维摇杆的 `KEY` 信号实际是一个相对 GND 的单端模拟电压，连接到 STM32H723 的 `PA5`。

`PA5` 在单端 ADC 模式下对应：

- 引脚：`PA5`
- ADC 实例：`ADC1` 或 `ADC2`
- 单端输入：`ADC12_INP19`
- HAL 通道：`ADC_CHANNEL_19`
- 输入模式：`ADC_SINGLE_ENDED`

不要把 `PA5` 配置成 `ADC1_INN18` 或通道 18 的差分负端。通道 18 差分模式测量的是 `PA4(INP18) - PA5(INN18)`，与摇杆单端输出的硬件形式不符，会导致数值漂移、方向值改变或无法正确判定。

## 2. CubeMX 配置

在 ADC1 的 Regular Conversion 中配置两个 Rank：

| Rank | 用途 | ADC 通道 | 输入模式 |
|---|---|---|---|
| Rank 1 | 电池电压 | 按原工程配置，例如 Channel 4 | Single-ended |
| Rank 2 | 五维摇杆 | Channel 19 / PA5 | Single-ended |

摇杆通道建议配置：

```text
Channel: ADC_CHANNEL_19
Rank: ADC_REGULAR_RANK_2
Sampling Time: ADC_SAMPLETIME_387CYCLES_5
Single/Differential: ADC_SINGLE_ENDED
```

ADC DMA 保持循环模式：

```text
Conversion Data Management: DMA Circular Mode
DMA Direction: Peripheral to Memory
Peripheral Data Width: Half Word
Memory Data Width: Half Word
Memory Increment: Enabled
DMA Mode: Circular
```

在 `.ioc` 文件中对应的关键内容应类似：

```ini
ADC1.Channel-3\#ChannelRegularConversion=ADC_CHANNEL_19
ADC1.Rank-3\#ChannelRegularConversion=2
ADC1.SingleDiff-3\#ChannelRegularConversion=ADC_SINGLE_ENDED
PA5.Signal=ADCx_INP19
SH.ADCx_INP19.0=ADC1_INP19,IN19-Single-Ended
SH.ADCx_INP19.ConfNb=1
```

应删除原来的差分配置，例如：

```ini
ADC1.Channel-3\#ChannelRegularConversion=ADC_CHANNEL_18
ADC1.SingleDiff-3\#ChannelRegularConversion=ADC_DIFFERENTIAL_ENDED
PA5.Signal=ADCx_INN18
SH.ADCx_INN18.0=ADC1_INN18,IN18-Differential
SH.ADCx_INP18.0=ADC1_INP18,IN18-Differential
```

如果 `PA4` 不再用于其他功能，可将其恢复为未使用状态或普通模拟模式，但不能继续让 ADC Rank2 使用 PA4/PA5 的通道 18 差分组合。

## 3. HAL ADC 初始化修改

摇杆所在 Rank 的通道配置应改为：

```c
sConfig.Channel = ADC_CHANNEL_19;
sConfig.Rank = ADC_REGULAR_RANK_2;
sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
sConfig.SingleDiff = ADC_SINGLE_ENDED;

if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
    Error_Handler();
}
```

ADC GPIO 初始化只需要将 `PA5` 配为模拟输入：

```c
__HAL_RCC_GPIOA_CLK_ENABLE();

GPIO_InitStruct.Pin = GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

对应的反初始化为：

```c
HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
```

## 4. ADC 校准修改

本工程所有 ADC 规则通道均为单端输入，因此初始化时只进行单端偏移校准：

```cpp
HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
```

删除不再需要的差分校准：

```cpp
HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET,
                            ADC_DIFFERENTIAL_ENDED);
```

注意：HAL 的 ADC 校准只校准 MCU ADC 外设自身的偏移误差，不会自动学习五维摇杆各方向的理想 ADC 值。

## 5. DMA 数据顺序

如果规则转换序列为：

```text
Rank1 = 电池电压
Rank2 = 五维摇杆
```

并使用两个半字启动 DMA：

```cpp
ADC_Init(&hadc1, 2);
```

那么数据对应关系为：

```cpp
ADC1_Manage_Object.ADC_Data[0]  // Rank1：电池电压
ADC1_Manage_Object.ADC_Data[1]  // Rank2：五维摇杆 PA5/Channel 19
```

摇杆更新代码保持为：

```cpp
BSP_Joystick.Update(ADC1_Manage_Object.ADC_Data[1]);
```

如果另一个工程的 Rank 顺序不同，必须按实际 Rank 顺序调整数组索引。

## 6. 方向判定策略

当前驱动不再使用固定区间，而是比较当前 ADC 数值与六个理想值的绝对距离，选择距离最小的状态：

```cpp
struct Struct_Joystick_Ideal_Adc
{
    uint16_t None;
    uint16_t Center;
    uint16_t Up;
    uint16_t Down;
    uint16_t Left;
    uint16_t Right;
};
```

核心判定逻辑等价于：

```cpp
const uint16_t ideal_values[] =
{
    Ideal_Adc.None,
    Ideal_Adc.Center,
    Ideal_Adc.Up,
    Ideal_Adc.Down,
    Ideal_Adc.Left,
    Ideal_Adc.Right,
};

uint8_t nearest = 0;
uint16_t min_distance = UINT16_MAX;

for (uint8_t i = 0; i < 6; ++i)
{
    const uint16_t distance = adc_value > ideal_values[i]
                                  ? adc_value - ideal_values[i]
                                  : ideal_values[i] - adc_value;
    if (distance < min_distance)
    {
        min_distance = distance;
        nearest = i;
    }
}
```

枚举顺序必须与数组顺序一致：

```cpp
None = 0,
Center = 1,
Up = 2,
Down = 3,
Left = 4,
Right = 5
```

原有的 20 ms 去抖和按下/释放边沿事件逻辑可以保留。

## 7. 必须重新测量理想值

从通道 18 差分输入切换到 PA5/通道 19 单端输入后，旧 ADC 数值不能直接沿用。需要重新烧录后依次记录以下六种状态的稳定原始值：

1. 摇杆悬空，无按键动作
2. 中键
3. 上
4. 下
5. 左
6. 右

建议每个状态连续采集至少 100～500 个样本，去掉刚按下时的过渡样本，然后使用稳定样本的中位数或截尾平均数作为理想值。不要只记录单个瞬时样本。

将测量结果写入：

```cpp
static const Struct_Joystick_Ideal_Adc Joystick_Ideal_Adc_Default =
{
    none_adc,
    center_adc,
    up_adc,
    down_adc,
    left_adc,
    right_adc,
};
```

旧工程中曾使用的 `300/2310/1200/800/1840/1500` 来自之前的输入配置，切换为单端输入后只能作为历史数据，不能视为有效默认值。

## 8. 验收步骤

完成迁移后按以下顺序验证：

1. 编译通过，确认没有残留 `ADC_CHANNEL_18` 或 `ADC_DIFFERENTIAL_ENDED` 的摇杆配置。
2. 检查 ADC DMA 数组，确认电池值和摇杆值没有交换。
3. 空闲时观察摇杆原始 ADC 值，应稳定在某个范围，不应持续大幅跳变。
4. 依次按五个方向，确认每个方向都有独立且可重复的 ADC 数值中心。
5. 写入重新测得的六个理想值。
6. 验证最近值判断结果、20 ms 去抖、按下事件和释放事件。
7. 多次重启设备，确认各方向仍能稳定识别。

## 9. 本工程实际修改文件

本次修改涉及：

```text
Core/Src/adc.c
User_File/1_Middleware/Driver/ADC/drv_adc.cpp
User_File/2_Device/BSP/LCD/bsp_joystick_5way.h
User_File/4_Task/tsk_config_and_callback.cpp
User_File/2_Device/BSP/LCD/bsp_lcd_README.md
dm02_test.ioc
```

本地编译已经通过。按照要求，本次没有烧录固件。

