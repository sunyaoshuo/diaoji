# LCD + 5 维摇杆模组驱动 (ST7789V2, 240×280)

本目录为《LCD+5维摇杆模块》在 dm02_test 工程上的 BSP 驱动, 风格仿照工程内其它设备驱动
(`Class_MVR2TB` 等). LCD 与 5 维摇杆属于同一物理模组, 故放在同一目录.

| 文件 | 说明 |
|------|------|
| `bsp_lcd_register.h` | ST7789V2 命令宏、分辨率/偏移常量、RGB565 颜色宏、初始化指令表 |
| `bsp_lcd_font.h` | 内置 8×16 ASCII 点阵字体 (Courier 离线渲染, 置于 Flash) |
| `bsp_lcd.h` / `bsp_lcd.cpp` | `Class_LCD_ST7789` 驱动, 全局实例 `BSP_LCD` |
| `bsp_joystick_5way.h` / `bsp_joystick_5way.cpp` | `Class_Joystick_5Way` 5 维摇杆驱动, 全局实例 `BSP_Joystick` |

---

## 一、CubeMX 配置核对

依据 PDF 引脚定义表与 `dm02_test.ioc` / 生成的 `spi.c`、`gpio.c`、`adc.c` 比对:

| 项 | 状态 | 说明 |
|----|------|------|
| SPI1 引脚速度 (PB3/PB4/PD7) | ✅ 已在 CubeMX 设为 Very High | `.ioc` 持久 |
| PD10 (DC 数据/命令) | ✅ 已在 CubeMX 配为 GPIO_Output | `.ioc` 持久 |
| PE15 (CS) | ✅ GPIO 输出 | 一直正常 |
| ADC1 通道 18 (摇杆) | ✅ 已加入规则序列 (Rank2, 单端, 387.5 周期, DMA 循环) | `adc.c` |
| **SPI1 模式 / 速度** | ✅ 驱动统一接管 | mode 0、8 位；控制 0.625MHz、局部及全屏像素 40MHz |

> **关于 SPI 配置 (重要)**: 驱动 `Init()` 统一接管 LCD 所需的关键参数:
> - SPI mode 0、8 位数据帧;
> - 初始化和控制命令使用 0.625MHz;
> - 字符、局部矩形和图片的像素负载使用 40MHz;
> - `Fill()` 的连续全屏像素流使用 40MHz;
> - 切速时保证 CS 为高且 SPI 空闲，仅修改 `CFG1.MBR`，不调用 `HAL_SPI_Init()`.
>
> 本板已依次实测 1.25/2.5/5/10/20/40MHz 全屏像素流正常；局部像素当前测试 40MHz。禁止在 LCD 事务间用
> `HAL_SPI_Init()` 动态切速，因为它会重建整个 STM32H7 SPI 外设状态并导致黑屏。

> **关于摇杆 ADC**: KEY 实际为 PA5 单端输出。STM32H723 的 PA5 对应 **ADC1_INP19**，
> 因此工程使用 ADC1 通道 19 单端采样，不能把 PA5 当作通道 18 的差分负端读取。
> 以硬件实测为准. 本驱动与 ADC 解耦, 把对应 ADC 原值喂给 `BSP_Joystick.Update()` 即可.

> SPI1 虽配置了 TX DMA，但当前 LCD 绘图统一使用阻塞发送。

---

## 二、引脚连接 (本工程)

| 模组信号 | MCU 引脚 | 说明 |
|----------|----------|------|
| SPI1_SCK | PB3 | SPI 时钟 |
| SPI1_MISO | PB4 | SPI 主入 (屏幕基本只写) |
| SPI1_MOSI | PD7 | SPI 主出 |
| SPI1_CS | PE15 | 片选, 低有效 |
| GPIO (DC) | PD10 | 数据/命令, 0=命令 1=数据 |
| BL (背光) | PB10 | 本板把模组 "IIC2_SCL" 线复用为背光使能 (高有效). 注: CubeMX 里 PB10 原是 I2C2_SCL, 驱动 `Init()` 会重新配为推挽输出; I2C2 未使用, 建议在 CubeMX 里关掉 I2C2 更干净 |
| KEY (摇杆) | PA5 | ADC1 通道 19 单端 (见第一节) |

模组 RST 本工程未单独引出, `Init()` 的 RST 参数留空即可.

---

## 三、LCD 用法

### 3.1 初始化 (在 `Task_Init()` 中, `SYS_Timestamp.Init` 之后)

```cpp
#include "2_Device/BSP/LCD/bsp_lcd.h"

// Task_Init() 内:
BSP_LCD.Init(&hspi1,
             GPIOE, GPIO_PIN_15,   // CS = PE15
             GPIOD, GPIO_PIN_10);  // DC = PD10
BSP_LCD.Fill(LCD_COLOR_BLACK);
BSP_LCD.Draw_String(4, 4, "LCD ST7789V2 OK", LCD_COLOR_WHITE, LCD_COLOR_BLACK);
BSP_LCD.Draw_Number(4, 24, 12345, LCD_COLOR_GREEN, LCD_COLOR_BLACK);
```

### 3.2 绘图 API 摘要

```cpp
BSP_LCD.Fill(color);                                        // 全屏填充
BSP_LCD.Fill_Rect(x, y, w, h, color);                       // 实心矩形
BSP_LCD.Draw_Point(x, y, color);                            // 画点
BSP_LCD.Draw_Line(x0, y0, x1, y1, color);                   // 画线
BSP_LCD.Draw_Rectangle(x, y, w, h, color, /*filled=*/false);// 矩形框
BSP_LCD.Draw_Circle(x, y, r, color, /*filled=*/false);      // 圆
BSP_LCD.Draw_String(x, y, "text", fg, bg);                  // 字符串 (8x16)
BSP_LCD.Draw_Number(x, y, 42, fg, bg);                      // 整数
BSP_LCD.Draw_Image(x, y, w, h, rgb565_buffer);              // 原始图像 (整块)
BSP_LCD.Set_Orientation(LCD_Orientation_Landscape);         // 横竖屏
BSP_LCD.Display_Off() / Display_On();                       // 显示开关
BSP_LCD.Set_Backlight(false);                               // 背光 (需接 BL 引脚)
uint16_t c = LCD_COLOR_RGB565(255, 128, 0);                 // RGB 合成
```

颜色用 `LCD_COLOR_WHITE/BLACK/RED/GREEN/BLUE/...` 或 `LCD_COLOR_RGB565(r,g,b)`.

---

## 四、5 维摇杆用法

摇杆 5 个方向共用一个 ADC 通道 (电阻分压), 驱动与 ADC 采集**解耦**: 把最新 ADC 原始值喂给
`BSP_Joystick.Update(adc)`, 驱动负责去抖与按下/松开事件.

### 4.1 ADC 已配好, 只需启动 DMA

CubeMX 已把 ADC1 配为 2 通道 (Rank1=通道4 电池, Rank2=通道19 单端摇杆), DMA 循环.
在 `Task_Init()` 里调用一次 `ADC_Init` 启动采样:

```cpp
#include "1_Middleware/Driver/ADC/drv_adc.h"

ADC_Init(&hadc1, 2);   // 2 = 通道数 (电池 + 摇杆)
```

之后 `ADC1_Manage_Object.ADC_Data[0]` 是电池电压(通道4),
`ADC1_Manage_Object.ADC_Data[1]` 是摇杆(PA5/通道19单端).

### 4.2 实测校准理想 ADC 值

```cpp
BSP_Joystick.Init();   // 使用默认实测理想值

// 在 Task1ms 里喂值并把原始值显示到屏幕, 依次按 上/下/左/右/中/悬空 记录 ADC 原值
BSP_Joystick.Update(ADC1_Manage_Object.ADC_Data[1]);
BSP_LCD.Draw_Number(4, 40, BSP_Joystick.Get_Raw_Adc(), LCD_COLOR_YELLOW, LCD_COLOR_BLACK);

// 用悬空/中/上/下/左/右的实测稳定中心值构造配置:
Struct_Joystick_Ideal_Adc ideal =
{
    /*None  */ none_adc,
    /*Center*/ center_adc,
    /*Up    */ up_adc,
    /*Down  */ down_adc,
    /*Left  */ left_adc,
    /*Right */ right_adc,
};
BSP_Joystick.Set_Ideal_Adc(ideal);
```

### 4.3 读取方向与事件

```cpp
// Task1ms_Callback() 内:
BSP_Joystick.Update(ADC1_Manage_Object.ADC_Data[1]);

Enum_Joystick_Direction dir = BSP_Joystick.Get_Direction(); // None/Center/Up/Down/Left/Right
if (BSP_Joystick.Get_Just_Pressed())   { /* 本周期刚按下 */ }
if (BSP_Joystick.Get_Just_Released())  { /* 本周期刚松开 */ }
```

边沿事件按 "本周期" 语义, 每次 `Update()` 自动清零 (也可手动 `Clear_Edge()`).

---

## 五、备注

- 字体为 8×16 ASCII (0x20~0x7E), 放在 Flash (`static const`). 如需更大字体可自行扩展
  `Struct_LCD_Font` 指向新字模.
- 像素在 SPI 上按 RGB565 **高字节在前**发送, 与参考例程一致.
- 240×280 屏在 240×320 GRAM 中 **Y 方向偏移 +20** (X 无偏移, 竖屏 MADCTL=0x00), 已在
  `Set_Window()` 内部处理.
- **CubeMX 重新生成代码后, 若有文件增删/改名 (如本次 `usbd_cdc_if.c`→`.cpp`), 需要让 CMake
  重新配置 (CLion 里 Reload CMake Project, 或命令行 `cmake -S . -B <build_dir>`) 才能更新
  源文件列表, 否则会报 "missing and no known rule to make".**
