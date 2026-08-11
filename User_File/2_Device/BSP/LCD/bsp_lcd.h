/**
 * @file bsp_lcd.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD (ST7789V2, 240x280, RGB565, 4 线 SPI) 驱动
 * @version 0.1
 * @date 2026-07-24 0.1 根据《LCD+5维摇杆模块》手册及参考例程新建
 *
 * @note
 * 1. 驱动芯片 ST7789V2, 物理分辨率 240x280, RGB565.
 * 2. 通信为 4 线 SPI, 阻塞发送, 需要外接 DC (数据/命令) 引脚.
 * 3. 初始化序列与 Y 方向 +20 偏移取自该模组参考例程, 已在该屏验证可用.
 * 4. 控制命令固定低速; 全屏连续像素流在 CS 高电平且 SPI 空闲时仅切换 MBR 分频.
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef BSP_LCD_H
#define BSP_LCD_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_lcd_register.h"
#include "bsp_lcd_font.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// SPI 阻塞收发超时, 单位 ms
#define LCD_SPI_TIMEOUT ((uint32_t)100)

// SPI1 内核时钟 80MHz。控制命令保持 0.625MHz，局部及全屏像素均使用 40MHz。
#define LCD_SPI_CONTROL_BAUDRATEPRESCALER (SPI_BAUDRATEPRESCALER_128)
#define LCD_SPI_LOCAL_PIXEL_BAUDRATEPRESCALER (SPI_BAUDRATEPRESCALER_2)
#define LCD_SPI_PIXEL_BAUDRATEPRESCALER   (SPI_BAUDRATEPRESCALER_2)
#define LCD_SPI_BAUDRATEPRESCALER         LCD_SPI_CONTROL_BAUDRATEPRESCALER

// 区域填充用临时缓冲, 单位像素 (整行最多 280 像素, 取 320 留余量)
#define LCD_FILL_BUFFER_PIXELS ((uint16_t)320)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 屏幕方向
 */
enum Enum_LCD_Orientation : uint8_t
{
    LCD_Orientation_Portrait = 0,       // 240 x 280
    LCD_Orientation_Landscape,          // 280 x 240
    LCD_Orientation_Portrait_Flip,      // 240 x 280 翻转
    LCD_Orientation_Landscape_Flip,     // 280 x 240 翻转
};

/**
 * @brief 特化, LCD (ST7789V2) 驱动
 */
class Class_LCD_ST7789
{
    // SPI1 DMA 发送完成钩子需要访问受保护的 DMA_Tx_Callback
    friend void BSP_LCD_SPI1_DMA_TxCpltHook(void);

public:
    /**
     * @brief 初始化并绑定 SPI 与控制引脚
     *
     * @param __SPI_Handler SPI 句柄
     * @param __CS_GPIOx 片选 GPIO 端口, 传 nullptr 表示片选由硬件下拉
     * @param __CS_GPIO_Pin 片选 GPIO 引脚号
     * @param __DC_GPIOx 数据/命令 GPIO 端口
     * @param __DC_GPIO_Pin 数据/命令 GPIO 引脚号
     * @param __RST_GPIOx 复位 GPIO 端口, 传 nullptr 表示无复位引脚 (走软件复位)
     * @param __RST_GPIO_Pin 复位 GPIO 引脚号
     * @param __BL_GPIOx 背光 GPIO 端口, 传 nullptr 表示无背光引脚
     * @param __BL_GPIO_Pin 背光 GPIO 引脚号
     * @param __Orientation 屏幕方向
     */
    void Init(SPI_HandleTypeDef *__SPI_Handler,
              GPIO_TypeDef *__CS_GPIOx, uint16_t __CS_GPIO_Pin,
              GPIO_TypeDef *__DC_GPIOx, uint16_t __DC_GPIO_Pin,
              GPIO_TypeDef *__RST_GPIOx = nullptr, uint16_t __RST_GPIO_Pin = 0,
              GPIO_TypeDef *__BL_GPIOx = nullptr, uint16_t __BL_GPIO_Pin = 0,
              const Enum_LCD_Orientation &__Orientation = LCD_Orientation_Portrait);

    inline uint16_t Get_Width() const;
    inline uint16_t Get_Height() const;
    inline Enum_LCD_Orientation Get_Orientation() const;
    inline bool Get_Backlight() const;

    // 屏幕控制 ---------------------------------------------------------------
    void Set_Orientation(const Enum_LCD_Orientation &__Orientation);
    void Display_On();
    void Display_Off();
    void Sleep_Out();
    void Sleep_In();
    void Set_Inversion(const bool &__Enable);
    void Set_Backlight(const bool &__Enable);

    // 兼容保留的忙状态接口；当前全屏填充使用阻塞发送。
    inline bool Is_Busy() const;
    void Wait_Until_Idle();

    // 基础绘图 ---------------------------------------------------------------
    void Fill(const uint16_t &__Color);
    void Fill_Rect(const uint16_t &__X, const uint16_t &__Y,
                   const uint16_t &__Width, const uint16_t &__Height,
                   const uint16_t &__Color);
    void Draw_Point(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Color);
    void Draw_Line(uint16_t __X0, uint16_t __Y0, uint16_t __X1, uint16_t __Y1,
                   const uint16_t &__Color);
    void Draw_Rectangle(const uint16_t &__X, const uint16_t &__Y,
                        const uint16_t &__Width, const uint16_t &__Height,
                        const uint16_t &__Color, const bool &__Filled = false);
    void Draw_Circle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Radius,
                     const uint16_t &__Color, const bool &__Filled = false);

    // 文本 -------------------------------------------------------------------
    // __Scale 为整数缩放倍数 (1=原尺寸 8x16, 2=16x32, 4=32x64 ...), 每个字模像素放大成 Scale x Scale 块
    void Draw_Char(const uint16_t &__X, const uint16_t &__Y, const char &__Char,
                   const uint16_t &__Color, const uint16_t &__Background,
                   const Struct_LCD_Font &__Font = LCD_Font_Default, const uint8_t &__Scale = 1);
    void Draw_String(const uint16_t &__X, const uint16_t &__Y, const char *__String,
                     const uint16_t &__Color, const uint16_t &__Background,
                     const Struct_LCD_Font &__Font = LCD_Font_Default, const uint8_t &__Scale = 1);
    void Draw_Number(const uint16_t &__X, const uint16_t &__Y, const int32_t &__Value,
                     const uint16_t &__Color, const uint16_t &__Background,
                     const Struct_LCD_Font &__Font = LCD_Font_Default, const uint8_t &__Scale = 1);

    // 原始图像 ---------------------------------------------------------------
    void Draw_Image(const uint16_t &__X, const uint16_t &__Y,
                    const uint16_t &__Width, const uint16_t &__Height,
                    const uint16_t *__Image);

protected:
    // 初始化相关常量

    // 绑定的 SPI
    SPI_HandleTypeDef *SPI_Handler = nullptr;

    // 控制引脚, CS/RST/BL 可为空
    GPIO_TypeDef *CS_GPIOx = nullptr;
    uint16_t CS_GPIO_Pin = 0;
    GPIO_TypeDef *DC_GPIOx = nullptr;
    uint16_t DC_GPIO_Pin = 0;
    GPIO_TypeDef *RST_GPIOx = nullptr;
    uint16_t RST_GPIO_Pin = 0;
    GPIO_TypeDef *BL_GPIOx = nullptr;
    uint16_t BL_GPIO_Pin = 0;

    // 常量

    // 内部变量

    // 当前方向与几何
    Enum_LCD_Orientation Orientation = LCD_Orientation_Portrait;
    uint8_t Madctl = 0x00;
    uint16_t Width = LCD_WIDTH;
    uint16_t Height = LCD_HEIGHT;
    uint16_t X_Offset = LCD_X_OFFSET;
    uint16_t Y_Offset = LCD_Y_OFFSET;

    // 初始化完成标志
    bool Init_Finished = false;
    // 背光状态
    bool Backlight_Enabled = true;

    // 区域填充用临时缓冲
    uint8_t Fill_Buffer[LCD_FILL_BUFFER_PIXELS * 2U];

    // DMA 异步填充状态 (ISR 与主循环共享, 必须 volatile)
    volatile bool DMA_Busy = false;
    volatile uint32_t Async_Remaining = 0U;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    void Reset_Hardware();
    void Send_Init_Sequence();
    void Apply_Orientation(const Enum_LCD_Orientation &__Orientation);

    void Select();
    void Deselect();

    void Write_Command(const uint8_t &__Command);
    void Write_Data8(const uint8_t &__Data);
    void Write_Data16(const uint16_t &__Data);
    void Write_Pixels(const uint16_t *__Pixels, uint32_t __Count);
    void Set_Window(uint16_t __X0, uint16_t __Y0, uint16_t __X1, uint16_t __Y1);
    HAL_StatusTypeDef Transmit_DMA_Wait(const uint8_t *__Data, uint16_t __Length);

    void Start_Fill_Chunk();
    void DMA_Tx_Callback();
    HAL_StatusTypeDef Set_SPI_Baudrate(uint32_t __Prescaler);
};

/* Exported variables --------------------------------------------------------*/

extern Class_LCD_ST7789 BSP_LCD;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取屏幕宽度
 */
inline uint16_t Class_LCD_ST7789::Get_Width() const
{
    return (Width);
}

/**
 * @brief 获取屏幕高度
 */
inline uint16_t Class_LCD_ST7789::Get_Height() const
{
    return (Height);
}

/**
 * @brief 获取屏幕方向
 */
inline Enum_LCD_Orientation Class_LCD_ST7789::Get_Orientation() const
{
    return (Orientation);
}

/**
 * @brief 获取背光状态
 */
inline bool Class_LCD_ST7789::Get_Backlight() const
{
    return (Backlight_Enabled);
}

inline bool Class_LCD_ST7789::Is_Busy() const
{
    return (DMA_Busy);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
