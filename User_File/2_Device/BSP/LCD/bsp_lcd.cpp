/**
 * @file bsp_lcd.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD (ST7789V2, 240x280, RGB565, 4 线 SPI) 驱动
 * @version 0.1
 * @date 2026-07-24 0.1 根据《LCD+5维摇杆模块》手册及参考例程新建
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_lcd.h"
#include <stdio.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_LCD_ST7789 BSP_LCD;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化并绑定 SPI 与控制引脚, 完成屏幕初始化
 */
void Class_LCD_ST7789::Init(SPI_HandleTypeDef *__SPI_Handler,
                            GPIO_TypeDef *__CS_GPIOx, uint16_t __CS_GPIO_Pin,
                            GPIO_TypeDef *__DC_GPIOx, uint16_t __DC_GPIO_Pin,
                            GPIO_TypeDef *__RST_GPIOx, uint16_t __RST_GPIO_Pin,
                            GPIO_TypeDef *__BL_GPIOx, uint16_t __BL_GPIO_Pin,
                            const Enum_LCD_Orientation &__Orientation)
{
    SPI_Handler = __SPI_Handler;
    CS_GPIOx = __CS_GPIOx;
    CS_GPIO_Pin = __CS_GPIO_Pin;
    DC_GPIOx = __DC_GPIOx;
    DC_GPIO_Pin = __DC_GPIO_Pin;
    RST_GPIOx = __RST_GPIOx;
    RST_GPIO_Pin = __RST_GPIO_Pin;
    BL_GPIOx = __BL_GPIOx;
    BL_GPIO_Pin = __BL_GPIO_Pin;

    // 自包含配置控制引脚为推挽输出 (CubeMX 若已配置则等价覆盖, 缺失时补齐)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    if (CS_GPIOx != nullptr)
    {
        GPIO_InitStruct.Pin = CS_GPIO_Pin;
        HAL_GPIO_Init(CS_GPIOx, &GPIO_InitStruct);
        HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);
    }
    if (DC_GPIOx != nullptr)
    {
        GPIO_InitStruct.Pin = DC_GPIO_Pin;
        HAL_GPIO_Init(DC_GPIOx, &GPIO_InitStruct);
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_RESET);
    }
    if (RST_GPIOx != nullptr)
    {
        GPIO_InitStruct.Pin = RST_GPIO_Pin;
        HAL_GPIO_Init(RST_GPIOx, &GPIO_InitStruct);
        HAL_GPIO_WritePin(RST_GPIOx, RST_GPIO_Pin, GPIO_PIN_SET);
    }
    if (BL_GPIOx != nullptr)
    {
        GPIO_InitStruct.Pin = BL_GPIO_Pin;
        HAL_GPIO_Init(BL_GPIOx, &GPIO_InitStruct);
        HAL_GPIO_WritePin(BL_GPIOx, BL_GPIO_Pin, GPIO_PIN_SET);
        Backlight_Enabled = true;
    }

    if (SPI_Handler == nullptr || DC_GPIOx == nullptr)
    {
        return;
    }

    // 强制 8 位数据帧 + 由 LCD_SPI_BAUDRATEPRESCALER 决定的速度 (CubeMX 的 DataSize/分频不可靠, 在此统一接管).
    SPI_Handler->Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Handler->Init.BaudRatePrescaler = LCD_SPI_BAUDRATEPRESCALER;
    SPI_Handler->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    if (HAL_SPI_Init(SPI_Handler) != HAL_OK)
    {
        return;
    }

    Reset_Hardware();
    Send_Init_Sequence();
    Apply_Orientation(__Orientation);

    Fill(LCD_COLOR_BLACK);
    Init_Finished = true;
}

/**
 * @brief 硬件复位 (无复位引脚时为空操作, 依靠初始化序列中的软件复位)
 */
void Class_LCD_ST7789::Reset_Hardware()
{
    if (RST_GPIOx == nullptr)
    {
        return;
    }

    HAL_GPIO_WritePin(RST_GPIOx, RST_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RST_GPIOx, RST_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RST_GPIOx, RST_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/**
 * @brief 发送初始化指令序列
 */
void Class_LCD_ST7789::Send_Init_Sequence()
{
    Select();
    for (uint16_t i = 0; i < LCD_INIT_COMMANDS_COUNT; ++i)
    {
        const Struct_LCD_Init_Cmd *cmd = &LCD_INIT_COMMANDS[i];
        Write_Command(cmd->Command);
        for (uint8_t j = 0; j < cmd->Data_Length; ++j)
        {
            Write_Data8(cmd->Data[j]);
        }
        if (cmd->Delay_Ms != 0)
        {
            HAL_Delay(cmd->Delay_Ms);
        }
    }
    Deselect();
}

/**
 * @brief 应用屏幕方向, 设置 MADCTL 与地址偏移
 */
void Class_LCD_ST7789::Apply_Orientation(const Enum_LCD_Orientation &__Orientation)
{
    Orientation = __Orientation;

    switch (__Orientation)
    {
    case LCD_Orientation_Landscape:
        Madctl = LCD_MADCTL_MX | LCD_MADCTL_MV;
        Width = 280;
        Height = 240;
        X_Offset = 20;
        Y_Offset = 0;
        break;

    case LCD_Orientation_Portrait_Flip:
        Madctl = LCD_MADCTL_MY;
        Width = LCD_WIDTH;
        Height = LCD_HEIGHT;
        X_Offset = LCD_X_OFFSET;
        Y_Offset = LCD_Y_OFFSET;
        break;

    case LCD_Orientation_Landscape_Flip:
        Madctl = LCD_MADCTL_MY | LCD_MADCTL_MV;
        Width = 280;
        Height = 240;
        X_Offset = 20;
        Y_Offset = 0;
        break;

    case LCD_Orientation_Portrait:
    default:
        Madctl = 0x00;
        Width = LCD_WIDTH;
        Height = LCD_HEIGHT;
        X_Offset = LCD_X_OFFSET;
        Y_Offset = LCD_Y_OFFSET;
        break;
    }

    Select();
    Write_Command(LCD_CMD_MADCTL);
    Write_Data8(Madctl);
    Deselect();
}

/**
 * @brief 切换屏幕方向
 */
void Class_LCD_ST7789::Set_Orientation(const Enum_LCD_Orientation &__Orientation)
{
    Apply_Orientation(__Orientation);
}

void Class_LCD_ST7789::Display_On()
{
    Select();
    Write_Command(LCD_CMD_DISPON);
    Deselect();
}

void Class_LCD_ST7789::Display_Off()
{
    Select();
    Write_Command(LCD_CMD_DISPOFF);
    Deselect();
}

void Class_LCD_ST7789::Sleep_Out()
{
    Select();
    Write_Command(LCD_CMD_SLPOUT);
    Deselect();
    HAL_Delay(120);
}

void Class_LCD_ST7789::Sleep_In()
{
    Select();
    Write_Command(LCD_CMD_SLPIN);
    Deselect();
    HAL_Delay(5);
}

void Class_LCD_ST7789::Set_Inversion(const bool &__Enable)
{
    Select();
    Write_Command(__Enable ? LCD_CMD_INVON : LCD_CMD_INVOFF);
    Deselect();
}

void Class_LCD_ST7789::Set_Backlight(const bool &__Enable)
{
    Backlight_Enabled = __Enable;
    if (BL_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(BL_GPIOx, BL_GPIO_Pin, __Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief 拉低片选. 先等待可能的异步刷屏完成, 避免与 DMA 回调里的 Deselect 竞争.
 */
void Class_LCD_ST7789::Select()
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }
    if (CS_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 拉高片选
 */
void Class_LCD_ST7789::Deselect()
{
    if (CS_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 使用 TX DMA 发送并等待完成，保持现有同步绘图 API 的缓冲区生命周期语义。
 */
HAL_StatusTypeDef Class_LCD_ST7789::Transmit_DMA_Wait(const uint8_t *__Data, uint16_t __Length)
{
    if (SPI_Handler == nullptr || SPI_Handler->hdmatx == nullptr ||
        __Data == nullptr || __Length == 0U)
    {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef start_status =
        HAL_SPI_Transmit_DMA(SPI_Handler, const_cast<uint8_t *>(__Data), __Length);
    if (start_status != HAL_OK)
    {
        return start_status;
    }

    const uint32_t start_tick = HAL_GetTick();
    while (HAL_SPI_GetState(SPI_Handler) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start_tick) >= LCD_SPI_TIMEOUT)
        {
            HAL_SPI_DMAStop(SPI_Handler);
            return HAL_TIMEOUT;
        }
    }

    return (HAL_SPI_GetError(SPI_Handler) == HAL_SPI_ERROR_NONE) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 写命令 (DC=0)
 */
void Class_LCD_ST7789::Write_Command(const uint8_t &__Command)
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_RESET);
    }
    uint8_t temp = __Command;
    HAL_SPI_Transmit(SPI_Handler, &temp, 1U, LCD_SPI_TIMEOUT);
}

/**
 * @brief 写一字节数据 (DC=1)
 */
void Class_LCD_ST7789::Write_Data8(const uint8_t &__Data)
{
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }
    uint8_t temp = __Data;
    HAL_SPI_Transmit(SPI_Handler, &temp, 1U, LCD_SPI_TIMEOUT);
}

/**
 * @brief 写一字(双字节)数据, 高字节在前
 */
void Class_LCD_ST7789::Write_Data16(const uint16_t &__Data)
{
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }
    uint8_t temp[2] = {static_cast<uint8_t>(__Data >> 8), static_cast<uint8_t>(__Data & 0xff)};
    HAL_SPI_Transmit(SPI_Handler, temp, 2U, LCD_SPI_TIMEOUT);
}

/**
 * @brief 连续写入若干像素 (RGB565, 高字节在前)
 */
void Class_LCD_ST7789::Write_Pixels(const uint16_t *__Pixels, uint32_t __Count)
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }

    uint32_t remaining = __Count;
    const uint16_t *p = __Pixels;
    while (remaining > 0U)
    {
        uint32_t chunk = (remaining > LCD_FILL_BUFFER_PIXELS) ? LCD_FILL_BUFFER_PIXELS : remaining;
        for (uint32_t i = 0U; i < chunk; ++i)
        {
            uint16_t color = p[i];
            Fill_Buffer[i * 2U] = static_cast<uint8_t>(color >> 8);
            Fill_Buffer[i * 2U + 1U] = static_cast<uint8_t>(color & 0xffU);
        }
        Transmit_DMA_Wait(Fill_Buffer, static_cast<uint16_t>(chunk * 2U));
        p += chunk;
        remaining -= chunk;
    }
}

/**
 * @brief 设置显示窗口, 之后连续写入的像素按行优先填入该窗口
 *        注意: 不管理片选, 调用者负责 Select/Deselect. 末尾发送 RAMWR.
 */
void Class_LCD_ST7789::Set_Window(uint16_t __X0, uint16_t __Y0, uint16_t __X1, uint16_t __Y1)
{
    uint16_t x0 = __X0 + X_Offset;
    uint16_t x1 = __X1 + X_Offset;
    uint16_t y0 = __Y0 + Y_Offset;
    uint16_t y1 = __Y1 + Y_Offset;

    Write_Command(LCD_CMD_CASET);
    Write_Data8(static_cast<uint8_t>(x0 >> 8));
    Write_Data8(static_cast<uint8_t>(x0 & 0xff));
    Write_Data8(static_cast<uint8_t>(x1 >> 8));
    Write_Data8(static_cast<uint8_t>(x1 & 0xff));

    Write_Command(LCD_CMD_RASET);
    Write_Data8(static_cast<uint8_t>(y0 >> 8));
    Write_Data8(static_cast<uint8_t>(y0 & 0xff));
    Write_Data8(static_cast<uint8_t>(y1 >> 8));
    Write_Data8(static_cast<uint8_t>(y1 & 0xff));

    Write_Command(LCD_CMD_RAMWR);
}

/**
 * @brief 全屏阻塞填充。窗口命令使用控制速率；初始化完成后的连续像素流使用高速率。
 */
void Class_LCD_ST7789::Fill(const uint16_t &__Color)
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }

    // 预填充颜色到缓冲 (高字节在前)
    uint8_t hi = static_cast<uint8_t>(__Color >> 8);
    uint8_t lo = static_cast<uint8_t>(__Color & 0xffU);
    for (uint16_t i = 0U; i < LCD_FILL_BUFFER_PIXELS; ++i)
    {
        Fill_Buffer[i * 2U] = hi;
        Fill_Buffer[i * 2U + 1U] = lo;
    }

    Select();
    Set_Window(0, 0, Width - 1U, Height - 1U);
    const bool use_fast_pixels = Init_Finished;
    if (use_fast_pixels)
    {
        Deselect();
        if (Set_SPI_Baudrate(LCD_SPI_PIXEL_BAUDRATEPRESCALER) != HAL_OK)
        {
            return;
        }
        if (DC_GPIOx != nullptr)
        {
            HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
        }
        Select();
        HAL_Delay(1U);
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }

    uint32_t remaining = static_cast<uint32_t>(Width) * Height;
    while (remaining > 0U)
    {
        const uint32_t chunk = (remaining > LCD_FILL_BUFFER_PIXELS) ?
                               LCD_FILL_BUFFER_PIXELS : remaining;
        if (Transmit_DMA_Wait(Fill_Buffer,
                              static_cast<uint16_t>(chunk * 2U)) != HAL_OK)
        {
            break;
        }
        remaining -= chunk;
    }
    if (use_fast_pixels)
    {
        HAL_Delay(1U);
    }
    Deselect();
    if (use_fast_pixels)
    {
        Set_SPI_Baudrate(LCD_SPI_CONTROL_BAUDRATEPRESCALER);
    }
}

/**
 * @brief 阻塞等待异步刷屏完成. 带超时保护: 若 DMA 完成中断因故未触发
 *        (NVIC 关闭/线缆故障/配置错误), 强制恢复以免死锁主循环.
 */
void Class_LCD_ST7789::Wait_Until_Idle()
{
    uint32_t start = HAL_GetTick();
    while (DMA_Busy)
    {
        if (HAL_GetTick() - start > 5000U)
        {
            if (SPI_Handler != nullptr && SPI_Handler->hdmatx != nullptr)
            {
                HAL_SPI_DMAStop(SPI_Handler);
            }
            Async_Remaining = 0U;
            Deselect();
            DMA_Busy = false;
            break;
        }
    }
}

/**
 * @brief 启动下一个 DMA 分块 (整屏填充用)
 */
void Class_LCD_ST7789::Start_Fill_Chunk()
{
    uint32_t chunk = (Async_Remaining > LCD_FILL_BUFFER_PIXELS) ? LCD_FILL_BUFFER_PIXELS : Async_Remaining;
    if (HAL_SPI_Transmit_DMA(SPI_Handler, Fill_Buffer, chunk * 2U) != HAL_OK)
    {
        // DMA 启动失败, 直接结束本段以免卡死
        Async_Remaining = 0U;
        DMA_Busy = false;
        Deselect();
        return;
    }
    Async_Remaining -= chunk;
}

/**
 * @brief SPI TX DMA 完成中断回调 (经 drv_spi 的 HAL_SPI_TxCpltCallback 钩子进入).
 *        还有剩余分块则继续, 否则释放片选并清忙标志.
 */
void Class_LCD_ST7789::DMA_Tx_Callback()
{
    if (!DMA_Busy)
    {
        return;
    }
    if (Async_Remaining > 0U)
    {
        Start_Fill_Chunk();
    }
    else
    {
        Deselect();
        DMA_Busy = false;
    }
}

/**
 * @brief SPI1 TX DMA 完成钩子, 供 drv_spi 的 HAL_SPI_TxCpltCallback 调用 (覆盖其弱定义)
 */
void BSP_LCD_SPI1_DMA_TxCpltHook(void)
{
    BSP_LCD.DMA_Tx_Callback();
}

/**
 * @brief 在 CS 为高且 SPI 空闲时只修改 MBR 分频位，避免 HAL_SPI_Init 重置外设状态。
 */
HAL_StatusTypeDef Class_LCD_ST7789::Set_SPI_Baudrate(uint32_t __Prescaler)
{
    if (SPI_Handler == nullptr)
    {
        return HAL_ERROR;
    }

    Deselect();
    if (SPI_Handler->State != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    SPI_Handler->Init.BaudRatePrescaler = __Prescaler;
    MODIFY_REG(SPI_Handler->Instance->CFG1, SPI_CFG1_MBR, __Prescaler);
    return HAL_OK;
}

/**
 * @brief 矩形区域填充单色
 */
void Class_LCD_ST7789::Fill_Rect(const uint16_t &__X, const uint16_t &__Y,
                                 const uint16_t &__Width, const uint16_t &__Height,
                                 const uint16_t &__Color)
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }
    if (__Width == 0U || __Height == 0U)
    {
        return;
    }
    if (__X >= Width || __Y >= Height)
    {
        return;
    }

    // 用 uint32 计算避免 __X+__Width 超过 65535 时回绕
    uint32_t x1 = static_cast<uint32_t>(__X) + __Width - 1U;
    uint32_t y1 = static_cast<uint32_t>(__Y) + __Height - 1U;
    if (x1 >= Width)
    {
        x1 = Width - 1U;
    }
    if (y1 >= Height)
    {
        y1 = Height - 1U;
    }
    uint32_t total = (x1 - __X + 1U) * (y1 - __Y + 1U);

    // 预先将颜色填入缓冲
    for (uint16_t i = 0U; i < LCD_FILL_BUFFER_PIXELS; ++i)
    {
        Fill_Buffer[i * 2U] = static_cast<uint8_t>(__Color >> 8);
        Fill_Buffer[i * 2U + 1U] = static_cast<uint8_t>(__Color & 0xffU);
    }

    Select();
    Set_Window(__X, __Y, static_cast<uint16_t>(x1), static_cast<uint16_t>(y1));
    Deselect();
    if (Set_SPI_Baudrate(LCD_SPI_LOCAL_PIXEL_BAUDRATEPRESCALER) != HAL_OK)
    {
        return;
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }
    Select();

    uint32_t remaining = total;
    while (remaining > 0U)
    {
        uint32_t chunk = (remaining > LCD_FILL_BUFFER_PIXELS) ? LCD_FILL_BUFFER_PIXELS : remaining;
        Transmit_DMA_Wait(Fill_Buffer, static_cast<uint16_t>(chunk * 2U));
        remaining -= chunk;
    }
    Deselect();
    Set_SPI_Baudrate(LCD_SPI_CONTROL_BAUDRATEPRESCALER);
}

/**
 * @brief 画一个点
 */
void Class_LCD_ST7789::Draw_Point(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Color)
{
    if (__X >= Width || __Y >= Height)
    {
        return;
    }

    Select();
    Set_Window(__X, __Y, __X, __Y);
    Write_Data16(__Color);
    Deselect();
}

/**
 * @brief 画线 (整型 Bresenham, 起止点各画一次)
 */
void Class_LCD_ST7789::Draw_Line(uint16_t __X0, uint16_t __Y0, uint16_t __X1, uint16_t __Y1,
                                 const uint16_t &__Color)
{
    int32_t x0 = __X0;
    int32_t y0 = __Y0;
    int32_t x1 = __X1;
    int32_t y1 = __Y1;
    int32_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int32_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int32_t sx = (x0 < x1) ? 1 : -1;
    int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx - dy;

    for (;;)
    {
        Draw_Point(static_cast<uint16_t>(x0), static_cast<uint16_t>(y0), __Color);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        int32_t e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief 画矩形 (空心或实心)
 */
void Class_LCD_ST7789::Draw_Rectangle(const uint16_t &__X, const uint16_t &__Y,
                                      const uint16_t &__Width, const uint16_t &__Height,
                                      const uint16_t &__Color, const bool &__Filled)
{
    if (__Width == 0U || __Height == 0U)
    {
        return;
    }
    uint16_t x1 = __X + __Width - 1U;
    uint16_t y1 = __Y + __Height - 1U;

    if (__Filled)
    {
        Fill_Rect(__X, __Y, __Width, __Height, __Color);
        return;
    }

    Draw_Line(__X, __Y, x1, __Y, __Color);
    Draw_Line(__X, y1, x1, y1, __Color);
    Draw_Line(__X, __Y, __X, y1, __Color);
    Draw_Line(x1, __Y, x1, y1, __Color);
}

/**
 * @brief 画圆 (中点画圆法, 空心或实心)
 */
void Class_LCD_ST7789::Draw_Circle(const uint16_t &__X, const uint16_t &__Y, const uint16_t &__Radius,
                                   const uint16_t &__Color, const bool &__Filled)
{
    if (__Radius == 0U)
    {
        Draw_Point(__X, __Y, __Color);
        return;
    }

    int32_t a = 0;
    int32_t b = __Radius;
    int32_t di = 3 - (static_cast<int32_t>(__Radius) << 1);
    int32_t cx = __X;
    int32_t cy = __Y;

    // 画一条经屏幕边缘裁剪的水平线段 (用于实心圆)
    auto Draw_HSpan = [&](int32_t xl, int32_t xr, int32_t y)
    {
        if (y < 0 || y >= Height)
        {
            return;
        }
        if (xl < 0)
        {
            xl = 0;
        }
        if (xr >= Width)
        {
            xr = Width - 1;
        }
        if (xl > xr)
        {
            return;
        }
        Fill_Rect(static_cast<uint16_t>(xl), static_cast<uint16_t>(y),
                  static_cast<uint16_t>(xr - xl + 1), 1, __Color);
    };

    while (a <= b)
    {
        if (__Filled)
        {
            // 4 条水平直径填充, 覆盖整个圆 (经边缘裁剪)
            Draw_HSpan(cx - b, cx + b, cy + a);
            Draw_HSpan(cx - b, cx + b, cy - a);
            Draw_HSpan(cx - a, cx + a, cy + b);
            Draw_HSpan(cx - a, cx + a, cy - b);
        }
        else
        {
            Draw_Point(static_cast<uint16_t>(cx + a), static_cast<uint16_t>(cy - b), __Color);
            Draw_Point(static_cast<uint16_t>(cx + b), static_cast<uint16_t>(cy - a), __Color);
            Draw_Point(static_cast<uint16_t>(cx + b), static_cast<uint16_t>(cy + a), __Color);
            Draw_Point(static_cast<uint16_t>(cx + a), static_cast<uint16_t>(cy + b), __Color);
            Draw_Point(static_cast<uint16_t>(cx - a), static_cast<uint16_t>(cy + b), __Color);
            Draw_Point(static_cast<uint16_t>(cx - b), static_cast<uint16_t>(cy + a), __Color);
            Draw_Point(static_cast<uint16_t>(cx - b), static_cast<uint16_t>(cy - a), __Color);
            Draw_Point(static_cast<uint16_t>(cx - a), static_cast<uint16_t>(cy - b), __Color);
        }

        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            --b;
        }
        ++a;
    }
}

/**
 * @brief 在指定位置画一个字符 (背景色填充字符框)
 */
void Class_LCD_ST7789::Draw_Char(const uint16_t &__X, const uint16_t &__Y, const char &__Char,
                                 const uint16_t &__Color, const uint16_t &__Background,
                                 const Struct_LCD_Font &__Font, const uint8_t &__Scale)
{
    if (DMA_Busy)
    {
        Wait_Until_Idle();
    }
    if (__Font.Width == 0U || __Font.Height == 0U || __Scale == 0U)
    {
        return;
    }
    if (__X >= Width || __Y >= Height)
    {
        return;
    }

    uint16_t ch = static_cast<uint8_t>(__Char);
    if (ch < __Font.First_Char || ch >= __Font.First_Char + __Font.Char_Count)
    {
        ch = __Font.First_Char; // 越界回退为空格
    }

    uint16_t index = ch - __Font.First_Char;
    uint16_t bytes_per_row = (static_cast<uint16_t>(__Font.Width) + 7U) / 8U;
    uint32_t base = static_cast<uint32_t>(index) * __Font.Height * bytes_per_row;

    // 缩放后的输出尺寸, 裁剪到屏幕内, 并限制单行像素不超过 Fill_Buffer
    uint16_t out_w = static_cast<uint16_t>(__Font.Width) * __Scale;
    uint16_t out_h = static_cast<uint16_t>(__Font.Height) * __Scale;
    if (out_w > LCD_FILL_BUFFER_PIXELS)
    {
        out_w = LCD_FILL_BUFFER_PIXELS;
    }
    if (static_cast<uint32_t>(__X) + out_w > Width)
    {
        out_w = Width - __X;
    }
    if (static_cast<uint32_t>(__Y) + out_h > Height)
    {
        out_h = Height - __Y;
    }

    Select();
    Set_Window(__X, __Y, __X + out_w - 1U, __Y + out_h - 1U);
    Deselect();
    if (Set_SPI_Baudrate(LCD_SPI_LOCAL_PIXEL_BAUDRATEPRESCALER) != HAL_OK)
    {
        return;
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }
    Select();

    // 按输出行扫描, 每个输出像素映射回字模像素 (整数缩放)
    for (uint16_t out_row = 0U; out_row < out_h; ++out_row)
    {
        uint16_t font_row = out_row / __Scale;
        uint32_t n = 0U;
        for (uint16_t out_col = 0U; out_col < out_w; ++out_col)
        {
            uint16_t font_col = out_col / __Scale;
            uint8_t byte = __Font.Data[base + static_cast<uint32_t>(font_row) * bytes_per_row + (font_col / 8U)];
            uint8_t bit = (byte >> (7U - (font_col % 8U))) & 0x01U;
            uint16_t color = (bit != 0U) ? __Color : __Background;
            Fill_Buffer[n++] = static_cast<uint8_t>(color >> 8);
            Fill_Buffer[n++] = static_cast<uint8_t>(color & 0xffU);
        }
        Transmit_DMA_Wait(Fill_Buffer, static_cast<uint16_t>(n));
    }
    Deselect();
    Set_SPI_Baudrate(LCD_SPI_CONTROL_BAUDRATEPRESCALER);
}

/**
 * @brief 画字符串 (按 Scale 放大)
 */
void Class_LCD_ST7789::Draw_String(const uint16_t &__X, const uint16_t &__Y, const char *__String,
                                   const uint16_t &__Color, const uint16_t &__Background,
                                   const Struct_LCD_Font &__Font, const uint8_t &__Scale)
{
    if (__String == nullptr)
    {
        return;
    }

    uint16_t char_w = static_cast<uint16_t>(__Font.Width) * __Scale;
    uint16_t char_h = static_cast<uint16_t>(__Font.Height) * __Scale;
    uint16_t x = __X;
    uint16_t y = __Y;
    for (uint16_t i = 0U; __String[i] != '\0'; ++i)
    {
        if (x + char_w > Width)
        {
            x = __X;
            y += char_h;
            if (y + char_h > Height)
            {
                break;
            }
        }
        Draw_Char(x, y, __String[i], __Color, __Background, __Font, __Scale);
        x += char_w;
    }
}

/**
 * @brief 画有符号整数 (按 Scale 放大)
 */
void Class_LCD_ST7789::Draw_Number(const uint16_t &__X, const uint16_t &__Y, const int32_t &__Value,
                                   const uint16_t &__Color, const uint16_t &__Background,
                                   const Struct_LCD_Font &__Font, const uint8_t &__Scale)
{
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%ld", static_cast<long>(__Value));
    Draw_String(__X, __Y, buffer, __Color, __Background, __Font, __Scale);
}

/**
 * @brief 绘制原始 RGB565 图像缓冲 (整块刷新, 像素按行优先排列)
 */
void Class_LCD_ST7789::Draw_Image(const uint16_t &__X, const uint16_t &__Y,
                                  const uint16_t &__Width, const uint16_t &__Height,
                                  const uint16_t *__Image)
{
    if (__Image == nullptr || __Width == 0U || __Height == 0U)
    {
        return;
    }
    if (__X + __Width > Width || __Y + __Height > Height)
    {
        return;
    }

    Select();
    Set_Window(__X, __Y, __X + __Width - 1U, __Y + __Height - 1U);
    Deselect();
    if (Set_SPI_Baudrate(LCD_SPI_LOCAL_PIXEL_BAUDRATEPRESCALER) != HAL_OK)
    {
        return;
    }
    if (DC_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(DC_GPIOx, DC_GPIO_Pin, GPIO_PIN_SET);
    }
    Select();
    Write_Pixels(__Image, static_cast<uint32_t>(__Width) * __Height);
    Deselect();
    Set_SPI_Baudrate(LCD_SPI_CONTROL_BAUDRATEPRESCALER);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
