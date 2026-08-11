/**
 * @file bsp_lcd_register.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD (ST7789V2) 寄存器定义与初始化指令表
 * @version 0.1
 * @date 2026-07-24 0.1 根据《LCD+5维摇杆模块》手册及参考例程新建
 *
 * @note
 * 1. 驱动芯片 ST7789V2, 物理分辨率 240x280, RGB565, 4 线 SPI.
 * 2. 初始化序列取自该模组参考例程 (Waveshare LCD_1in69 / ST7789V), 已在该屏验证可用.
 * 3. 240x280 屏在 240x320 GRAM 中存在 Y 方向 +20 像素偏移, 见 LCD_Y_OFFSET.
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef BSP_LCD_REGISTER_H
#define BSP_LCD_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros ----------------------------------------------------------*/

// 面板分辨率
#define LCD_WIDTH ((uint16_t)240)
#define LCD_HEIGHT ((uint16_t)280)

// 240x280 屏在 240x320 GRAM 中的地址偏移, 竖屏 MADCTL=0x00 时 X 无偏移, Y 偏移 20
#define LCD_X_OFFSET ((uint16_t)0)
#define LCD_Y_OFFSET ((uint16_t)20)

// ST7789V2 命令
#define LCD_CMD_NOP ((uint8_t)0x00)
#define LCD_CMD_SWRESET ((uint8_t)0x01)
#define LCD_CMD_SLPIN ((uint8_t)0x10)
#define LCD_CMD_SLPOUT ((uint8_t)0x11)
#define LCD_CMD_PTLON ((uint8_t)0x12)
#define LCD_CMD_NORON ((uint8_t)0x13)
#define LCD_CMD_INVOFF ((uint8_t)0x20)
#define LCD_CMD_INVON ((uint8_t)0x21)
#define LCD_CMD_DISPOFF ((uint8_t)0x28)
#define LCD_CMD_DISPON ((uint8_t)0x29)
#define LCD_CMD_CASET ((uint8_t)0x2A)
#define LCD_CMD_RASET ((uint8_t)0x2B)
#define LCD_CMD_RAMWR ((uint8_t)0x2C)
#define LCD_CMD_RAMRD ((uint8_t)0x2E)
#define LCD_CMD_MADCTL ((uint8_t)0x36)
#define LCD_CMD_COLMOD ((uint8_t)0x3A)
#define LCD_CMD_PORCTRL ((uint8_t)0xB2)
#define LCD_CMD_GCTRL ((uint8_t)0xB7)
#define LCD_CMD_VCOMS ((uint8_t)0xBB)
#define LCD_CMD_LCMCTRL ((uint8_t)0xC0)
#define LCD_CMD_VRHEN ((uint8_t)0xC2)
#define LCD_CMD_VRHS ((uint8_t)0xC3)
#define LCD_CMD_VDVS ((uint8_t)0xC4)
#define LCD_CMD_FRCTRL2 ((uint8_t)0xC6)
#define LCD_CMD_PWCTRL1 ((uint8_t)0xD0)
#define LCD_CMD_PGC ((uint8_t)0xE0)
#define LCD_CMD_NGC ((uint8_t)0xE1)

// MADCTL 位
#define LCD_MADCTL_MY ((uint8_t)0x80)
#define LCD_MADCTL_MX ((uint8_t)0x40)
#define LCD_MADCTL_MV ((uint8_t)0x20)
#define LCD_MADCTL_ML ((uint8_t)0x10)
#define LCD_MADCTL_BGR ((uint8_t)0x08)
#define LCD_MADCTL_MH ((uint8_t)0x04)

// 像素格式: 0x05 = 16bit/像素 RGB565 (MCU 接口 DBI=101)
#define LCD_COLMOD_16BPP ((uint8_t)0x05)

// RGB565 颜色合成, 8bit 输入
#define LCD_COLOR_RGB565(R, G, B) \
    ((uint16_t)((((uint8_t)(R) & 0xF8U) << 8U) | \
                (((uint8_t)(G) & 0xFCU) << 3U) | \
                (((uint8_t)(B)) >> 3U)))

// 常用颜色 (RGB565)
#define LCD_COLOR_BLACK   ((uint16_t)0x0000)
#define LCD_COLOR_WHITE   ((uint16_t)0xFFFF)
#define LCD_COLOR_RED     ((uint16_t)0xF800)
#define LCD_COLOR_GREEN   ((uint16_t)0x07E0)
#define LCD_COLOR_BLUE    ((uint16_t)0x001F)
#define LCD_COLOR_CYAN    ((uint16_t)0x07FF)
#define LCD_COLOR_MAGENTA ((uint16_t)0xF81F)
#define LCD_COLOR_YELLOW  ((uint16_t)0xFFE0)
#define LCD_COLOR_ORANGE  ((uint16_t)0xFD20)
#define LCD_COLOR_GRAY    ((uint16_t)0x8410)

/* Exported types -----------------------------------------------------------*/

/**
 * @brief 一条初始化指令: 命令 + 参数 + 后置延时
 */
struct Struct_LCD_Init_Cmd
{
    uint8_t Command;
    uint8_t Data_Length;
    uint8_t Data[16];
    uint16_t Delay_Ms;
};

/* Exported variables -------------------------------------------------------*/

/**
 * @brief ST7789V2 初始化指令序列
 *        取自该模组参考例程, 已在 240x280 屏验证可用.
 */
static const Struct_LCD_Init_Cmd LCD_INIT_COMMANDS[] =
{
    {LCD_CMD_SWRESET, 0, {0x00}, 150},
    {LCD_CMD_SLPOUT, 0, {0x00}, 120},

    {LCD_CMD_MADCTL, 1, {0x00}, 0},
    {LCD_CMD_COLMOD, 1, {LCD_COLMOD_16BPP}, 0},

    {LCD_CMD_PORCTRL, 5, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 0},
    {LCD_CMD_GCTRL, 1, {0x35}, 0},
    {LCD_CMD_VCOMS, 1, {0x3B}, 0},
    {LCD_CMD_LCMCTRL, 1, {0x2C}, 0},
    {LCD_CMD_VRHEN, 1, {0x01}, 0},
    {LCD_CMD_VRHS, 1, {0x16}, 0},
    {LCD_CMD_VDVS, 1, {0x20}, 0},
    {LCD_CMD_FRCTRL2, 1, {0x0F}, 0},
    {LCD_CMD_PWCTRL1, 2, {0xA4, 0xA1}, 0},

    {LCD_CMD_PGC, 14,
     {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54,
      0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23, 0x00, 0x00}, 0},

    {LCD_CMD_NGC, 14,
     {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44,
      0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23, 0x00, 0x00}, 0},

    {LCD_CMD_INVON, 0, {0x00}, 0},
    {LCD_CMD_SLPOUT, 0, {0x00}, 120},
    {LCD_CMD_DISPON, 0, {0x00}, 20},
};

#define LCD_INIT_COMMANDS_COUNT \
    (sizeof(LCD_INIT_COMMANDS) / sizeof(LCD_INIT_COMMANDS[0]))

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
