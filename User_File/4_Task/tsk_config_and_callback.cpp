/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "spi.h"
#include "2_Device/BSP/LCD/bsp_lcd.h"
#include "2_Device/BSP/LCD/bsp_joystick_5way.h"
#include "3_Application/Crane/alg_crane_control.h"
#include "3_Application/MU/alg_mu_platform.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{

}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    TIM_1ms_IWDG_PeriodElapsedCallback();

    Crane_Control.TIM_1ms_PeriodElapsedCallback();
    MU_Platform.TIM_1ms_PeriodElapsedCallback();

    // 5 维摇杆: 喂入 ADC1 通道19单端(Rank2) 原始值并去抖 (只做轻量更新, 刷屏放主循环)
    BSP_Joystick.Update(ADC1_Manage_Object.ADC_Data[1]);
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    MU_Platform.TIM_125us_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    MU_Platform.TIM_10us_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);


    BSP_Power.Init(1,1,1);

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    // MUsss platform: BMI088, CAN1 motors and 3-RSS inverse kinematics.
    // 平衡台使能 CAN1 电机，并将每次上电位置写入电机 Flash 作为零点。
    MU_Platform.Init();

    // 吊机 CAN2 电机、PE13/PE9 直线电机与 USART1 上位机初始化。
    // 该模块上电后保持安全关闭，只有收到 SAFE,1 且心跳持续时才允许输出。
    Crane_Control.Init();

    // ADC1 启动 DMA 采样, 2 个规则通道: Rank1=通道4(电池), Rank2=通道19单端(摇杆)
    ADC_Init(&hadc1, 2);

    // LCD 初始化 (SPI1, CS=PE15, DC=PD10, RST=PB11, BL=PB10)
    // 注: PB10/PB11 在 CubeMX 里原是 I2C2_SCL/SDA, 本板复用为 背光/复位, 驱动会重新配为推挽输出
    BSP_LCD.Init(&hspi1, GPIOE, GPIO_PIN_15, GPIOD, GPIO_PIN_10,
                 GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_10);
    BSP_LCD.Fill(LCD_COLOR_BLACK);
    BSP_LCD.Draw_String(4, 4, "LCD OK", LCD_COLOR_WHITE, LCD_COLOR_BLACK, LCD_Font_24, 2);

    // 5 维摇杆初始化 (默认占位阈值, 务必按 README 第四节实测校准)
    BSP_Joystick.Init();

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
// ---- 智能刷新辅助: 只在内容变化时重绘, 仅在变短时清残留尾部 (625kHz 下减少 SPI 字节) ----

static uint8_t LCD_Str_Len(const char *s)
{
    uint8_t n = 0;
    if (s != nullptr)
    {
        while (s[n] != '\0')
        {
            ++n;
        }
    }
    return (n);
}

static uint8_t LCD_Int_Len(int32_t v)
{
    uint8_t n = (v < 0) ? 1U : 0U;
    uint32_t a = (v < 0) ? static_cast<uint32_t>(-v) : static_cast<uint32_t>(v);
    if (a == 0U)
    {
        return (n + 1U);
    }
    while (a != 0U)
    {
        ++n;
        a /= 10U;
    }
    return (n);
}

// 刷新字符串: 不整块清屏, 仅当新串比旧串短时清掉残留尾部
static void LCD_Draw_String_Smart(const char *str, uint16_t x, uint16_t y,
                                  uint8_t &last_len, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint8_t new_len = LCD_Str_Len(str);
    uint16_t cw = static_cast<uint16_t>(17U * scale);
    uint16_t ch = static_cast<uint16_t>(24U * scale);
    if (new_len < last_len)
    {
        BSP_LCD.Fill_Rect(static_cast<uint16_t>(x + new_len * cw), y,
                          static_cast<uint16_t>((last_len - new_len) * cw), ch, bg);
    }
    BSP_LCD.Draw_String(x, y, str, color, bg, LCD_Font_24, scale);
    last_len = new_len;
}

// 刷新整数: 仅当 |变化| >= threshold 时重绘; 仅当位数变短时清残留尾部
static void LCD_Draw_Int_Smart(int32_t value, uint16_t x, uint16_t y,
                               int32_t &last_value, uint8_t &last_len,
                               uint16_t color, uint16_t bg, uint8_t scale, int32_t threshold)
{
    int32_t delta = value - last_value;
    if (delta < 0)
    {
        delta = -delta;
    }
    if (delta < threshold)
    {
        return;
    }

    uint8_t new_len = LCD_Int_Len(value);
    uint16_t cw = static_cast<uint16_t>(17U * scale);
    uint16_t ch = static_cast<uint16_t>(24U * scale);
    if (new_len < last_len)
    {
        BSP_LCD.Fill_Rect(static_cast<uint16_t>(x + new_len * cw), y,
                          static_cast<uint16_t>((last_len - new_len) * cw), ch, bg);
    }
    BSP_LCD.Draw_Number(x, y, value, color, bg, LCD_Font_24, scale);
    last_value = value;
    last_len = new_len;
}

void Task_Loop()
{
    Crane_Control.Loop();
    MU_Platform.Loop();

    // 稳态 (摇杆不动、电池平稳) 下零刷屏; 仅在方向/数值变化时才重绘对应字段.
    static Enum_Joystick_Direction Last_Direction = Joystick_Direction_None;
    static bool First_Draw = true;
    static uint8_t Last_Dir_Len = 0;
    static int32_t Last_Joy = -1000000;
    static int32_t Last_Bat = -1000000;
    static uint8_t Last_Joy_Len = 0;
    static uint8_t Last_Bat_Len = 0;

    Enum_Joystick_Direction dir = BSP_Joystick.Get_Direction();
    if (First_Draw || dir != Last_Direction)
    {
        const char *name;
        switch (dir)
        {
        case Joystick_Direction_Center: name = "Center"; break;
        case Joystick_Direction_Up:     name = "Up";     break;
        case Joystick_Direction_Down:   name = "Down";   break;
        case Joystick_Direction_Left:   name = "Left";   break;
        case Joystick_Direction_Right:  name = "Right";  break;
        default:                        name = "None";   break;
        }
        LCD_Draw_String_Smart(name, 4, 72, Last_Dir_Len, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 2);
        Last_Direction = dir;
        First_Draw = false;
    }

    // JOY = 摇杆通道 (ADC1 ch19 单端/PA5), BAT = 电池通道 (ADC1 ch4)
    LCD_Draw_Int_Smart(BSP_Joystick.Get_Raw_Adc(), 4, 140, Last_Joy, Last_Joy_Len,
                       LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 2, 20);
    LCD_Draw_Int_Smart(ADC1_Manage_Object.ADC_Data[0], 4, 196, Last_Bat, Last_Bat_Len,
                       LCD_COLOR_GREEN, LCD_COLOR_BLACK, 2, 40);
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }
    MU_Platform.EXTI_Flag_Callback(GPIO_Pin);
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
