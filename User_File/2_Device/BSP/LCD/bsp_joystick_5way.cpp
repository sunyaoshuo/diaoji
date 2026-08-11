/**
 * @file bsp_joystick_5way.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD 模组自带 5 维摇杆 (ADC 电阻分压) 驱动
 * @version 0.1
 * @date 2026-07-24 0.1 根据《LCD+5维摇杆模块》手册及参考例程新建
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_joystick_5way.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Joystick_5Way BSP_Joystick;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 */
void Class_Joystick_5Way::Init(const Struct_Joystick_Ideal_Adc &__Ideal_Adc,
                               const uint16_t &__Debounce_Ms)
{
    Ideal_Adc = __Ideal_Adc;
    Debounce_Ms = __Debounce_Ms;

    Raw_Adc = JOYSTICK_ADC_MAX;
    Candidate_Direction = Joystick_Direction_None;
    Candidate_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    Direction = Joystick_Direction_None;
    Direction_Changed = false;
    Just_Pressed = false;
    Just_Released = false;
}

/**
 * @brief 设置各状态的理想 ADC 值
 */
void Class_Joystick_5Way::Set_Ideal_Adc(const Struct_Joystick_Ideal_Adc &__Ideal_Adc)
{
    Ideal_Adc = __Ideal_Adc;
}

/**
 * @brief 将 ADC 原始值解码为与理想值绝对距离最近的状态
 */
Enum_Joystick_Direction Class_Joystick_5Way::Decode(const uint16_t &__Adc_Raw) const
{
    const uint16_t ideal_values[6] =
    {
        Ideal_Adc.None,
        Ideal_Adc.Center,
        Ideal_Adc.Up,
        Ideal_Adc.Down,
        Ideal_Adc.Left,
        Ideal_Adc.Right,
    };

    uint8_t nearest_index = 0U;
    uint16_t nearest_distance = (__Adc_Raw >= ideal_values[0]) ?
                                (__Adc_Raw - ideal_values[0]) : (ideal_values[0] - __Adc_Raw);

    for (uint8_t i = 1U; i < 6U; ++i)
    {
        const uint16_t distance = (__Adc_Raw >= ideal_values[i]) ?
                                  (__Adc_Raw - ideal_values[i]) : (ideal_values[i] - __Adc_Raw);
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest_index = i;
        }
    }

    return (static_cast<Enum_Joystick_Direction>(nearest_index));
}

/**
 * @brief 喂入最新 ADC 原始值并更新去抖状态与边沿事件
 */
void Class_Joystick_5Way::Update(const uint16_t &__Adc_Raw)
{
    // 边沿事件按 "本周期" 语义, 每次 Update 先清零, 避免不调用 Clear_Edge 时卡住
    Just_Pressed = false;
    Just_Released = false;

    Raw_Adc = __Adc_Raw;

    Enum_Joystick_Direction decoded = Decode(__Adc_Raw);
    uint64_t now = SYS_Timestamp.Get_Current_Timestamp();

    // 新候选: 重置计时
    if (decoded != Candidate_Direction)
    {
        Candidate_Direction = decoded;
        Candidate_Timestamp = now;
    }

    // 候选方向稳定超过去抖时间则确认为当前方向
    if ((now - Candidate_Timestamp) >= static_cast<uint64_t>(Debounce_Ms) * 1000ULL)
    {
        if (Candidate_Direction != Direction)
        {
            bool was_pressed = (Direction != Joystick_Direction_None);
            bool now_pressed = (Candidate_Direction != Joystick_Direction_None);

            Direction = Candidate_Direction;
            Direction_Changed = true;
            Just_Pressed = (!was_pressed && now_pressed);
            Just_Released = (was_pressed && !now_pressed);
        }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
