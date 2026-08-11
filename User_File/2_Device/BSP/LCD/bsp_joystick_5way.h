/**
 * @file bsp_joystick_5way.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief LCD 模组自带 5 维摇杆 (ADC 电阻分压) 驱动
 * @version 0.1
 * @date 2026-07-24 0.1 根据《LCD+5维摇杆模块》手册及参考例程新建
 *
 * @note
 * 1. 5 维摇杆 (上/下/左/右/中) 共用一个 ADC 通道, 取与实测理想 ADC 值距离最近的方向.
 * 2. 驱动与 ADC 采集解耦: 调用方把最新 ADC 原始值喂给 Update(), 驱动负责去抖与事件.
 * 3. 默认理想值来自当前模组实测, 不同模组/分压电阻存在差异, 应按实测校准 (见 README).
 * 4. 摇杆 KEY 在本工程接 PA05；PA05 的单端 ADC 输入为 ADC1_INP19。工程将通道 19
 *    加入规则转换序列，并把 Rank2 结果喂给本驱动 (见 README 的 CubeMX 步骤).
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef BSP_JOYSTICK_5WAY_H
#define BSP_JOYSTICK_5WAY_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// ADC 分辨率, 12bit
#define JOYSTICK_ADC_MAX ((uint16_t)4095U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 摇杆方向
 */
enum Enum_Joystick_Direction : uint8_t
{
    Joystick_Direction_None = 0,    // 未按下/悬空
    Joystick_Direction_Center,      // 中
    Joystick_Direction_Up,          // 上
    Joystick_Direction_Down,        // 下
    Joystick_Direction_Left,        // 左
    Joystick_Direction_Right,       // 右
};

/**
 * @brief 各方向及悬空状态的理想 ADC 原始值, 务必按实测校准
 */
struct Struct_Joystick_Ideal_Adc
{
    uint16_t None;
    uint16_t Center;
    uint16_t Up;
    uint16_t Down;
    uint16_t Left;
    uint16_t Right;
};

/* Exported variables --------------------------------------------------------*/

/**
 * @brief 默认理想值 (PA5/ADC1_INP19 单端读数；切换输入模式后需重新实测校准)
 *        解码时选择与当前 ADC 绝对差最小的状态.
 */
static const Struct_Joystick_Ideal_Adc Joystick_Ideal_Adc_Default =
{
    300U,   // None   (悬空)
    2310U,  // Center (中)
    1200U,  // Up     (上)
    800U,   // Down   (下)
    1840U,  // Left   (左)
    1500U,  // Right  (右)
};

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 特化, 5 维摇杆
 */
class Class_Joystick_5Way
{
public:
    /**
     * @brief 初始化
     *
     * @param __Ideal_Adc 各方向与悬空状态的理想 ADC 值
     * @param __Debounce_Ms 去抖时间, 单位 ms
     */
    void Init(const Struct_Joystick_Ideal_Adc &__Ideal_Adc = Joystick_Ideal_Adc_Default,
              const uint16_t &__Debounce_Ms = 20);

    inline Enum_Joystick_Direction Get_Direction() const;
    inline bool Get_Pressed() const;
    inline uint16_t Get_Raw_Adc() const;
    inline bool Get_Direction_Changed() const;
    inline void Clear_Direction_Changed();
    inline bool Get_Just_Pressed() const;
    inline bool Get_Just_Released() const;
    inline void Clear_Edge();

    void Set_Ideal_Adc(const Struct_Joystick_Ideal_Adc &__Ideal_Adc);
    inline const Struct_Joystick_Ideal_Adc &Get_Ideal_Adc() const;

    /**
     * @brief 喂入最新 ADC 原始值, 完成去抖与事件更新. 建议在 1ms 任务中调用.
     */
    void Update(const uint16_t &__Adc_Raw);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    Struct_Joystick_Ideal_Adc Ideal_Adc;
    uint16_t Debounce_Ms = 20;

    // 最近一次原始 ADC
    uint16_t Raw_Adc = JOYSTICK_ADC_MAX;

    // 候选方向 (去抖中)
    Enum_Joystick_Direction Candidate_Direction = Joystick_Direction_None;
    uint64_t Candidate_Timestamp = 0;

    // 读变量

    // 已确认的当前方向
    Enum_Joystick_Direction Direction = Joystick_Direction_None;
    // 方向是否发生改变 (供消费)
    bool Direction_Changed = false;
    // 边沿事件: 本周期刚按下 / 刚释放
    bool Just_Pressed = false;
    bool Just_Released = false;

    // 写变量

    // 读写变量

    // 内部函数

    Enum_Joystick_Direction Decode(const uint16_t &__Adc_Raw) const;
};

/* Exported variables --------------------------------------------------------*/

extern Class_Joystick_5Way BSP_Joystick;

/* Inline functions ----------------------------------------------------------*/

inline Enum_Joystick_Direction Class_Joystick_5Way::Get_Direction() const
{
    return (Direction);
}

inline bool Class_Joystick_5Way::Get_Pressed() const
{
    return (Direction != Joystick_Direction_None);
}

inline uint16_t Class_Joystick_5Way::Get_Raw_Adc() const
{
    return (Raw_Adc);
}

inline bool Class_Joystick_5Way::Get_Direction_Changed() const
{
    return (Direction_Changed);
}

inline void Class_Joystick_5Way::Clear_Direction_Changed()
{
    Direction_Changed = false;
}

inline bool Class_Joystick_5Way::Get_Just_Pressed() const
{
    return (Just_Pressed);
}

inline bool Class_Joystick_5Way::Get_Just_Released() const
{
    return (Just_Released);
}

inline void Class_Joystick_5Way::Clear_Edge()
{
    Just_Pressed = false;
    Just_Released = false;
}

inline const Struct_Joystick_Ideal_Adc &Class_Joystick_5Way::Get_Ideal_Adc() const
{
    return (Ideal_Adc);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
