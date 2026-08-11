/**
 * @file alg_crane_control.cpp
 * @brief 吊机控制实现
 *
 * USART1, 115200 8N1, ASCII 行协议：
 *   PING\n
 *   SAFE,0|1\n
 *   SET,EXT,<centidegree>\n
 *   SET,WINCH,<centidegree>\n
 *   SET,YAW,<centidegree>\n
 *   SPEED,EXT,<centiradian_per_second>\n
 *   SPEED,WINCH,<centiradian_per_second>\n
 *   SPEED,YAW,<centiradian_per_second>\n
 *   LIMIT,EXT,<min_centidegree>,<max_centidegree>\n
 *   LIMIT,YAW,<min_centidegree>,<max_centidegree>\n
 *   PITCH,UP|DOWN|STOP\n
 *
 * 遥测：
 *   TEL,<safe_requested>,<outputs_enabled>,<ext_cdeg>,<ext_online>,
 *       <winch_cdeg>,<winch_online>,<yaw_cdeg>,<yaw_online>,
 *       <pitch_direction>,<command_age_ms>,<rx_overflow_count>,<tx_error_count>,
 *       <ext_speed_cradps>,<winch_speed_cradps>,<yaw_speed_cradps>,
 *       <ext_min_cdeg>,<ext_max_cdeg>,<yaw_min_cdeg>,<yaw_max_cdeg>\n
 */

#include "alg_crane_control.h"

#include "fdcan.h"
#include "main.h"
#include "usart.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
constexpr float CRANE_PI = 3.14159265358979323846f;
constexpr float EXTENSION_DEFAULT_SPEED_RADPS = 2.0f;
constexpr float WINCH_DEFAULT_SPEED_RADPS = 3.0f;
constexpr float RS_ACCELERATION_RADPS2 = 8.0f;
constexpr float RS_CURRENT_LIMIT_A = 5.0f;
constexpr float YAW_DEFAULT_SPEED_RADPS = 1.5f;

void Crane_CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    Crane_Control.CAN2_RxCpltCallback(Header, Buffer);
}

}

Class_Crane_Control Crane_Control;

void Class_Crane_Control::Init()
{
    // PE13/PE9 原为 TIM1 PWM 引脚。此应用将它们最终重配为直线电机方向输出。
    // 先清零 ODR，再切换到推挽输出，避免初始化瞬间误动作。
    HAL_GPIO_WritePin(GPIOE, PITCH_MOTOR_A_Pin | PITCH_MOTOR_B_Pin, GPIO_PIN_RESET);
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = PITCH_MOTOR_A_Pin | PITCH_MOTOR_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // 两台 RobStride 电机均挂载 CAN2，ID 1/2，PP 位置模式。
    Extension_Motor.Init(&hfdcan2, 1, RS_MASTER_ID);
    Extension_Motor.Set_Control_Method(Motor_RS_Control_Method_POSITION_PP);
    Extension_Motor.Set_Control_Speed_Limit(EXTENSION_DEFAULT_SPEED_RADPS);
    Extension_Motor.Set_Control_Acceleration(RS_ACCELERATION_RADPS2);
    Extension_Motor.Set_Control_Deceleration(RS_ACCELERATION_RADPS2);
    Extension_Motor.Set_Control_Current_Limit(RS_CURRENT_LIMIT_A);

    Winch_Motor.Init(&hfdcan2, 2, RS_MASTER_ID);
    Winch_Motor.Set_Control_Method(Motor_RS_Control_Method_POSITION_PP);
    Winch_Motor.Set_Control_Speed_Limit(WINCH_DEFAULT_SPEED_RADPS);
    Winch_Motor.Set_Control_Acceleration(RS_ACCELERATION_RADPS2);
    Winch_Motor.Set_Control_Deceleration(RS_ACCELERATION_RADPS2);
    Winch_Motor.Set_Control_Current_Limit(RS_CURRENT_LIMIT_A);

    // 达妙 YAW：MASTER_ID=0x00，CAN_ID=0x01，位置-速度模式，挂载 CAN2。
    // 驱动会按协议将位置模式发送 ID 映射为 CAN_ID + 0x100 = 0x101。
    Yaw_Motor.Init(&hfdcan2,
                   0x00,
                   0x01,
                   Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    Yaw_Motor.Set_Control_Omega(YAW_DEFAULT_SPEED_RADPS);

    CAN_Init(&hfdcan2, Crane_CAN2_Callback);
    // USART1 使用硬件循环 DMA，由主循环按 DMA 写指针取数。避免共享 UART 驱动的
    // Receive-to-Idle 回调在中断内停止并重启 DMA，连续心跳下可能导致接收链路卡死。
    if (HAL_UART_Receive_DMA(&huart1, UART_DMA_Buffer, UART_DMA_SIZE) == HAL_OK)
    {
        // 循环 DMA 不依赖半满/全满回调推进，关闭这两个中断以减少无意义中断负载。
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    else
    {
        ++UART_Ring_Overflow_Count;
    }

    // 上电默认关闭全部输出，并显式向 CAN 电机发送失能/停止帧。
    Disable_Outputs(true);
    Last_Valid_Command_ms = HAL_GetTick();
    Last_Telemetry_ms = HAL_GetTick();
}

void Class_Crane_Control::Loop()
{
    Process_UART();

    const uint32_t Now_ms = HAL_GetTick();
    if (Safety_Requested && ((Now_ms - Last_Valid_Command_ms) > COMMAND_TIMEOUT_MS))
    {
        Disable_Outputs(true);
    }

    Handle_Startup(Now_ms);

    if (Outputs_Enabled &&
        (Pitch_Direction != Crane_Pitch_Direction_STOP) &&
        ((Now_ms - Last_Pitch_Command_ms) > PITCH_DEADMAN_TIMEOUT_MS))
    {
        Apply_Pitch_Output(Crane_Pitch_Direction_STOP);
    }

    if (Telemetry_Requested || ((Now_ms - Last_Telemetry_ms) >= TELEMETRY_PERIOD_MS))
    {
        Telemetry_Requested = false;
        Last_Telemetry_ms = Now_ms;
        Send_Telemetry(Now_ms);
    }
}

void Class_Crane_Control::TIM_1ms_PeriodElapsedCallback()
{
    static uint8_t Control_Divider = 0;
    static uint8_t Alive_Divider = 0;

    if (++Control_Divider >= 10)
    {
        Control_Divider = 0;
        if (Outputs_Enabled)
        {
            Extension_Motor.TIM_Send_PeriodElapsedCallback();
            Winch_Motor.TIM_Send_PeriodElapsedCallback();
            Yaw_Motor.TIM_Send_PeriodElapsedCallback();
        }
    }

    if (++Alive_Divider >= 100)
    {
        Alive_Divider = 0;
        Extension_Motor.TIM_100ms_Alive_PeriodElapsedCallback();
        Winch_Motor.TIM_100ms_Alive_PeriodElapsedCallback();

        // DM 驱动的存活回调可能自动发送使能帧，因此仅在安全输出已打开时调用。
        if (Outputs_Enabled)
        {
            Yaw_Motor.TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}

void Class_Crane_Control::CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    const uint32_t Now_ms = HAL_GetTick();

    if (Header.IdType == FDCAN_EXTENDED_ID)
    {
        Extension_Motor.CAN_RxCpltCallback(Header, Buffer);
        Winch_Motor.CAN_RxCpltCallback(Header, Buffer);

        const Struct_Motor_RS_Extended_ID ID = Class_Motor_RS::Parse_Extended_ID(Header.Identifier);
        const uint8_t Source_ID = static_cast<uint8_t>(ID.Data_Field & 0xFFu);
        if (ID.Target_ID == RS_MASTER_ID)
        {
            if (Source_ID == 1)
            {
                Extension_Feedback_Received = true;
                Extension_Last_Feedback_ms = Now_ms;
            }
            else if (Source_ID == 2)
            {
                Winch_Feedback_Received = true;
                Winch_Last_Feedback_ms = Now_ms;
            }
        }
    }
    else if ((Header.IdType == FDCAN_STANDARD_ID) && (Header.Identifier == 0x00u))
    {
        Yaw_Motor.CAN_RxCpltCallback();
        Yaw_Feedback_Received = true;
        Yaw_Last_Feedback_ms = Now_ms;
    }
}

void Class_Crane_Control::UART1_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr)
    {
        return;
    }

    for (uint16_t Index = 0; Index < Length; ++Index)
    {
        const uint16_t Next = static_cast<uint16_t>((UART_Ring_Head + 1u) % UART_RING_SIZE);
        if (Next == UART_Ring_Tail)
        {
            ++UART_Ring_Overflow_Count;
            break;
        }

        UART_Ring[UART_Ring_Head] = Buffer[Index];
        __DMB();
        UART_Ring_Head = Next;
    }
}

void Class_Crane_Control::Process_UART()
{
    // DMA NDTR 表示尚未传输的字节数；换算为当前硬件写入位置并搬入软件环形队列。
    // DCache 未启用，因此 CPU 可直接读取 DMA 更新后的缓冲区。
    const uint16_t DMA_Head = static_cast<uint16_t>(
        (UART_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) % UART_DMA_SIZE);
    while (UART_DMA_Tail != DMA_Head)
    {
        const uint16_t Next = static_cast<uint16_t>((UART_Ring_Head + 1u) % UART_RING_SIZE);
        if (Next == UART_Ring_Tail)
        {
            ++UART_Ring_Overflow_Count;
            break;
        }

        UART_Ring[UART_Ring_Head] = UART_DMA_Buffer[UART_DMA_Tail];
        UART_Ring_Head = Next;
        UART_DMA_Tail = static_cast<uint16_t>((UART_DMA_Tail + 1u) % UART_DMA_SIZE);
    }

    while (UART_Ring_Tail != UART_Ring_Head)
    {
        const uint8_t Byte = UART_Ring[UART_Ring_Tail];
        UART_Ring_Tail = static_cast<uint16_t>((UART_Ring_Tail + 1u) % UART_RING_SIZE);

        if (Byte == '\r')
        {
            continue;
        }

        if (Byte == '\n')
        {
            if (UART_Line_Length > 0)
            {
                UART_Line[UART_Line_Length] = '\0';
                Parse_Command(UART_Line);
                UART_Line_Length = 0;
            }
            continue;
        }

        if ((Byte >= 0x20u) && (Byte <= 0x7Eu))
        {
            if (UART_Line_Length < (UART_LINE_SIZE - 1u))
            {
                UART_Line[UART_Line_Length++] = static_cast<char>(Byte);
            }
            else
            {
                UART_Line_Length = 0;
                ++UART_Ring_Overflow_Count;
            }
        }
    }
}

void Class_Crane_Control::Parse_Command(char *Line)
{
    if (Line == nullptr)
    {
        return;
    }

    const uint32_t Now_ms = HAL_GetTick();
    bool Valid = false;

    if (std::strcmp(Line, "PING") == 0)
    {
        // 心跳同时请求立即回传一帧，避免上位机只能依赖周期调度判断链路。
        Telemetry_Requested = true;
        Valid = true;
    }
    else if (std::strcmp(Line, "SAFE,1") == 0)
    {
        Request_Safety(true, Now_ms);
        Valid = true;
    }
    else if (std::strcmp(Line, "SAFE,0") == 0)
    {
        Request_Safety(false, Now_ms);
        Valid = true;
    }
    else
    {
        struct Axis_Command
        {
            const char *Prefix;
            int32_t Min;
            int32_t Max;
            uint8_t Axis;
        };

        static constexpr Axis_Command Commands[] = {
            {"SET,EXT,", CRANE_EXTENSION_MIN_CDEG, CRANE_EXTENSION_MAX_CDEG, 0},
            {"SET,WINCH,", CRANE_WINCH_MIN_CDEG, CRANE_WINCH_MAX_CDEG, 1},
            {"SET,YAW,", CRANE_YAW_MIN_CDEG, CRANE_YAW_MAX_CDEG, 2},
        };

        for (const Axis_Command &Command : Commands)
        {
            const size_t Prefix_Length = std::strlen(Command.Prefix);
            if (std::strncmp(Line, Command.Prefix, Prefix_Length) != 0)
            {
                continue;
            }

            char *End = nullptr;
            const long Parsed = std::strtol(Line + Prefix_Length, &End, 10);
            if ((End == (Line + Prefix_Length)) || (*End != '\0') ||
                (Parsed < static_cast<long>(INT32_MIN)) || (Parsed > static_cast<long>(INT32_MAX)))
            {
                break;
            }

            int32_t Active_Min = Command.Min;
            int32_t Active_Max = Command.Max;
            if (Command.Axis == 0u)
            {
                Active_Min = Extension_Min_Cdeg;
                Active_Max = Extension_Max_Cdeg;
            }
            else if (Command.Axis == 2u)
            {
                Active_Min = Yaw_Min_Cdeg;
                Active_Max = Yaw_Max_Cdeg;
            }
            const int32_t Target = Clamp_Centidegree(static_cast<int32_t>(Parsed), Active_Min, Active_Max);
            if (Outputs_Enabled)
            {
                if (Command.Axis == 0)
                {
                    Extension_Motor.Set_Control_Angle(Centidegree_To_Radian(Target));
                }
                else if (Command.Axis == 1)
                {
                    Winch_Motor.Set_Control_Angle(Centidegree_To_Radian(Target));
                }
                else
                {
                    Yaw_Motor.Set_Control_Angle(Centidegree_To_Radian(Target));
                }
            }
            Valid = true;
            break;
        }

        struct Speed_Command
        {
            const char *Prefix;
            int32_t Min;
            int32_t Max;
            uint8_t Axis;
        };

        static constexpr Speed_Command Speed_Commands[] = {
            {"SPEED,EXT,", CRANE_RS_SPEED_MIN_CRADPS, CRANE_RS_SPEED_MAX_CRADPS, 0},
            {"SPEED,WINCH,", CRANE_RS_SPEED_MIN_CRADPS, CRANE_RS_SPEED_MAX_CRADPS, 1},
            {"SPEED,YAW,", CRANE_YAW_SPEED_MIN_CRADPS, CRANE_YAW_SPEED_MAX_CRADPS, 2},
        };

        if (!Valid)
        {
            for (const Speed_Command &Command : Speed_Commands)
            {
                const size_t Prefix_Length = std::strlen(Command.Prefix);
                if (std::strncmp(Line, Command.Prefix, Prefix_Length) != 0)
                {
                    continue;
                }

                char *End = nullptr;
                const long Parsed = std::strtol(Line + Prefix_Length, &End, 10);
                if ((End != (Line + Prefix_Length)) && (*End == '\0') &&
                    (Parsed >= Command.Min) && (Parsed <= Command.Max))
                {
                    const float Speed_radps = static_cast<float>(Parsed) / 100.0f;
                    if (Command.Axis == 0u)
                    {
                        Extension_Motor.Set_Control_Speed_Limit(Speed_radps);
                        if (Outputs_Enabled) Extension_Motor.CAN_Send_Set_PP_Max_Speed(Speed_radps);
                    }
                    else if (Command.Axis == 1u)
                    {
                        Winch_Motor.Set_Control_Speed_Limit(Speed_radps);
                        if (Outputs_Enabled) Winch_Motor.CAN_Send_Set_PP_Max_Speed(Speed_radps);
                    }
                    else
                    {
                        Yaw_Motor.Set_Control_Omega(Speed_radps);
                    }
                    Valid = true;
                }
                break;
            }
        }

        struct Limit_Command
        {
            const char *Prefix;
            int32_t Hard_Min;
            int32_t Hard_Max;
            uint8_t Axis;
        };

        static constexpr Limit_Command Limit_Commands[] = {
            {"LIMIT,EXT,", CRANE_EXTENSION_MIN_CDEG, CRANE_EXTENSION_MAX_CDEG, 0},
            {"LIMIT,YAW,", CRANE_YAW_MIN_CDEG, CRANE_YAW_MAX_CDEG, 2},
        };

        if (!Valid)
        {
            for (const Limit_Command &Command : Limit_Commands)
            {
                const size_t Prefix_Length = std::strlen(Command.Prefix);
                if (std::strncmp(Line, Command.Prefix, Prefix_Length) != 0)
                {
                    continue;
                }

                char *Separator = nullptr;
                const char *Min_Text = Line + Prefix_Length;
                const long Parsed_Min = std::strtol(Min_Text, &Separator, 10);
                if ((Separator != Min_Text) && (*Separator == ','))
                {
                    char *End = nullptr;
                    const char *Max_Text = Separator + 1;
                    const long Parsed_Max = std::strtol(Max_Text, &End, 10);
                    if ((End != Max_Text) && (*End == '\0') &&
                        (Parsed_Min >= Command.Hard_Min) &&
                        (Parsed_Max <= Command.Hard_Max) &&
                        (Parsed_Min < Parsed_Max))
                    {
                        if (Command.Axis == 0u)
                        {
                            Extension_Min_Cdeg = static_cast<int32_t>(Parsed_Min);
                            Extension_Max_Cdeg = static_cast<int32_t>(Parsed_Max);
                        }
                        else
                        {
                            Yaw_Min_Cdeg = static_cast<int32_t>(Parsed_Min);
                            Yaw_Max_Cdeg = static_cast<int32_t>(Parsed_Max);
                        }
                        Valid = true;
                        Telemetry_Requested = true;
                    }
                }
                break;
            }
        }

        if (std::strcmp(Line, "PITCH,UP") == 0)
        {
            if (Outputs_Enabled)
            {
                Apply_Pitch_Output(Crane_Pitch_Direction_UP);
            }
            Last_Pitch_Command_ms = Now_ms;
            Valid = true;
        }
        else if (std::strcmp(Line, "PITCH,DOWN") == 0)
        {
            if (Outputs_Enabled)
            {
                Apply_Pitch_Output(Crane_Pitch_Direction_DOWN);
            }
            Last_Pitch_Command_ms = Now_ms;
            Valid = true;
        }
        else if (std::strcmp(Line, "PITCH,STOP") == 0)
        {
            Apply_Pitch_Output(Crane_Pitch_Direction_STOP);
            Last_Pitch_Command_ms = Now_ms;
            Valid = true;
        }
    }

    if (Valid)
    {
        Last_Valid_Command_ms = Now_ms;
    }
}

void Class_Crane_Control::Request_Safety(bool Enable, uint32_t Now_ms)
{
    if (!Enable)
    {
        Disable_Outputs(true);
        return;
    }

    if (Safety_Requested)
    {
        return;
    }

    // 无扰启用：以当前反馈位置作为初始目标，避免仅打开安全开关就突然回零。
    Extension_Motor.Set_Control_Angle(Extension_Motor.Get_Total_Angle());
    Winch_Motor.Set_Control_Angle(Winch_Motor.Get_Total_Angle());
    Yaw_Motor.Set_Control_Angle(Yaw_Motor.Get_Now_Angle());

    Safety_Requested = true;
    Outputs_Enabled = false;
    Startup_State = Crane_Startup_State_EXTENSION_START;
    Startup_Deadline_ms = Now_ms;
    Last_Pitch_Command_ms = Now_ms;
    Apply_Pitch_Output(Crane_Pitch_Direction_STOP);
}

void Class_Crane_Control::Disable_Outputs(bool Transmit_Stop)
{
    Safety_Requested = false;
    Outputs_Enabled = false;
    Startup_State = Crane_Startup_State_DISABLED;

    Extension_Motor.Set_Control_Enable(false);
    Winch_Motor.Set_Control_Enable(false);
    Apply_Pitch_Output(Crane_Pitch_Direction_STOP);

    if (Transmit_Stop)
    {
        Extension_Motor.CAN_Send_Stop(false);
        Winch_Motor.CAN_Send_Stop(false);
        Yaw_Motor.CAN_Send_Exit();
    }
}

void Class_Crane_Control::Handle_Startup(uint32_t Now_ms)
{
    if (!Safety_Requested)
    {
        return;
    }

    switch (Startup_State)
    {
    case Crane_Startup_State_EXTENSION_START:
        Extension_Motor.CAN_Send_Start_Control();
        Startup_State = Crane_Startup_State_EXTENSION_REPORT;
        Startup_Deadline_ms = Now_ms + STARTUP_STEP_DELAY_MS;
        break;

    case Crane_Startup_State_EXTENSION_REPORT:
        if (Time_Reached(Now_ms, Startup_Deadline_ms))
        {
            Extension_Motor.CAN_Send_Set_Active_Report_Period_MS(50);
            Extension_Motor.CAN_Send_Set_Active_Report(true);
            Startup_State = Crane_Startup_State_WINCH_START;
            Startup_Deadline_ms = Now_ms + STARTUP_STEP_DELAY_MS;
        }
        break;

    case Crane_Startup_State_WINCH_START:
        if (Time_Reached(Now_ms, Startup_Deadline_ms))
        {
            Winch_Motor.CAN_Send_Start_Control();
            Startup_State = Crane_Startup_State_WINCH_REPORT;
            Startup_Deadline_ms = Now_ms + STARTUP_STEP_DELAY_MS;
        }
        break;

    case Crane_Startup_State_WINCH_REPORT:
        if (Time_Reached(Now_ms, Startup_Deadline_ms))
        {
            Winch_Motor.CAN_Send_Set_Active_Report_Period_MS(50);
            Winch_Motor.CAN_Send_Set_Active_Report(true);
            Startup_State = Crane_Startup_State_YAW_START;
            Startup_Deadline_ms = Now_ms + STARTUP_STEP_DELAY_MS;
        }
        break;

    case Crane_Startup_State_YAW_START:
        if (Time_Reached(Now_ms, Startup_Deadline_ms))
        {
            Yaw_Motor.CAN_Send_Enter();
            Outputs_Enabled = true;
            Startup_State = Crane_Startup_State_ACTIVE;
        }
        break;

    case Crane_Startup_State_ACTIVE:
    case Crane_Startup_State_DISABLED:
    default:
        break;
    }
}

void Class_Crane_Control::Apply_Pitch_Output(Enum_Crane_Pitch_Direction Direction)
{
    if (!Outputs_Enabled && (Direction != Crane_Pitch_Direction_STOP))
    {
        Direction = Crane_Pitch_Direction_STOP;
    }

    // 换向前先保证两路都为低电平，杜绝桥臂方向信号同时为高。
    HAL_GPIO_WritePin(GPIOE, PITCH_MOTOR_A_Pin | PITCH_MOTOR_B_Pin, GPIO_PIN_RESET);
    if (Direction == Crane_Pitch_Direction_UP)
    {
        HAL_GPIO_WritePin(GPIOE, PITCH_MOTOR_A_Pin, GPIO_PIN_SET);
    }
    else if (Direction == Crane_Pitch_Direction_DOWN)
    {
        HAL_GPIO_WritePin(GPIOE, PITCH_MOTOR_B_Pin, GPIO_PIN_SET);
    }

    Pitch_Direction = Direction;
}

void Class_Crane_Control::Send_Telemetry(uint32_t Now_ms)
{
    // USART1 只由吊机模块使用。若先前的 TX DMA 完成链路异常导致 gState 卡在
    // BUSY_TX，先仅中止发送方向；RX DMA 和上位机心跳接收不受影响。
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        HAL_UART_AbortTransmit(&huart1);
        ++UART_Tx_Error_Count;
    }

    static char Tx_Buffer[320];
    const uint32_t Command_Age_ms = Now_ms - Last_Valid_Command_ms;
    const int Length = std::snprintf(
        Tx_Buffer,
        sizeof(Tx_Buffer),
        "TEL,%u,%u,%ld,%u,%ld,%u,%ld,%u,%d,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
        Safety_Requested ? 1u : 0u,
        Outputs_Enabled ? 1u : 0u,
        static_cast<long>(Radian_To_Centidegree(Extension_Motor.Get_Total_Angle())),
        Feedback_Is_Online(Extension_Feedback_Received, Extension_Last_Feedback_ms, Now_ms) ? 1u : 0u,
        static_cast<long>(Radian_To_Centidegree(Winch_Motor.Get_Total_Angle())),
        Feedback_Is_Online(Winch_Feedback_Received, Winch_Last_Feedback_ms, Now_ms) ? 1u : 0u,
        static_cast<long>(Radian_To_Centidegree(Yaw_Motor.Get_Now_Angle())),
        Feedback_Is_Online(Yaw_Feedback_Received, Yaw_Last_Feedback_ms, Now_ms) ? 1u : 0u,
        static_cast<int>(Pitch_Direction),
        static_cast<unsigned long>(Command_Age_ms),
        static_cast<unsigned long>(UART_Ring_Overflow_Count),
        static_cast<unsigned long>(UART_Tx_Error_Count),
        static_cast<long>(Extension_Motor.Get_Control_Speed_Limit() * 100.0f + 0.5f),
        static_cast<long>(Winch_Motor.Get_Control_Speed_Limit() * 100.0f + 0.5f),
        static_cast<long>(Yaw_Motor.Get_Control_Omega() * 100.0f + 0.5f),
        static_cast<long>(Extension_Min_Cdeg),
        static_cast<long>(Extension_Max_Cdeg),
        static_cast<long>(Yaw_Min_Cdeg),
        static_cast<long>(Yaw_Max_Cdeg));

    if ((Length > 0) && (Length < static_cast<int>(sizeof(Tx_Buffer))))
    {
        // 典型遥测帧在 115200 baud 下约 5~8 ms。使用轮询发送消除 TX DMA
        // 完成中断未复位 gState 后永久停发的问题；1 ms 电机控制中断仍可抢占。
        const HAL_StatusTypeDef Status = HAL_UART_Transmit(
            &huart1,
            reinterpret_cast<uint8_t *>(Tx_Buffer),
            static_cast<uint16_t>(Length),
            20u);
        if (Status != HAL_OK)
        {
            ++UART_Tx_Error_Count;
            HAL_UART_AbortTransmit(&huart1);
        }
    }
    else
    {
        ++UART_Tx_Error_Count;
    }
}

bool Class_Crane_Control::Time_Reached(uint32_t Now_ms, uint32_t Deadline_ms)
{
    return (static_cast<int32_t>(Now_ms - Deadline_ms) >= 0);
}

bool Class_Crane_Control::Feedback_Is_Online(bool Received, uint32_t Last_ms, uint32_t Now_ms)
{
    return Received && ((Now_ms - Last_ms) <= MOTOR_ONLINE_TIMEOUT_MS);
}

int32_t Class_Crane_Control::Clamp_Centidegree(int32_t Value, int32_t Min, int32_t Max)
{
    if (Value < Min)
    {
        return Min;
    }
    if (Value > Max)
    {
        return Max;
    }
    return Value;
}

float Class_Crane_Control::Centidegree_To_Radian(int32_t Centidegree)
{
    return static_cast<float>(Centidegree) * CRANE_PI / 18000.0f;
}

int32_t Class_Crane_Control::Radian_To_Centidegree(float Radian)
{
    const float Value = Radian * 18000.0f / CRANE_PI;
    if (Value >= static_cast<float>(INT32_MAX))
    {
        return INT32_MAX;
    }
    if (Value <= static_cast<float>(INT32_MIN))
    {
        return INT32_MIN;
    }
    return static_cast<int32_t>(Value + ((Value >= 0.0f) ? 0.5f : -0.5f));
}
