/**
 * @file dvc_motor_IK.cpp
 * @author Project integration
 * @brief 上海瓴控/KMTECH 电机 CAN 通信驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《电机 CAN 总线通讯协议 V2.36》新增
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_IK.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace
{
constexpr float MOTOR_IK_PI = 3.14159265358979323846f;
constexpr float MOTOR_IK_DEG_TO_RAD = MOTOR_IK_PI / 180.0f;
constexpr float MOTOR_IK_RAD_TO_DEG = 180.0f / MOTOR_IK_PI;
constexpr float MOTOR_IK_CELSIUS_TO_KELVIN = 273.15f;

uint8_t Clamp_U8(const int32_t Value, const uint8_t Min, const uint8_t Max)
{
    if (Value < static_cast<int32_t>(Min))
    {
        return (Min);
    }
    if (Value > static_cast<int32_t>(Max))
    {
        return (Max);
    }
    return (static_cast<uint8_t>(Value));
}

int16_t Clamp_I16(const int32_t Value, const int16_t Min, const int16_t Max)
{
    if (Value < static_cast<int32_t>(Min))
    {
        return (Min);
    }
    if (Value > static_cast<int32_t>(Max))
    {
        return (Max);
    }
    return (static_cast<int16_t>(Value));
}

int16_t Round_Clamp_I16(const double Value, const int16_t Min, const int16_t Max)
{
    if (Value != Value)
    {
        return (0);
    }
    if (Value <= static_cast<double>(Min))
    {
        return (Min);
    }
    if (Value >= static_cast<double>(Max))
    {
        return (Max);
    }

    return (static_cast<int16_t>((Value >= 0.0) ? (Value + 0.5) : (Value - 0.5)));
}

int32_t Round_Clamp_I32(const double Value)
{
    if (Value != Value)
    {
        return (0);
    }
    if (Value <= static_cast<double>(INT32_MIN))
    {
        return (INT32_MIN);
    }
    if (Value >= static_cast<double>(INT32_MAX))
    {
        return (INT32_MAX);
    }

    return (static_cast<int32_t>((Value >= 0.0) ? (Value + 0.5) : (Value - 0.5)));
}

uint16_t Round_Clamp_U16(const double Value)
{
    if ((Value != Value) || (Value <= 0.0))
    {
        return (0u);
    }
    if (Value >= 65535.0)
    {
        return (65535u);
    }

    return (static_cast<uint16_t>(Value + 0.5));
}

uint32_t Round_Clamp_U32(const double Value)
{
    if ((Value != Value) || (Value <= 0.0))
    {
        return (0u);
    }
    if (Value >= static_cast<double>(UINT32_MAX))
    {
        return (UINT32_MAX);
    }

    return (static_cast<uint32_t>(Value + 0.5));
}

uint16_t Read_U16_LE(const uint8_t *Data)
{
    return (static_cast<uint16_t>(Data[0]) |
            (static_cast<uint16_t>(Data[1]) << 8));
}

int16_t Read_I16_LE(const uint8_t *Data)
{
    return (static_cast<int16_t>(Read_U16_LE(Data)));
}

uint32_t Read_U32_LE(const uint8_t *Data)
{
    return (static_cast<uint32_t>(Data[0]) |
            (static_cast<uint32_t>(Data[1]) << 8) |
            (static_cast<uint32_t>(Data[2]) << 16) |
            (static_cast<uint32_t>(Data[3]) << 24));
}

int32_t Read_I32_LE(const uint8_t *Data)
{
    return (static_cast<int32_t>(Read_U32_LE(Data)));
}

int64_t Read_I56_LE(const uint8_t *Data)
{
    uint64_t Raw = 0u;
    for (uint8_t i = 0; i < 7u; i++)
    {
        Raw |= (static_cast<uint64_t>(Data[i]) << (8u * i));
    }

    // DATA[1:7] 只有 56 位；bit55 为符号位，需要扩展到 int64_t。
    if ((Raw & (static_cast<uint64_t>(1u) << 55u)) != 0u)
    {
        Raw |= 0xff00000000000000ULL;
    }
    return (static_cast<int64_t>(Raw));
}

void Write_U16_LE(uint8_t *Data, const uint16_t Value)
{
    Data[0] = static_cast<uint8_t>(Value & 0xffu);
    Data[1] = static_cast<uint8_t>((Value >> 8) & 0xffu);
}

void Write_I16_LE(uint8_t *Data, const int16_t Value)
{
    Write_U16_LE(Data, static_cast<uint16_t>(Value));
}

void Write_U32_LE(uint8_t *Data, const uint32_t Value)
{
    Data[0] = static_cast<uint8_t>(Value & 0xffu);
    Data[1] = static_cast<uint8_t>((Value >> 8) & 0xffu);
    Data[2] = static_cast<uint8_t>((Value >> 16) & 0xffu);
    Data[3] = static_cast<uint8_t>((Value >> 24) & 0xffu);
}

void Write_I32_LE(uint8_t *Data, const int32_t Value)
{
    Write_U32_LE(Data, static_cast<uint32_t>(Value));
}

int32_t Radian_To_0p01_Degree_I32(const float Radian)
{
    const double Raw = static_cast<double>(Radian) *
                       static_cast<double>(MOTOR_IK_RAD_TO_DEG) * 100.0;
    return (Round_Clamp_I32(Raw));
}

uint32_t Radian_To_0p01_Degree_U32(const float Radian)
{
    const double Raw = static_cast<double>(Radian) *
                       static_cast<double>(MOTOR_IK_RAD_TO_DEG) * 100.0;
    return (Round_Clamp_U32(Raw));
}

int32_t Radps_To_0p01_DPS_I32(const float Omega)
{
    const double Raw = static_cast<double>(Omega) *
                       static_cast<double>(MOTOR_IK_RAD_TO_DEG) * 100.0;
    return (Round_Clamp_I32(Raw));
}

uint16_t Radps_To_DPS_U16(const float Omega)
{
    const double Positive_Omega = (Omega < 0.0f) ? -static_cast<double>(Omega) : static_cast<double>(Omega);
    const double Raw = Positive_Omega * static_cast<double>(MOTOR_IK_RAD_TO_DEG);
    return (Round_Clamp_U16(Raw));
}

float Degree_To_Radian(const float Degree)
{
    return (Degree * MOTOR_IK_DEG_TO_RAD);
}
} // namespace

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化电机对象
 */
void Class_Motor_IK::Init(const FDCAN_HandleTypeDef *hcan,
                          const uint8_t &__Motor_ID,
                          const Enum_Motor_IK_Series &__Motor_Series,
                          const uint8_t &__Encoder_Bits,
                          const float &__Current_Resolution)
{
    CAN_Manage_Object = nullptr;
    CAN_Handler = nullptr;

    if (hcan != nullptr)
    {
        CAN_Handler = const_cast<FDCAN_HandleTypeDef *>(hcan);

        if (hcan->Instance == FDCAN1)
        {
            CAN_Manage_Object = &CAN1_Manage_Object;
        }
        else if (hcan->Instance == FDCAN2)
        {
            CAN_Manage_Object = &CAN2_Manage_Object;
        }
        else if (hcan->Instance == FDCAN3)
        {
            CAN_Manage_Object = &CAN3_Manage_Object;
        }

        if ((CAN_Manage_Object != nullptr) && (CAN_Manage_Object->CAN_Handler == nullptr))
        {
            CAN_Manage_Object->CAN_Handler = CAN_Handler;
        }
    }

    Motor_ID = Clamp_U8(__Motor_ID, 1u, 32u);
    CAN_ID = static_cast<uint16_t>(0x140u + Motor_ID);
    Motor_Series = __Motor_Series;

    if ((__Encoder_Bits == 14u) || (__Encoder_Bits == 15u) || (__Encoder_Bits == 16u))
    {
        Encoder_Bits = __Encoder_Bits;
    }
    else
    {
        Encoder_Bits = 14u;
    }
    Encoder_Counts = static_cast<uint32_t>(1UL << Encoder_Bits);

    if (__Current_Resolution > 0.0f)
    {
        Current_Resolution = __Current_Resolution;
    }
    else
    {
        switch (Motor_Series)
        {
        case (Motor_IK_Series_MG):
        {
            Current_Resolution = 66.0f / 4096.0f;
            break;
        }
        case (Motor_IK_Series_MS):
        {
            Current_Resolution = 0.0f;
            break;
        }
        case (Motor_IK_Series_MF):
        case (Motor_IK_Series_MH):
        case (Motor_IK_Series_MHF):
        default:
        {
            Current_Resolution = 33.0f / 4096.0f;
            break;
        }
        }
    }

    Flag = 0u;
    Pre_Flag = 0u;
    Motor_IK_Status = Motor_IK_Status_DISABLE;

    Control_Method = Motor_IK_Control_Method_NONE;
    Control_Enable = false;
    Control_Power = 0;
    Control_Current = 0.0f;
    Control_Omega = 0.0f;
    Control_Current_Limit = 0.0f;
    Control_Angle = 0.0f;
    Control_Angle_Increment = 0.0f;
    Control_Max_Omega = 0.0f;
    Control_Spin_Direction = Motor_IK_Spin_Direction_CLOCKWISE;
    Increment_Command_Pending = false;

    Reset_Data();
}

/**
 * @brief 清空反馈缓存，并设置有意义的未知默认值
 */
void Class_Motor_IK::Reset_Data()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(&Control_Parameter_Data, 0, sizeof(Control_Parameter_Data));
    memset(&Setting_Parameter_Data, 0, sizeof(Setting_Parameter_Data));

    Rx_Data.Temperature = MOTOR_IK_CELSIUS_TO_KELVIN;
    Rx_Data.Motor_State = Motor_IK_Motor_State_UNKNOWN;
    Rx_Data.Brake_Status = Motor_IK_Brake_Status_UNKNOWN;
}

/**
 * @brief 从当前 CAN 管理对象处理接收帧
 */
void Class_Motor_IK::CAN_RxCpltCallback()
{
    if (CAN_Manage_Object == nullptr)
    {
        return;
    }

    CAN_RxCpltCallback(CAN_Manage_Object->Rx_Header, CAN_Manage_Object->Rx_Buffer);
}

/**
 * @brief 处理指定 Header/Buffer
 */
void Class_Motor_IK::CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header, const uint8_t *Buffer)
{
    if ((Buffer == nullptr) || (Header.Identifier != CAN_ID))
    {
        return;
    }

    Flag++;
    Data_Process(Buffer);
}

/**
 * @brief 发送 8 字节标准 CAN 数据帧
 */
void Class_Motor_IK::Send_Frame(uint8_t *Data) const
{
    if ((CAN_Handler == nullptr) || (Data == nullptr))
    {
        return;
    }

    CAN_Transmit_Data(CAN_Handler, CAN_ID, Data, 8u);
}

/**
 * @brief 发送只有命令字的空参数命令
 */
void Class_Motor_IK::Send_Empty_Command(const uint8_t &Command) const
{
    uint8_t Data[8] = {0};
    Data[0] = Command;
    Send_Frame(Data);
}

/**
 * @brief 读取状态 1 和错误标志
 */
void Class_Motor_IK::CAN_Send_Read_Status_1() const
{
    Send_Empty_Command(Motor_IK_Command_READ_STATUS_1);
}

/**
 * @brief 清除错误标志
 */
void Class_Motor_IK::CAN_Send_Clear_Error() const
{
    Send_Empty_Command(Motor_IK_Command_CLEAR_ERROR);
}

/**
 * @brief 读取状态 2
 */
void Class_Motor_IK::CAN_Send_Read_Status_2() const
{
    Send_Empty_Command(Motor_IK_Command_READ_STATUS_2);
}

/**
 * @brief 读取状态 3
 */
void Class_Motor_IK::CAN_Send_Read_Status_3() const
{
    Send_Empty_Command(Motor_IK_Command_READ_STATUS_3);
}

/**
 * @brief 关闭电机，清除圈数和此前控制指令
 */
void Class_Motor_IK::CAN_Send_Motor_Shutdown() const
{
    Send_Empty_Command(Motor_IK_Command_SHUTDOWN);
}

/**
 * @brief 运行/开启电机
 */
void Class_Motor_IK::CAN_Send_Motor_Run() const
{
    Send_Empty_Command(Motor_IK_Command_RUN);
}

/**
 * @brief 停止电机，但不清除运行状态
 */
void Class_Motor_IK::CAN_Send_Motor_Stop() const
{
    Send_Empty_Command(Motor_IK_Command_STOP);
}

/**
 * @brief 控制或读取抱闸器
 */
void Class_Motor_IK::CAN_Send_Brake(const Enum_Motor_IK_Brake_Command &__Brake_Command) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_BRAKE;
    Data[1] = static_cast<uint8_t>(__Brake_Command);
    Send_Frame(Data);
}

/**
 * @brief 读取抱闸器状态
 */
void Class_Motor_IK::CAN_Send_Read_Brake() const
{
    CAN_Send_Brake(Motor_IK_Brake_Command_READ);
}

/**
 * @brief MS 系列开环控制，范围 -850~850
 */
void Class_Motor_IK::CAN_Send_Open_Loop_Control(const int16_t &__Power_Control) const
{
    uint8_t Data[8] = {0};
    const int16_t Power = Clamp_I16(__Power_Control, -850, 850);

    Data[0] = Motor_IK_Command_OPEN_LOOP;
    Write_I16_LE(&Data[4], Power);
    Send_Frame(Data);
}

/**
 * @brief 物理电流 A 转协议 IQ 原始值
 */
int16_t Class_Motor_IK::Current_To_Raw(const float &Current) const
{
    if (Current_Resolution <= 0.0f)
    {
        return (0);
    }

    const double Raw = static_cast<double>(Current) / static_cast<double>(Current_Resolution);
    return (Round_Clamp_I16(Raw, -2048, 2048));
}

/**
 * @brief 协议 IQ 原始值转物理电流 A
 */
float Class_Motor_IK::Raw_To_Current(const int16_t &Raw) const
{
    return (static_cast<float>(Raw) * Current_Resolution);
}

/**
 * @brief MF/MH/MHF/MG 系列转矩电流控制，单位 A
 */
void Class_Motor_IK::CAN_Send_Torque_Control(const float &__Current) const
{
    CAN_Send_Torque_Control_Raw(Current_To_Raw(__Current));
}

/**
 * @brief 转矩电流控制原始值，范围 -2048~2048
 */
void Class_Motor_IK::CAN_Send_Torque_Control_Raw(const int16_t &__IQ_Control) const
{
    uint8_t Data[8] = {0};
    const int16_t IQ = Clamp_I16(__IQ_Control, -2048, 2048);

    Data[0] = Motor_IK_Command_TORQUE;
    Write_I16_LE(&Data[4], IQ);
    Send_Frame(Data);
}

/**
 * @brief 速度闭环控制；速度 rad/s，电流限制 A
 */
void Class_Motor_IK::CAN_Send_Speed_Control(const float &__Omega, const float &__Current_Limit) const
{
    float Current_Limit = __Current_Limit;
    if (Current_Limit < 0.0f)
    {
        Current_Limit = -Current_Limit;
    }

    CAN_Send_Speed_Control_Raw(Radps_To_0p01_DPS_I32(__Omega), Current_To_Raw(Current_Limit));
}

/**
 * @brief 速度闭环控制原始值；速度单位 0.01dps，IQ 限制范围 -2048~2048
 */
void Class_Motor_IK::CAN_Send_Speed_Control_Raw(const int32_t &__Speed_Control, const int16_t &__IQ_Limit) const
{
    uint8_t Data[8] = {0};
    const int16_t IQ_Limit = Clamp_I16(__IQ_Limit, -2048, 2048);

    Data[0] = Motor_IK_Command_SPEED;
    Write_I16_LE(&Data[2], IQ_Limit);
    Write_I32_LE(&Data[4], __Speed_Control);
    Send_Frame(Data);
}

/**
 * @brief 多圈位置闭环控制 1，角度 rad
 */
void Class_Motor_IK::CAN_Send_Multi_Turn_Angle_Control_1(const float &__Angle) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_MULTI_TURN_ANGLE_1;
    Write_I32_LE(&Data[4], Radian_To_0p01_Degree_I32(__Angle));
    Send_Frame(Data);
}

/**
 * @brief 多圈位置闭环控制 2，角度 rad，最大速度 rad/s
 */
void Class_Motor_IK::CAN_Send_Multi_Turn_Angle_Control_2(const float &__Angle, const float &__Max_Omega) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_MULTI_TURN_ANGLE_2;
    Write_U16_LE(&Data[2], Radps_To_DPS_U16(__Max_Omega));
    Write_I32_LE(&Data[4], Radian_To_0p01_Degree_I32(__Angle));
    Send_Frame(Data);
}

/**
 * @brief 单圈位置闭环控制 1
 */
void Class_Motor_IK::CAN_Send_Single_Turn_Angle_Control_1(const float &__Angle, const Enum_Motor_IK_Spin_Direction &__Direction) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_SINGLE_TURN_ANGLE_1;
    Data[1] = static_cast<uint8_t>(__Direction);
    Write_U32_LE(&Data[4], Radian_To_0p01_Degree_U32(__Angle));
    Send_Frame(Data);
}

/**
 * @brief 单圈位置闭环控制 2
 */
void Class_Motor_IK::CAN_Send_Single_Turn_Angle_Control_2(const float &__Angle, const float &__Max_Omega, const Enum_Motor_IK_Spin_Direction &__Direction) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_SINGLE_TURN_ANGLE_2;
    Data[1] = static_cast<uint8_t>(__Direction);
    Write_U16_LE(&Data[2], Radps_To_DPS_U16(__Max_Omega));
    Write_U32_LE(&Data[4], Radian_To_0p01_Degree_U32(__Angle));
    Send_Frame(Data);
}

/**
 * @brief 增量位置闭环控制 1
 */
void Class_Motor_IK::CAN_Send_Increment_Angle_Control_1(const float &__Angle_Increment) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_INCREMENT_ANGLE_1;
    Write_I32_LE(&Data[4], Radian_To_0p01_Degree_I32(__Angle_Increment));
    Send_Frame(Data);
}

/**
 * @brief 增量位置闭环控制 2
 * @note 协议正文将 maxSpeed 写作 uint32_t，但报文字段只有 DATA[2:3] 两字节，本驱动按 uint16_t 实现。
 */
void Class_Motor_IK::CAN_Send_Increment_Angle_Control_2(const float &__Angle_Increment, const float &__Max_Omega) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_INCREMENT_ANGLE_2;
    Write_U16_LE(&Data[2], Radps_To_DPS_U16(__Max_Omega));
    Write_I32_LE(&Data[4], Radian_To_0p01_Degree_I32(__Angle_Increment));
    Send_Frame(Data);
}

/**
 * @brief 读取 RAM 控制参数
 */
void Class_Motor_IK::CAN_Send_Read_Control_Parameter(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_READ_CONTROL_PARAMETER;
    Data[1] = static_cast<uint8_t>(__Parameter_ID);
    Send_Frame(Data);
}

/**
 * @brief 写入 RAM 控制参数原始 6 字节
 */
void Class_Motor_IK::CAN_Send_Write_Control_Parameter_Raw(const uint8_t &__Parameter_ID, const uint8_t __Parameter_Data[6]) const
{
    if (__Parameter_Data == nullptr)
    {
        return;
    }

    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_WRITE_CONTROL_PARAMETER;
    Data[1] = __Parameter_ID;
    memcpy(&Data[2], __Parameter_Data, 6u);
    Send_Frame(Data);
}

/**
 * @brief 写入位置/速度/电流环 PID 到 RAM
 */
void Class_Motor_IK::CAN_Send_Write_Control_PID(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID,
                                                const uint16_t &__K_P,
                                                const uint16_t &__K_I,
                                                const uint16_t &__K_D) const
{
    uint8_t Parameter[6] = {0};
    Write_U16_LE(&Parameter[0], __K_P);
    Write_U16_LE(&Parameter[2], __K_I);
    Write_U16_LE(&Parameter[4], __K_D);
    CAN_Send_Write_Control_Parameter_Raw(static_cast<uint8_t>(__Parameter_ID), Parameter);
}

/**
 * @brief 写入 int16 类型 RAM 控制参数；值放在 DATA[4:5]
 */
void Class_Motor_IK::CAN_Send_Write_Control_Parameter_I16(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID, const int16_t &__Value) const
{
    uint8_t Parameter[6] = {0};
    Write_I16_LE(&Parameter[2], __Value);
    CAN_Send_Write_Control_Parameter_Raw(static_cast<uint8_t>(__Parameter_ID), Parameter);
}

/**
 * @brief 写入 int32 类型 RAM 控制参数；值放在 DATA[4:7]
 */
void Class_Motor_IK::CAN_Send_Write_Control_Parameter_I32(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID, const int32_t &__Value) const
{
    uint8_t Parameter[6] = {0};
    Write_I32_LE(&Parameter[2], __Value);
    CAN_Send_Write_Control_Parameter_Raw(static_cast<uint8_t>(__Parameter_ID), Parameter);
}

/**
 * @brief 读取编码器数据
 */
void Class_Motor_IK::CAN_Send_Read_Encoder() const
{
    Send_Empty_Command(Motor_IK_Command_READ_ENCODER);
}

/**
 * @brief 校准编码器；会写 ROM，不应频繁调用
 */
void Class_Motor_IK::CAN_Send_Calibrate_Encoder() const
{
    Send_Empty_Command(Motor_IK_Command_CALIBRATE_ENCODER);
}

/**
 * @brief 设置当前位置为永久零点；会写 ROM，重新上电后生效
 */
void Class_Motor_IK::CAN_Send_Set_Zero_ROM() const
{
    Send_Empty_Command(Motor_IK_Command_SET_ZERO_ROM);
}

/**
 * @brief 读取多圈角度
 */
void Class_Motor_IK::CAN_Send_Read_Multi_Turn_Angle() const
{
    Send_Empty_Command(Motor_IK_Command_READ_MULTI_TURN_ANGLE);
}

/**
 * @brief 读取单圈角度
 */
void Class_Motor_IK::CAN_Send_Read_Single_Turn_Angle() const
{
    Send_Empty_Command(Motor_IK_Command_READ_SINGLE_TURN_ANGLE);
}

/**
 * @brief 设置当前位置为 RAM 零点；重新上电后失效
 */
void Class_Motor_IK::CAN_Send_Set_Zero_RAM() const
{
    Send_Empty_Command(Motor_IK_Command_SET_ZERO_RAM);
}

/**
 * @brief 读取设定参数原始索引
 */
void Class_Motor_IK::CAN_Send_Read_Setting_Parameter_Raw(const uint8_t &__Parameter_1, const uint8_t &__Parameter_2) const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_READ_SETTING_PARAMETER;
    Data[1] = __Parameter_1;
    Data[2] = __Parameter_2;
    Send_Frame(Data);
}

/**
 * @brief 写入设定参数原始 DATA[1:7]
 */
void Class_Motor_IK::CAN_Send_Write_Setting_Parameter_Raw(const uint8_t __Parameter_Data[7]) const
{
    if (__Parameter_Data == nullptr)
    {
        return;
    }

    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_WRITE_SETTING_PARAMETER;
    memcpy(&Data[1], __Parameter_Data, 7u);
    Send_Frame(Data);
}

/**
 * @brief 读取单个设定参数
 */
void Class_Motor_IK::CAN_Send_Read_Single_Setting(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID) const
{
    CAN_Send_Read_Setting_Parameter_Raw(0x05u, static_cast<uint8_t>(__Parameter_ID));
}

/**
 * @brief 写入 uint8 类型单个设定参数
 */
void Class_Motor_IK::CAN_Send_Write_Single_Setting_U8(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const uint8_t &__Value) const
{
    uint8_t Parameter[7] = {0x05u, static_cast<uint8_t>(__Parameter_ID), 0x00u, __Value, 0x00u, 0x00u, 0x00u};
    CAN_Send_Write_Setting_Parameter_Raw(Parameter);
}

/**
 * @brief 写入 int16 类型单个设定参数
 */
void Class_Motor_IK::CAN_Send_Write_Single_Setting_I16(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const int16_t &__Value) const
{
    uint8_t Parameter[7] = {0};
    Parameter[0] = 0x05u;
    Parameter[1] = static_cast<uint8_t>(__Parameter_ID);
    Write_I16_LE(&Parameter[3], __Value);
    CAN_Send_Write_Setting_Parameter_Raw(Parameter);
}

/**
 * @brief 写入 int32 类型单个设定参数
 */
void Class_Motor_IK::CAN_Send_Write_Single_Setting_I32(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const int32_t &__Value) const
{
    uint8_t Parameter[7] = {0};
    Parameter[0] = 0x05u;
    Parameter[1] = static_cast<uint8_t>(__Parameter_ID);
    Write_I32_LE(&Parameter[3], __Value);
    CAN_Send_Write_Setting_Parameter_Raw(Parameter);
}

/**
 * @brief 读取多参数 PID 设定
 */
void Class_Motor_IK::CAN_Send_Read_Setting_PID(const Enum_Motor_IK_Setting_PID_ID &__PID_ID) const
{
    CAN_Send_Read_Setting_Parameter_Raw(static_cast<uint8_t>(__PID_ID), 0x00u);
}

/**
 * @brief 写入多参数 PID 设定；写入后还需发送保存设定参数命令
 */
void Class_Motor_IK::CAN_Send_Write_Setting_PID(const Enum_Motor_IK_Setting_PID_ID &__PID_ID,
                                                const uint16_t &__K_P,
                                                const uint16_t &__K_I,
                                                const uint16_t &__K_D) const
{
    uint8_t Parameter[7] = {0};
    Parameter[0] = static_cast<uint8_t>(__PID_ID);
    Write_U16_LE(&Parameter[1], __K_P);
    Write_U16_LE(&Parameter[3], __K_I);
    Write_U16_LE(&Parameter[5], __K_D);
    CAN_Send_Write_Setting_Parameter_Raw(Parameter);
}

/**
 * @brief 保存设定参数到 ROM
 */
void Class_Motor_IK::CAN_Send_Save_Setting_Parameters() const
{
    uint8_t Data[8] = {0};
    Data[0] = Motor_IK_Command_SAVE_SETTING_PARAMETER;
    Data[1] = 0x05u;
    Data[2] = 0xfau;
    Send_Frame(Data);
}

/**
 * @brief 重启电机；该命令无回复
 */
void Class_Motor_IK::CAN_Send_Restart() const
{
    Send_Empty_Command(Motor_IK_Command_RESTART);
}

/**
 * @brief 周期在线检测
 */
void Class_Motor_IK::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_IK_Status = Motor_IK_Status_DISABLE;
    }
    else
    {
        Motor_IK_Status = Motor_IK_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief 周期发送控制命令
 */
void Class_Motor_IK::TIM_Send_PeriodElapsedCallback()
{
    if (!Control_Enable)
    {
        return;
    }

    if (((Control_Method == Motor_IK_Control_Method_INCREMENT_ANGLE_1) ||
         (Control_Method == Motor_IK_Control_Method_INCREMENT_ANGLE_2)) &&
        (!Increment_Command_Pending))
    {
        return;
    }

    Output();

    if ((Control_Method == Motor_IK_Control_Method_INCREMENT_ANGLE_1) ||
        (Control_Method == Motor_IK_Control_Method_INCREMENT_ANGLE_2))
    {
        Increment_Command_Pending = false;
    }
}

/**
 * @brief 按选定控制模式输出
 */
void Class_Motor_IK::Output()
{
    switch (Control_Method)
    {
    case (Motor_IK_Control_Method_OPEN_LOOP):
    {
        CAN_Send_Open_Loop_Control(Control_Power);
        break;
    }
    case (Motor_IK_Control_Method_TORQUE):
    {
        CAN_Send_Torque_Control(Control_Current);
        break;
    }
    case (Motor_IK_Control_Method_SPEED):
    {
        CAN_Send_Speed_Control(Control_Omega, Control_Current_Limit);
        break;
    }
    case (Motor_IK_Control_Method_MULTI_TURN_ANGLE_1):
    {
        CAN_Send_Multi_Turn_Angle_Control_1(Control_Angle);
        break;
    }
    case (Motor_IK_Control_Method_MULTI_TURN_ANGLE_2):
    {
        CAN_Send_Multi_Turn_Angle_Control_2(Control_Angle, Control_Max_Omega);
        break;
    }
    case (Motor_IK_Control_Method_SINGLE_TURN_ANGLE_1):
    {
        CAN_Send_Single_Turn_Angle_Control_1(Control_Angle, Control_Spin_Direction);
        break;
    }
    case (Motor_IK_Control_Method_SINGLE_TURN_ANGLE_2):
    {
        CAN_Send_Single_Turn_Angle_Control_2(Control_Angle, Control_Max_Omega, Control_Spin_Direction);
        break;
    }
    case (Motor_IK_Control_Method_INCREMENT_ANGLE_1):
    {
        CAN_Send_Increment_Angle_Control_1(Control_Angle_Increment);
        break;
    }
    case (Motor_IK_Control_Method_INCREMENT_ANGLE_2):
    {
        CAN_Send_Increment_Angle_Control_2(Control_Angle_Increment, Control_Max_Omega);
        break;
    }
    case (Motor_IK_Control_Method_NONE):
    default:
    {
        break;
    }
    }
}

/**
 * @brief 解析接收帧
 */
void Class_Motor_IK::Data_Process(const uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return;
    }

    Rx_Data.Last_Command = Buffer[0];
    memcpy(Rx_Data.Last_Raw_Data, Buffer, 8u);

    switch (Buffer[0])
    {
    case (Motor_IK_Command_READ_STATUS_1):
    case (Motor_IK_Command_CLEAR_ERROR):
    {
        Parse_Status_1(Buffer);
        break;
    }
    case (Motor_IK_Command_READ_STATUS_2):
    case (Motor_IK_Command_OPEN_LOOP):
    case (Motor_IK_Command_TORQUE):
    case (Motor_IK_Command_SPEED):
    case (Motor_IK_Command_MULTI_TURN_ANGLE_1):
    case (Motor_IK_Command_MULTI_TURN_ANGLE_2):
    case (Motor_IK_Command_SINGLE_TURN_ANGLE_1):
    case (Motor_IK_Command_SINGLE_TURN_ANGLE_2):
    case (Motor_IK_Command_INCREMENT_ANGLE_1):
    case (Motor_IK_Command_INCREMENT_ANGLE_2):
    {
        Parse_Status_2(Buffer);
        break;
    }
    case (Motor_IK_Command_READ_STATUS_3):
    {
        Parse_Status_3(Buffer);
        break;
    }
    case (Motor_IK_Command_SHUTDOWN):
    {
        Rx_Data.Motor_State = Motor_IK_Motor_State_DISABLE;
        break;
    }
    case (Motor_IK_Command_RUN):
    {
        Rx_Data.Motor_State = Motor_IK_Motor_State_ENABLE;
        break;
    }
    case (Motor_IK_Command_STOP):
    {
        // STOP 不改变开启/关闭状态。
        break;
    }
    case (Motor_IK_Command_BRAKE):
    {
        if (Buffer[1] == 0x00u)
        {
            Rx_Data.Brake_Status = Motor_IK_Brake_Status_POWER_OFF;
        }
        else if (Buffer[1] == 0x01u)
        {
            Rx_Data.Brake_Status = Motor_IK_Brake_Status_POWER_ON;
        }
        else
        {
            Rx_Data.Brake_Status = Motor_IK_Brake_Status_UNKNOWN;
        }
        break;
    }
    case (Motor_IK_Command_READ_CONTROL_PARAMETER):
    case (Motor_IK_Command_WRITE_CONTROL_PARAMETER):
    {
        Parse_Control_Parameter(Buffer);
        break;
    }
    case (Motor_IK_Command_READ_ENCODER):
    {
        Rx_Data.Encoder = Read_U16_LE(&Buffer[2]);
        Rx_Data.Encoder_Raw = Read_U16_LE(&Buffer[4]);
        Rx_Data.Encoder_Offset = Read_U16_LE(&Buffer[6]);
        Rx_Data.Encoder_Angle = static_cast<float>(Rx_Data.Encoder) /
                                static_cast<float>(Encoder_Counts) * 2.0f * MOTOR_IK_PI;
        break;
    }
    case (Motor_IK_Command_CALIBRATE_ENCODER):
    {
        Rx_Data.Align_Value = Read_U32_LE(&Buffer[1]);
        Rx_Data.Align_Ratio = Read_U16_LE(&Buffer[5]);
        Rx_Data.Align_State = Buffer[7];
        Rx_Data.Align_Success = ((Buffer[7] & 0x01u) != 0u);
        Rx_Data.Align_Phase_Reversed = ((Buffer[7] & 0x10u) != 0u);
        break;
    }
    case (Motor_IK_Command_SET_ZERO_ROM):
    {
        Rx_Data.Encoder_Offset_ROM = Read_U32_LE(&Buffer[4]);
        break;
    }
    case (Motor_IK_Command_READ_MULTI_TURN_ANGLE):
    {
        Rx_Data.Multi_Turn_Angle_Raw = Read_I56_LE(&Buffer[1]);
        Rx_Data.Multi_Turn_Angle = Degree_To_Radian(static_cast<float>(Rx_Data.Multi_Turn_Angle_Raw) * 0.01f);
        break;
    }
    case (Motor_IK_Command_READ_SINGLE_TURN_ANGLE):
    {
        Rx_Data.Single_Turn_Angle_Raw = Read_U32_LE(&Buffer[4]);
        Rx_Data.Single_Turn_Angle = Degree_To_Radian(static_cast<float>(Rx_Data.Single_Turn_Angle_Raw) * 0.01f);
        break;
    }
    case (Motor_IK_Command_READ_SETTING_PARAMETER):
    case (Motor_IK_Command_WRITE_SETTING_PARAMETER):
    {
        Parse_Setting_Parameter(Buffer);
        break;
    }
    case (Motor_IK_Command_SAVE_SETTING_PARAMETER):
    {
        Rx_Data.Save_Setting_Result_Valid = true;
        Rx_Data.Save_Setting_Success = (Buffer[2] == 0x01u);
        break;
    }
    case (Motor_IK_Command_SET_ZERO_RAM):
    case (Motor_IK_Command_RESTART):
    default:
    {
        break;
    }
    }
}

/**
 * @brief 解析状态 1 / 清错回复
 */
void Class_Motor_IK::Parse_Status_1(const uint8_t *Buffer)
{
    Rx_Data.Temperature_Raw = static_cast<int8_t>(Buffer[1]);
    Rx_Data.Temperature = static_cast<float>(Rx_Data.Temperature_Raw) + MOTOR_IK_CELSIUS_TO_KELVIN;

    Rx_Data.Bus_Voltage_Raw = Read_I16_LE(&Buffer[2]);
    Rx_Data.Bus_Voltage = static_cast<float>(Rx_Data.Bus_Voltage_Raw) * 0.01f;

    Rx_Data.Bus_Current_Raw = Read_I16_LE(&Buffer[4]);
    Rx_Data.Bus_Current = static_cast<float>(Rx_Data.Bus_Current_Raw) * 0.01f;

    if (Buffer[6] == 0x00u)
    {
        Rx_Data.Motor_State = Motor_IK_Motor_State_ENABLE;
    }
    else if (Buffer[6] == 0x10u)
    {
        Rx_Data.Motor_State = Motor_IK_Motor_State_DISABLE;
    }
    else
    {
        Rx_Data.Motor_State = Motor_IK_Motor_State_UNKNOWN;
    }
    Rx_Data.Error_State = Buffer[7];
}

/**
 * @brief 解析状态 2 以及各控制命令回复
 */
void Class_Motor_IK::Parse_Status_2(const uint8_t *Buffer)
{
    Rx_Data.Temperature_Raw = static_cast<int8_t>(Buffer[1]);
    Rx_Data.Temperature = static_cast<float>(Rx_Data.Temperature_Raw) + MOTOR_IK_CELSIUS_TO_KELVIN;

    Rx_Data.Torque_Current_Raw = Read_I16_LE(&Buffer[2]);
    if (Motor_Series == Motor_IK_Series_MS)
    {
        Rx_Data.Output_Power = Rx_Data.Torque_Current_Raw;
        Rx_Data.Torque_Current = 0.0f;
    }
    else
    {
        Rx_Data.Torque_Current = Raw_To_Current(Rx_Data.Torque_Current_Raw);
        Rx_Data.Output_Power = 0;
    }

    Rx_Data.Speed_Raw = Read_I16_LE(&Buffer[4]);
    Rx_Data.Now_Omega = Degree_To_Radian(static_cast<float>(Rx_Data.Speed_Raw));

    Rx_Data.Encoder = Read_U16_LE(&Buffer[6]);
    Rx_Data.Encoder_Angle = static_cast<float>(Rx_Data.Encoder) /
                            static_cast<float>(Encoder_Counts) * 2.0f * MOTOR_IK_PI;
}

/**
 * @brief 解析状态 3
 */
void Class_Motor_IK::Parse_Status_3(const uint8_t *Buffer)
{
    Rx_Data.Temperature_Raw = static_cast<int8_t>(Buffer[1]);
    Rx_Data.Temperature = static_cast<float>(Rx_Data.Temperature_Raw) + MOTOR_IK_CELSIUS_TO_KELVIN;

    Rx_Data.Phase_Current_Raw[0] = Read_I16_LE(&Buffer[2]);
    Rx_Data.Phase_Current_Raw[1] = Read_I16_LE(&Buffer[4]);
    Rx_Data.Phase_Current_Raw[2] = Read_I16_LE(&Buffer[6]);

    Rx_Data.Phase_Current[0] = Raw_To_Current(Rx_Data.Phase_Current_Raw[0]);
    Rx_Data.Phase_Current[1] = Raw_To_Current(Rx_Data.Phase_Current_Raw[1]);
    Rx_Data.Phase_Current[2] = Raw_To_Current(Rx_Data.Phase_Current_Raw[2]);
}

/**
 * @brief 解析 RAM 控制参数
 */
void Class_Motor_IK::Parse_Control_Parameter(const uint8_t *Buffer)
{
    memset(&Control_Parameter_Data, 0, sizeof(Control_Parameter_Data));
    Control_Parameter_Data.Valid = true;
    Control_Parameter_Data.Command = Buffer[0];
    Control_Parameter_Data.ID = Buffer[1];
    memcpy(Control_Parameter_Data.Raw, &Buffer[2], 6u);

    switch (Buffer[1])
    {
    case (Motor_IK_Control_Parameter_POSITION_PID):
    case (Motor_IK_Control_Parameter_SPEED_PID):
    case (Motor_IK_Control_Parameter_CURRENT_PID):
    {
        Control_Parameter_Data.PID.K_P = Read_U16_LE(&Buffer[2]);
        Control_Parameter_Data.PID.K_I = Read_U16_LE(&Buffer[4]);
        Control_Parameter_Data.PID.K_D = Read_U16_LE(&Buffer[6]);
        break;
    }
    case (Motor_IK_Control_Parameter_TORQUE_LIMIT):
    {
        Control_Parameter_Data.Value_I16 = Read_I16_LE(&Buffer[4]);
        break;
    }
    case (Motor_IK_Control_Parameter_SPEED_LIMIT):
    case (Motor_IK_Control_Parameter_ANGLE_LIMIT):
    case (Motor_IK_Control_Parameter_CURRENT_RAMP):
    case (Motor_IK_Control_Parameter_SPEED_RAMP):
    {
        Control_Parameter_Data.Value_I32 = Read_I32_LE(&Buffer[4]);
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief 解析设定参数
 */
void Class_Motor_IK::Parse_Setting_Parameter(const uint8_t *Buffer)
{
    memset(&Setting_Parameter_Data, 0, sizeof(Setting_Parameter_Data));
    Setting_Parameter_Data.Valid = true;
    Setting_Parameter_Data.Command = Buffer[0];
    Setting_Parameter_Data.Parameter_1 = Buffer[1];
    Setting_Parameter_Data.Parameter_2 = Buffer[2];
    memcpy(Setting_Parameter_Data.Raw, &Buffer[1], 7u);

    if (Buffer[1] == 0x05u)
    {
        switch (Buffer[2])
        {
        case (Motor_IK_Setting_Parameter_DRIVER_ID):
        case (Motor_IK_Setting_Parameter_BUS_TYPE):
        case (Motor_IK_Setting_Parameter_RS485_BAUDRATE):
        case (Motor_IK_Setting_Parameter_CAN_BAUDRATE):
        {
            Setting_Parameter_Data.Value_U8 = Buffer[4];
            break;
        }
        case (Motor_IK_Setting_Parameter_MAX_POWER):
        case (Motor_IK_Setting_Parameter_CURRENT_RAMP):
        {
            Setting_Parameter_Data.Value_I16 = Read_I16_LE(&Buffer[4]);
            break;
        }
        case (Motor_IK_Setting_Parameter_MAX_SPEED):
        case (Motor_IK_Setting_Parameter_MAX_ANGLE):
        case (Motor_IK_Setting_Parameter_SPEED_RAMP):
        {
            Setting_Parameter_Data.Value_I32 = Read_I32_LE(&Buffer[4]);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    else if ((Buffer[1] == Motor_IK_Setting_PID_POSITION) ||
             (Buffer[1] == Motor_IK_Setting_PID_SPEED) ||
             (Buffer[1] == Motor_IK_Setting_PID_CURRENT))
    {
        Setting_Parameter_Data.PID.K_P = Read_U16_LE(&Buffer[2]);
        Setting_Parameter_Data.PID.K_I = Read_U16_LE(&Buffer[4]);
        Setting_Parameter_Data.PID.K_D = Read_U16_LE(&Buffer[6]);
    }
}

/* End of file ---------------------------------------------------------------*/
