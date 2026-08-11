/**
 * @file dvc_motor_RS.cpp
 * @author Project integration
 * @brief RobStride/灵足 EL05 私有扩展 CAN 协议电机驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《EL05 使用说明书 260713》新增
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_RS.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

namespace
{
constexpr float MOTOR_RS_POSITION_MIN = -12.57f;
constexpr float MOTOR_RS_POSITION_MAX = 12.57f;
constexpr float MOTOR_RS_VELOCITY_MIN = -50.0f;
constexpr float MOTOR_RS_VELOCITY_MAX = 50.0f;
constexpr float MOTOR_RS_TORQUE_MIN = -6.0f;
constexpr float MOTOR_RS_TORQUE_MAX = 6.0f;
constexpr float MOTOR_RS_KP_MIN = 0.0f;
constexpr float MOTOR_RS_KP_MAX = 500.0f;
constexpr float MOTOR_RS_KD_MIN = 0.0f;
constexpr float MOTOR_RS_KD_MAX = 5.0f;
constexpr float MOTOR_RS_CURRENT_MIN = -11.0f;
constexpr float MOTOR_RS_CURRENT_MAX = 11.0f;
constexpr float MOTOR_RS_CELSIUS_TO_KELVIN = 273.15f;
constexpr uint32_t MOTOR_RS_EXTENDED_ID_MASK = 0x1FFFFFFFUL;
constexpr uint8_t MOTOR_RS_DEVICE_REPLY_TARGET = 0xFEu;

static_assert(sizeof(float) == sizeof(uint32_t), "RobStride protocol requires a 32-bit IEEE-754 float");

float Clamp_Float(const float Value, const float Min, const float Max)
{
    if (Value < Min)
    {
        return (Min);
    }
    if (Value > Max)
    {
        return (Max);
    }
    return (Value);
}

float Absolute_Float(const float Value)
{
    return ((Value < 0.0f) ? -Value : Value);
}

uint16_t Float_To_U16(const float Value, const float Min, const float Max)
{
    const float Clamped = Clamp_Float(Value, Min, Max);
    const float Normalized = (Clamped - Min) / (Max - Min);
    return (static_cast<uint16_t>(Normalized * 65535.0f + 0.5f));
}

float U16_To_Float(const uint16_t Value, const float Min, const float Max)
{
    return (static_cast<float>(Value) * (Max - Min) / 65535.0f + Min);
}

uint16_t Read_U16_LE(const uint8_t *Data)
{
    return (static_cast<uint16_t>(Data[0]) |
            (static_cast<uint16_t>(Data[1]) << 8));
}

uint16_t Read_U16_BE(const uint8_t *Data)
{
    return ((static_cast<uint16_t>(Data[0]) << 8) |
            static_cast<uint16_t>(Data[1]));
}

int16_t Read_I16_BE(const uint8_t *Data)
{
    return (static_cast<int16_t>(Read_U16_BE(Data)));
}

uint32_t Read_U32_LE(const uint8_t *Data)
{
    return (static_cast<uint32_t>(Data[0]) |
            (static_cast<uint32_t>(Data[1]) << 8) |
            (static_cast<uint32_t>(Data[2]) << 16) |
            (static_cast<uint32_t>(Data[3]) << 24));
}

uint32_t Read_U32_BE(const uint8_t *Data)
{
    return ((static_cast<uint32_t>(Data[0]) << 24) |
            (static_cast<uint32_t>(Data[1]) << 16) |
            (static_cast<uint32_t>(Data[2]) << 8) |
            static_cast<uint32_t>(Data[3]));
}

uint64_t Read_U64_LE(const uint8_t *Data)
{
    uint64_t Value = 0;
    for (uint8_t i = 0; i < 8u; i++)
    {
        Value |= (static_cast<uint64_t>(Data[i]) << (8u * i));
    }
    return (Value);
}

uint64_t Read_U64_BE(const uint8_t *Data)
{
    uint64_t Value = 0;
    for (uint8_t i = 0; i < 8u; i++)
    {
        Value = (Value << 8) | static_cast<uint64_t>(Data[i]);
    }
    return (Value);
}

float Read_Float_LE(const uint8_t *Data)
{
    const uint32_t Raw = Read_U32_LE(Data);
    float Value = 0.0f;
    memcpy(&Value, &Raw, sizeof(Value));
    return (Value);
}

void Write_U16_LE(uint8_t *Data, const uint16_t Value)
{
    Data[0] = static_cast<uint8_t>(Value & 0xFFu);
    Data[1] = static_cast<uint8_t>((Value >> 8) & 0xFFu);
}

void Write_U16_BE(uint8_t *Data, const uint16_t Value)
{
    Data[0] = static_cast<uint8_t>((Value >> 8) & 0xFFu);
    Data[1] = static_cast<uint8_t>(Value & 0xFFu);
}

void Write_U32_LE(uint8_t *Data, const uint32_t Value)
{
    Data[0] = static_cast<uint8_t>(Value & 0xFFu);
    Data[1] = static_cast<uint8_t>((Value >> 8) & 0xFFu);
    Data[2] = static_cast<uint8_t>((Value >> 16) & 0xFFu);
    Data[3] = static_cast<uint8_t>((Value >> 24) & 0xFFu);
}

void Write_Float_LE(uint8_t *Data, const float Value)
{
    uint32_t Raw = 0;
    memcpy(&Raw, &Value, sizeof(Raw));
    Write_U32_LE(Data, Raw);
}
} // namespace

/* Function prototypes -------------------------------------------------------*/

uint8_t Class_Motor_RS::CAN_Config_Extended_Filter(FDCAN_HandleTypeDef *hcan,
                                                    const uint32_t &__Filter_Index)
{
    if (hcan == nullptr)
    {
        return (0xFFu);
    }

    FDCAN_FilterTypeDef Filter = {};
    Filter.IdType = FDCAN_EXTENDED_ID;
    Filter.FilterIndex = __Filter_Index;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1 = 0x00000000UL;
    Filter.FilterID2 = 0x00000000UL;

    return (static_cast<uint8_t>(HAL_FDCAN_ConfigFilter(hcan, &Filter)));
}

uint32_t Class_Motor_RS::Build_Extended_ID(const uint8_t &__Communication_Type,
                                           const uint16_t &__Data_Field,
                                           const uint8_t &__Target_ID)
{
    return ((((static_cast<uint32_t>(__Communication_Type) & 0x1Fu) << 24) |
             (static_cast<uint32_t>(__Data_Field) << 8) |
             static_cast<uint32_t>(__Target_ID)) &
            MOTOR_RS_EXTENDED_ID_MASK);
}

Struct_Motor_RS_Extended_ID Class_Motor_RS::Parse_Extended_ID(const uint32_t &__Extended_ID)
{
    Struct_Motor_RS_Extended_ID Result = {};
    Result.Raw_ID = (__Extended_ID & MOTOR_RS_EXTENDED_ID_MASK);
    Result.Communication_Type = static_cast<uint8_t>((Result.Raw_ID >> 24) & 0x1Fu);
    Result.Data_Field = static_cast<uint16_t>((Result.Raw_ID >> 8) & 0xFFFFu);
    Result.Target_ID = static_cast<uint8_t>(Result.Raw_ID & 0xFFu);
    return (Result);
}

uint8_t Class_Motor_RS::Clamp_Motor_ID(const uint8_t &__Motor_ID)
{
    return ((__Motor_ID > 0x7Fu) ? 0x7Fu : __Motor_ID);
}

void Class_Motor_RS::Init(const FDCAN_HandleTypeDef *hcan,
                          const uint8_t &__Motor_ID,
                          const uint8_t &__Master_ID)
{
    CAN_Handler = const_cast<FDCAN_HandleTypeDef *>(hcan);
    CAN_Manage_Object = nullptr;

    if (hcan != nullptr)
    {
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
    }

    Motor_ID = Clamp_Motor_ID(__Motor_ID);
    Master_ID = __Master_ID;
    Last_Tx_Status = 0xFFu;

    Device_Query_Pending = false;
    Device_Query_Target = 0x7Fu;
    CAN_ID_Change_Pending = false;
    Pending_Motor_ID = 0u;

    Flag = 0u;
    Pre_Flag = 0u;
    Motor_RS_Status = Motor_RS_Status_DISABLE;

    Angle_Unwrap_Initialized = false;
    Previous_Cyclic_Angle = 0.0f;

    Control_Method = Motor_RS_Control_Method_NONE;
    Control_Enable = false;
    Control_Angle = 0.0f;
    Control_Omega = 0.0f;
    Control_Torque = 0.0f;
    Control_Current = 0.0f;
    Control_K_P = 0.0f;
    Control_K_D = 0.0f;
    Control_Current_Limit = 11.0f;
    Control_Speed_Limit = 10.0f;
    Control_Acceleration = 20.0f;
    Control_Deceleration = 0.0f;
    PP_Command_Pending = false;

    Reset_Data();
}

void Class_Motor_RS::Reset_Data()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(&Device_Info, 0, sizeof(Device_Info));
    memset(&Version_Data, 0, sizeof(Version_Data));
    memset(&Parameter_Data, 0, sizeof(Parameter_Data));
    memset(&Fault_Data, 0, sizeof(Fault_Data));

    Rx_Data.Motor_ID = Motor_ID;
    Rx_Data.Master_ID = Master_ID;
    Rx_Data.Mode_State = Motor_RS_Mode_State_RESET;
    Rx_Data.Now_Temperature = MOTOR_RS_CELSIUS_TO_KELVIN;
}

void Class_Motor_RS::CAN_RxCpltCallback()
{
    if (CAN_Manage_Object == nullptr)
    {
        return;
    }

    CAN_RxCpltCallback(CAN_Manage_Object->Rx_Header, CAN_Manage_Object->Rx_Buffer);
}

bool Class_Motor_RS::Is_Frame_For_This_Motor(const Struct_Motor_RS_Extended_ID &__ID) const
{
    const uint8_t Source_Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);

    if (__ID.Communication_Type == Motor_RS_Communication_Type_GET_DEVICE_ID)
    {
        if (__ID.Target_ID != MOTOR_RS_DEVICE_REPLY_TARGET)
        {
            return (false);
        }

        return ((Source_Motor_ID == Motor_ID) ||
                (CAN_ID_Change_Pending && (Source_Motor_ID == Pending_Motor_ID)) ||
                (Device_Query_Pending && (Source_Motor_ID == Device_Query_Target)));
    }

    switch (__ID.Communication_Type)
    {
    case (Motor_RS_Communication_Type_FEEDBACK):
    case (Motor_RS_Communication_Type_ACTIVE_REPORT):
    case (Motor_RS_Communication_Type_READ_PARAMETER):
    case (Motor_RS_Communication_Type_FAULT):
    {
        return ((__ID.Target_ID == Master_ID) && (Source_Motor_ID == Motor_ID));
    }
    default:
    {
        return (false);
    }
    }
}

void Class_Motor_RS::CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header,
                                        const uint8_t *Buffer)
{
    if ((Buffer == nullptr) || (Header.IdType != FDCAN_EXTENDED_ID))
    {
        return;
    }

    const Struct_Motor_RS_Extended_ID ID = Parse_Extended_ID(Header.Identifier);
    if (!Is_Frame_For_This_Motor(ID))
    {
        return;
    }

    const uint8_t Source_Motor_ID = static_cast<uint8_t>(ID.Data_Field & 0xFFu);
    if ((Source_Motor_ID == Motor_ID) ||
        (CAN_ID_Change_Pending && (Source_Motor_ID == Pending_Motor_ID)))
    {
        Flag++;
    }

    Data_Process(ID, Buffer);
}

void Class_Motor_RS::Send_Frame(const uint32_t &__Extended_ID,
                                const uint8_t __Data[8]) const
{
    if ((CAN_Handler == nullptr) || (__Data == nullptr))
    {
        Last_Tx_Status = 0xFFu;
        return;
    }

    FDCAN_TxHeaderTypeDef Header = {};
    Header.Identifier = (__Extended_ID & MOTOR_RS_EXTENDED_ID_MASK);
    Header.IdType = FDCAN_EXTENDED_ID;
    Header.TxFrameType = FDCAN_DATA_FRAME;
#ifdef FDCAN_DLC_BYTES_8
    Header.DataLength = FDCAN_DLC_BYTES_8;
#else
    Header.DataLength = 8u;
#endif
    Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Header.BitRateSwitch = FDCAN_BRS_OFF;
    Header.FDFormat = FDCAN_CLASSIC_CAN;
    Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Header.MessageMarker = 0u;

    Last_Tx_Status = static_cast<uint8_t>(HAL_FDCAN_AddMessageToTxFifoQ(
        CAN_Handler,
        &Header,
        const_cast<uint8_t *>(__Data)));
}

void Class_Motor_RS::Send_Command(const uint8_t &__Communication_Type,
                                  const uint16_t &__Data_Field,
                                  const uint8_t &__Target_ID,
                                  const uint8_t __Data[8]) const
{
    Send_Frame(Build_Extended_ID(__Communication_Type, __Data_Field, __Target_ID), __Data);
}

void Class_Motor_RS::CAN_Send_Get_Device_ID()
{
    CAN_Send_Get_Device_ID(Motor_ID);
}

void Class_Motor_RS::CAN_Send_Get_Device_ID(const uint8_t &__Target_ID)
{
    const uint8_t Target_ID = Clamp_Motor_ID(__Target_ID);
    uint8_t Data[8] = {0};

    Device_Query_Pending = true;
    Device_Query_Target = Target_ID;
    Device_Info.Valid = false;

    Send_Command(Motor_RS_Communication_Type_GET_DEVICE_ID,
                 static_cast<uint16_t>(Master_ID),
                 Target_ID,
                 Data);

    if (Last_Tx_Status != static_cast<uint8_t>(HAL_OK))
    {
        Device_Query_Pending = false;
    }
}

void Class_Motor_RS::CAN_Send_Motion_Control(const float &__Angle,
                                             const float &__Omega,
                                             const float &__K_P,
                                             const float &__K_D,
                                             const float &__Torque) const
{
    uint8_t Data[8] = {0};

    Write_U16_BE(&Data[0], Float_To_U16(__Angle, MOTOR_RS_POSITION_MIN, MOTOR_RS_POSITION_MAX));
    Write_U16_BE(&Data[2], Float_To_U16(__Omega, MOTOR_RS_VELOCITY_MIN, MOTOR_RS_VELOCITY_MAX));
    Write_U16_BE(&Data[4], Float_To_U16(__K_P, MOTOR_RS_KP_MIN, MOTOR_RS_KP_MAX));
    Write_U16_BE(&Data[6], Float_To_U16(__K_D, MOTOR_RS_KD_MIN, MOTOR_RS_KD_MAX));

    const uint16_t Torque = Float_To_U16(__Torque, MOTOR_RS_TORQUE_MIN, MOTOR_RS_TORQUE_MAX);
    Send_Command(Motor_RS_Communication_Type_MOTION_CONTROL, Torque, Motor_ID, Data);
}

void Class_Motor_RS::CAN_Send_Enable() const
{
    uint8_t Data[8] = {0};
    Send_Command(Motor_RS_Communication_Type_ENABLE,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Stop(const bool &__Clear_Error) const
{
    uint8_t Data[8] = {0};
    Data[0] = (__Clear_Error ? 1u : 0u);
    Send_Command(Motor_RS_Communication_Type_STOP,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Clear_Error() const
{
    CAN_Send_Stop(true);
}

void Class_Motor_RS::CAN_Send_Read_Version() const
{
    uint8_t Data[8] = {0};
    Data[1] = 0xC4u;
    Send_Command(Motor_RS_Communication_Type_STOP,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Set_Zero() const
{
    uint8_t Data[8] = {0};
    Data[0] = 1u;
    Send_Command(Motor_RS_Communication_Type_SET_ZERO,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Set_CAN_ID(const uint8_t &__New_Motor_ID)
{
    const uint8_t New_ID = Clamp_Motor_ID(__New_Motor_ID);
    uint8_t Data[8] = {0};

    Pending_Motor_ID = New_ID;
    CAN_ID_Change_Pending = true;

    const uint16_t Data_Field = static_cast<uint16_t>(
        (static_cast<uint16_t>(New_ID) << 8) |
        static_cast<uint16_t>(Master_ID));

    Send_Command(Motor_RS_Communication_Type_SET_CAN_ID,
                 Data_Field,
                 Motor_ID,
                 Data);

    if (Last_Tx_Status != static_cast<uint8_t>(HAL_OK))
    {
        CAN_ID_Change_Pending = false;
    }
}

void Class_Motor_RS::CAN_Send_Read_Parameter(const uint16_t &__Index) const
{
    uint8_t Data[8] = {0};
    Write_U16_LE(&Data[0], __Index);
    Send_Command(Motor_RS_Communication_Type_READ_PARAMETER,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Write_Parameter_Raw(const uint16_t &__Index,
                                                  const uint8_t __Raw[4]) const
{
    if (__Raw == nullptr)
    {
        return;
    }

    uint8_t Data[8] = {0};
    Write_U16_LE(&Data[0], __Index);
    memcpy(&Data[4], __Raw, 4u);
    Send_Command(Motor_RS_Communication_Type_WRITE_PARAMETER,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Write_Parameter_U8(const uint16_t &__Index,
                                                 const uint8_t &__Value) const
{
    const uint8_t Raw[4] = {__Value, 0u, 0u, 0u};
    CAN_Send_Write_Parameter_Raw(__Index, Raw);
}

void Class_Motor_RS::CAN_Send_Write_Parameter_U16(const uint16_t &__Index,
                                                  const uint16_t &__Value) const
{
    uint8_t Raw[4] = {0};
    Write_U16_LE(Raw, __Value);
    CAN_Send_Write_Parameter_Raw(__Index, Raw);
}

void Class_Motor_RS::CAN_Send_Write_Parameter_U32(const uint16_t &__Index,
                                                  const uint32_t &__Value) const
{
    uint8_t Raw[4] = {0};
    Write_U32_LE(Raw, __Value);
    CAN_Send_Write_Parameter_Raw(__Index, Raw);
}

void Class_Motor_RS::CAN_Send_Write_Parameter_I32(const uint16_t &__Index,
                                                  const int32_t &__Value) const
{
    CAN_Send_Write_Parameter_U32(__Index, static_cast<uint32_t>(__Value));
}

void Class_Motor_RS::CAN_Send_Write_Parameter_Float(const uint16_t &__Index,
                                                    const float &__Value) const
{
    uint8_t Raw[4] = {0};
    Write_Float_LE(Raw, __Value);
    CAN_Send_Write_Parameter_Raw(__Index, Raw);
}

void Class_Motor_RS::CAN_Send_Set_Run_Mode(const Enum_Motor_RS_Run_Mode &__Run_Mode) const
{
    CAN_Send_Write_Parameter_U8(Motor_RS_Parameter_RUN_MODE,
                                static_cast<uint8_t>(__Run_Mode));
}

void Class_Motor_RS::CAN_Send_Set_Current_Reference(const float &__Current) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_IQ_REFERENCE,
        Clamp_Float(__Current, MOTOR_RS_CURRENT_MIN, MOTOR_RS_CURRENT_MAX));
}

void Class_Motor_RS::CAN_Send_Set_Speed_Reference(const float &__Omega) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_SPEED_REFERENCE,
        Clamp_Float(__Omega, MOTOR_RS_VELOCITY_MIN, MOTOR_RS_VELOCITY_MAX));
}

void Class_Motor_RS::CAN_Send_Set_Position_Reference(const float &__Angle) const
{
    CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_POSITION_REFERENCE, __Angle);
}

void Class_Motor_RS::CAN_Send_Set_Torque_Limit(const float &__Torque_Limit) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_TORQUE_LIMIT,
        Clamp_Float(Absolute_Float(__Torque_Limit), 0.0f, MOTOR_RS_TORQUE_MAX));
}

void Class_Motor_RS::CAN_Send_Set_Current_Limit(const float &__Current_Limit) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_CURRENT_LIMIT,
        Clamp_Float(Absolute_Float(__Current_Limit), 0.0f, MOTOR_RS_CURRENT_MAX));
}

void Class_Motor_RS::CAN_Send_Set_CSP_Speed_Limit(const float &__Omega_Limit) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_CSP_SPEED_LIMIT,
        Clamp_Float(Absolute_Float(__Omega_Limit), 0.0f, MOTOR_RS_VELOCITY_MAX));
}

void Class_Motor_RS::CAN_Send_Set_Speed_Acceleration(const float &__Acceleration) const
{
    CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_SPEED_ACCELERATION,
                                   Absolute_Float(__Acceleration));
}

void Class_Motor_RS::CAN_Send_Set_PP_Max_Speed(const float &__Omega_Limit) const
{
    CAN_Send_Write_Parameter_Float(
        Motor_RS_Parameter_PP_MAX_SPEED,
        Clamp_Float(Absolute_Float(__Omega_Limit), 0.0f, MOTOR_RS_VELOCITY_MAX));
}

void Class_Motor_RS::CAN_Send_Set_PP_Acceleration(const float &__Acceleration) const
{
    CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_PP_ACCELERATION,
                                   Absolute_Float(__Acceleration));
}

void Class_Motor_RS::CAN_Send_Set_PP_Deceleration(const float &__Deceleration) const
{
    CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_PP_DECELERATION,
                                   Absolute_Float(__Deceleration));
}

void Class_Motor_RS::CAN_Send_Set_Active_Report_Period_Raw(const uint16_t &__Period_Raw) const
{
    const uint16_t Period = (__Period_Raw == 0u) ? 1u : __Period_Raw;
    CAN_Send_Write_Parameter_U16(Motor_RS_Parameter_ACTIVE_REPORT_TIME, Period);
}

void Class_Motor_RS::CAN_Send_Set_Active_Report_Period_MS(const uint16_t &__Period_MS) const
{
    uint16_t Period_Raw = 1u;
    if (__Period_MS > 10u)
    {
        Period_Raw = static_cast<uint16_t>(
            1u + static_cast<uint16_t>((static_cast<uint32_t>(__Period_MS - 10u) + 2u) / 5u));
    }
    CAN_Send_Set_Active_Report_Period_Raw(Period_Raw);
}

void Class_Motor_RS::CAN_Send_Set_CAN_Timeout_Raw(const uint32_t &__Timeout_Raw) const
{
    CAN_Send_Write_Parameter_U32(Motor_RS_Parameter_CAN_TIMEOUT, __Timeout_Raw);
}

void Class_Motor_RS::CAN_Send_Set_CAN_Timeout_Seconds(const float &__Timeout_Seconds) const
{
    float Seconds = __Timeout_Seconds;
    if (Seconds < 0.0f)
    {
        Seconds = 0.0f;
    }

    const double Raw_Double = static_cast<double>(Seconds) * 20000.0;
    const uint32_t Raw = (Raw_Double >= 4294967295.0)
                             ? 0xFFFFFFFFUL
                             : static_cast<uint32_t>(Raw_Double + 0.5);
    CAN_Send_Set_CAN_Timeout_Raw(Raw);
}

void Class_Motor_RS::CAN_Send_Set_Zero_Range(const Enum_Motor_RS_Zero_Range &__Zero_Range) const
{
    CAN_Send_Write_Parameter_U8(Motor_RS_Parameter_ZERO_RANGE,
                                static_cast<uint8_t>(__Zero_Range));
}

void Class_Motor_RS::CAN_Send_Set_Zero_Offset(const float &__Offset) const
{
    CAN_Send_Write_Parameter_Float(Motor_RS_Parameter_ZERO_OFFSET, __Offset);
}

void Class_Motor_RS::CAN_Send_Set_Cogging_Compensation(const bool &__Enable) const
{
    CAN_Send_Write_Parameter_U8(Motor_RS_Parameter_COGGING_COMPENSATION_ENABLE,
                                static_cast<uint8_t>(__Enable ? 1u : 0u));
}

void Class_Motor_RS::CAN_Send_Set_Initial_Calibration(const bool &__Enable) const
{
    CAN_Send_Write_Parameter_U8(Motor_RS_Parameter_INITIAL_CALIBRATION_ENABLE,
                                static_cast<uint8_t>(__Enable ? 1u : 0u));
}

void Class_Motor_RS::CAN_Send_Read_Fault_State() const
{
    uint8_t Data[8] = {0};
    Send_Command(Motor_RS_Communication_Type_FAULT,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Save_Parameters() const
{
    const uint8_t Data[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    Send_Command(Motor_RS_Communication_Type_SAVE,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Set_Baudrate(const Enum_Motor_RS_CAN_Baudrate &__Baudrate) const
{
    const uint8_t Data[8] = {1u, 2u, 3u, 4u, 5u, 6u,
                             static_cast<uint8_t>(__Baudrate), 0u};
    Send_Command(Motor_RS_Communication_Type_SET_BAUDRATE,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Set_Active_Report(const bool &__Enable) const
{
    const uint8_t Data[8] = {1u, 2u, 3u, 4u, 5u, 6u,
                             static_cast<uint8_t>(__Enable ? 1u : 0u), 0u};
    Send_Command(Motor_RS_Communication_Type_ACTIVE_REPORT,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Set_Protocol(const Enum_Motor_RS_Protocol &__Protocol) const
{
    const uint8_t Data[8] = {1u, 2u, 3u, 4u, 5u, 6u,
                             static_cast<uint8_t>(__Protocol), 0u};
    Send_Command(Motor_RS_Communication_Type_SET_PROTOCOL,
                 static_cast<uint16_t>(Master_ID),
                 Motor_ID,
                 Data);
}

void Class_Motor_RS::CAN_Send_Apply_Control_Configuration() const
{
    switch (Control_Method)
    {
    case (Motor_RS_Control_Method_MOTION):
    {
        CAN_Send_Set_Run_Mode(Motor_RS_Run_Mode_MOTION);
        break;
    }
    case (Motor_RS_Control_Method_CURRENT):
    {
        CAN_Send_Set_Run_Mode(Motor_RS_Run_Mode_CURRENT);
        break;
    }
    case (Motor_RS_Control_Method_SPEED):
    {
        CAN_Send_Set_Run_Mode(Motor_RS_Run_Mode_SPEED);
        CAN_Send_Set_Current_Limit(Control_Current_Limit);
        CAN_Send_Set_Speed_Acceleration(Control_Acceleration);
        break;
    }
    case (Motor_RS_Control_Method_POSITION_CSP):
    {
        CAN_Send_Set_Run_Mode(Motor_RS_Run_Mode_POSITION_CSP);
        CAN_Send_Set_CSP_Speed_Limit(Control_Speed_Limit);
        CAN_Send_Set_Current_Limit(Control_Current_Limit);
        break;
    }
    case (Motor_RS_Control_Method_POSITION_PP):
    {
        CAN_Send_Set_Run_Mode(Motor_RS_Run_Mode_POSITION_PP);
        CAN_Send_Set_PP_Max_Speed(Control_Speed_Limit);
        CAN_Send_Set_PP_Acceleration(Control_Acceleration);
        CAN_Send_Set_Current_Limit(Control_Current_Limit);
        if (Control_Deceleration > 0.0f)
        {
            CAN_Send_Set_PP_Deceleration(Control_Deceleration);
        }
        break;
    }
    case (Motor_RS_Control_Method_NONE):
    default:
    {
        break;
    }
    }
}

void Class_Motor_RS::CAN_Send_Start_Control()
{
    CAN_Send_Stop(false);
    CAN_Send_Apply_Control_Configuration();
    CAN_Send_Enable();
    Control_Enable = true;

    if (Control_Method == Motor_RS_Control_Method_POSITION_PP)
    {
        PP_Command_Pending = true;
    }
}

void Class_Motor_RS::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_RS_Status = (Flag == Pre_Flag)
                          ? Motor_RS_Status_DISABLE
                          : Motor_RS_Status_ENABLE;
    Pre_Flag = Flag;
}

void Class_Motor_RS::TIM_Send_PeriodElapsedCallback()
{
    if (!Control_Enable)
    {
        return;
    }

    Output();
}

void Class_Motor_RS::Output()
{
    switch (Control_Method)
    {
    case (Motor_RS_Control_Method_MOTION):
    {
        CAN_Send_Motion_Control(Control_Angle,
                                Control_Omega,
                                Control_K_P,
                                Control_K_D,
                                Control_Torque);
        break;
    }
    case (Motor_RS_Control_Method_CURRENT):
    {
        CAN_Send_Set_Current_Reference(Control_Current);
        break;
    }
    case (Motor_RS_Control_Method_SPEED):
    {
        CAN_Send_Set_Speed_Reference(Control_Omega);
        break;
    }
    case (Motor_RS_Control_Method_POSITION_CSP):
    {
        CAN_Send_Set_Position_Reference(Control_Angle);
        break;
    }
    case (Motor_RS_Control_Method_POSITION_PP):
    {
        if (PP_Command_Pending)
        {
            CAN_Send_Set_Position_Reference(Control_Angle);
            PP_Command_Pending = false;
        }
        break;
    }
    case (Motor_RS_Control_Method_NONE):
    default:
    {
        break;
    }
    }
}

void Class_Motor_RS::Data_Process(const Struct_Motor_RS_Extended_ID &__ID,
                                  const uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return;
    }

    Rx_Data.Last_Extended_ID = __ID.Raw_ID;
    Rx_Data.Last_Communication_Type = __ID.Communication_Type;
    Rx_Data.Last_Data_Field = __ID.Data_Field;
    Rx_Data.Last_Target_ID = __ID.Target_ID;
    memcpy(Rx_Data.Last_Raw_Data, Buffer, 8u);

    switch (__ID.Communication_Type)
    {
    case (Motor_RS_Communication_Type_GET_DEVICE_ID):
    {
        Parse_Device_Info(__ID, Buffer);
        break;
    }
    case (Motor_RS_Communication_Type_FEEDBACK):
    {
        if ((Buffer[0] == 0x00u) &&
            (Buffer[1] == 0xC4u) &&
            (Buffer[2] == 0x56u))
        {
            Parse_Version(__ID, Buffer);
        }
        else
        {
            Parse_Feedback(__ID, Buffer);
        }
        break;
    }
    case (Motor_RS_Communication_Type_ACTIVE_REPORT):
    {
        Parse_Feedback(__ID, Buffer);
        break;
    }
    case (Motor_RS_Communication_Type_READ_PARAMETER):
    {
        Parse_Parameter(__ID, Buffer);
        break;
    }
    case (Motor_RS_Communication_Type_FAULT):
    {
        Parse_Fault(__ID, Buffer);
        break;
    }
    default:
    {
        break;
    }
    }
}

void Class_Motor_RS::Parse_Device_Info(const Struct_Motor_RS_Extended_ID &__ID,
                                       const uint8_t *Buffer)
{
    const uint8_t Source_Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);

    Device_Info.Valid = true;
    Device_Info.Motor_ID = Source_Motor_ID;
    memcpy(Device_Info.UID_Raw, Buffer, 8u);
    Device_Info.UID_Little_Endian = Read_U64_LE(Buffer);
    Device_Info.UID_Big_Endian = Read_U64_BE(Buffer);

    if (CAN_ID_Change_Pending && (Source_Motor_ID == Pending_Motor_ID))
    {
        Motor_ID = Pending_Motor_ID;
        Rx_Data.Motor_ID = Motor_ID;
        CAN_ID_Change_Pending = false;
    }

    if (Device_Query_Pending && (Source_Motor_ID == Device_Query_Target))
    {
        Device_Query_Pending = false;
    }
}

void Class_Motor_RS::Parse_Feedback(const Struct_Motor_RS_Extended_ID &__ID,
                                    const uint8_t *Buffer)
{
    Rx_Data.Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);
    Rx_Data.Master_ID = __ID.Target_ID;
    Rx_Data.Feedback_Fault = static_cast<uint8_t>((__ID.Data_Field >> 8) & 0x3Fu);
    Rx_Data.Mode_State = static_cast<Enum_Motor_RS_Mode_State>((__ID.Data_Field >> 14) & 0x03u);

    Rx_Data.Position_Raw = Read_U16_BE(&Buffer[0]);
    Rx_Data.Velocity_Raw = Read_U16_BE(&Buffer[2]);
    Rx_Data.Torque_Raw = Read_U16_BE(&Buffer[4]);
    Rx_Data.Temperature_Raw = Read_I16_BE(&Buffer[6]);

    const float Cyclic_Angle = U16_To_Float(Rx_Data.Position_Raw,
                                            MOTOR_RS_POSITION_MIN,
                                            MOTOR_RS_POSITION_MAX);
    Rx_Data.Now_Angle = Cyclic_Angle;

    if (!Angle_Unwrap_Initialized)
    {
        Angle_Unwrap_Initialized = true;
        Previous_Cyclic_Angle = Cyclic_Angle;
        Rx_Data.Total_Angle = Cyclic_Angle;
    }
    else
    {
        const float Period = MOTOR_RS_POSITION_MAX - MOTOR_RS_POSITION_MIN;
        const float Half_Period = Period * 0.5f;
        float Delta = Cyclic_Angle - Previous_Cyclic_Angle;

        if (Delta > Half_Period)
        {
            Delta -= Period;
        }
        else if (Delta < -Half_Period)
        {
            Delta += Period;
        }

        Rx_Data.Total_Angle += Delta;
        Previous_Cyclic_Angle = Cyclic_Angle;
    }

    Rx_Data.Now_Omega = U16_To_Float(Rx_Data.Velocity_Raw,
                                     MOTOR_RS_VELOCITY_MIN,
                                     MOTOR_RS_VELOCITY_MAX);
    Rx_Data.Now_Torque = U16_To_Float(Rx_Data.Torque_Raw,
                                      MOTOR_RS_TORQUE_MIN,
                                      MOTOR_RS_TORQUE_MAX);
    Rx_Data.Now_Temperature = static_cast<float>(Rx_Data.Temperature_Raw) * 0.1f +
                              MOTOR_RS_CELSIUS_TO_KELVIN;
}

void Class_Motor_RS::Parse_Version(const Struct_Motor_RS_Extended_ID &__ID,
                                   const uint8_t *Buffer)
{
    Version_Data.Valid = true;
    Version_Data.Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);
    memcpy(Version_Data.Version_Byte, &Buffer[3], 4u);
    Version_Data.Version_U32 = Read_U32_BE(&Buffer[3]);
}

Enum_Motor_RS_Parameter_Type Class_Motor_RS::Get_Parameter_Type(const uint16_t &__Index)
{
    switch (__Index)
    {
    case (Motor_RS_Parameter_RUN_MODE):
    case (Motor_RS_Parameter_ZERO_RANGE):
    case (Motor_RS_Parameter_COGGING_COMPENSATION_ENABLE):
    case (Motor_RS_Parameter_INITIAL_CALIBRATION_ENABLE):
    {
        return (Motor_RS_Parameter_Type_UINT8);
    }
    case (Motor_RS_Parameter_ACTIVE_REPORT_TIME):
    {
        return (Motor_RS_Parameter_Type_UINT16);
    }
    case (Motor_RS_Parameter_CAN_TIMEOUT):
    {
        return (Motor_RS_Parameter_Type_UINT32);
    }
    case (Motor_RS_Parameter_IQ_REFERENCE):
    case (Motor_RS_Parameter_SPEED_REFERENCE):
    case (Motor_RS_Parameter_TORQUE_LIMIT):
    case (Motor_RS_Parameter_CURRENT_KP):
    case (Motor_RS_Parameter_CURRENT_KI):
    case (Motor_RS_Parameter_CURRENT_FILTER_GAIN):
    case (Motor_RS_Parameter_POSITION_REFERENCE):
    case (Motor_RS_Parameter_CSP_SPEED_LIMIT):
    case (Motor_RS_Parameter_CURRENT_LIMIT):
    case (Motor_RS_Parameter_MECHANICAL_POSITION):
    case (Motor_RS_Parameter_IQ_FILTERED):
    case (Motor_RS_Parameter_MECHANICAL_VELOCITY):
    case (Motor_RS_Parameter_BUS_VOLTAGE):
    case (Motor_RS_Parameter_POSITION_KP):
    case (Motor_RS_Parameter_SPEED_KP):
    case (Motor_RS_Parameter_SPEED_KI):
    case (Motor_RS_Parameter_SPEED_FILTER_GAIN):
    case (Motor_RS_Parameter_SPEED_ACCELERATION):
    case (Motor_RS_Parameter_PP_MAX_SPEED):
    case (Motor_RS_Parameter_PP_ACCELERATION):
    case (Motor_RS_Parameter_ZERO_OFFSET):
    case (Motor_RS_Parameter_PP_DECELERATION):
    {
        return (Motor_RS_Parameter_Type_FLOAT);
    }
    default:
    {
        return (Motor_RS_Parameter_Type_UNKNOWN);
    }
    }
}

void Class_Motor_RS::Parse_Parameter(const Struct_Motor_RS_Extended_ID &__ID,
                                     const uint8_t *Buffer)
{
    memset(&Parameter_Data, 0, sizeof(Parameter_Data));

    Parameter_Data.Valid = true;
    Parameter_Data.Result_Code = static_cast<uint8_t>((__ID.Data_Field >> 8) & 0xFFu);
    Parameter_Data.Success = (Parameter_Data.Result_Code == 0u);
    Parameter_Data.Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);
    Parameter_Data.Index = Read_U16_LE(&Buffer[0]);
    Parameter_Data.Type = Get_Parameter_Type(Parameter_Data.Index);
    memcpy(Parameter_Data.Raw, &Buffer[4], 4u);

    Parameter_Data.Value_U8 = Buffer[4];
    Parameter_Data.Value_U16 = Read_U16_LE(&Buffer[4]);
    Parameter_Data.Value_U32 = Read_U32_LE(&Buffer[4]);
    Parameter_Data.Value_I32 = static_cast<int32_t>(Parameter_Data.Value_U32);
    Parameter_Data.Value_Float = Read_Float_LE(&Buffer[4]);

    if (!Parameter_Data.Success)
    {
        return;
    }

    switch (Parameter_Data.Index)
    {
    case (Motor_RS_Parameter_MECHANICAL_POSITION):
    {
        Rx_Data.Mechanical_Position = Parameter_Data.Value_Float;
        Rx_Data.Mechanical_Position_Valid = true;
        break;
    }
    case (Motor_RS_Parameter_IQ_FILTERED):
    {
        Rx_Data.IQ_Filtered = Parameter_Data.Value_Float;
        Rx_Data.IQ_Filtered_Valid = true;
        break;
    }
    case (Motor_RS_Parameter_MECHANICAL_VELOCITY):
    {
        Rx_Data.Mechanical_Velocity = Parameter_Data.Value_Float;
        Rx_Data.Mechanical_Velocity_Valid = true;
        break;
    }
    case (Motor_RS_Parameter_BUS_VOLTAGE):
    {
        Rx_Data.Bus_Voltage = Parameter_Data.Value_Float;
        Rx_Data.Bus_Voltage_Valid = true;
        break;
    }
    default:
    {
        break;
    }
    }
}

void Class_Motor_RS::Parse_Fault(const Struct_Motor_RS_Extended_ID &__ID,
                                 const uint8_t *Buffer)
{
    Fault_Data.Valid = true;
    Fault_Data.Motor_ID = static_cast<uint8_t>(__ID.Data_Field & 0xFFu);
    Fault_Data.Fault = Read_U32_LE(&Buffer[0]);
    Fault_Data.Warning = Read_U32_LE(&Buffer[4]);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
