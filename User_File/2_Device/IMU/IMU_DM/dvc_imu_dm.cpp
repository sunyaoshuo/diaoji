/**
 * @file dvc_imu_dm.cpp
 * @author Project integration
 * @brief 达妙 DM-IMU-L1 的 CAN 驱动
 * @version 0.1
 * @date 2026-07-29 0.1 根据 DM-IMU-L1 V1.2 手册和官方 MC02 CAN 例程新建
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_imu_dm.h"

/* Private macros ------------------------------------------------------------*/

static constexpr uint8_t IMU_DM_COMMAND_HEADER = 0xcc;
static constexpr uint8_t IMU_DM_COMMAND_SEPARATOR = 0xdd;
static constexpr uint16_t IMU_DM_STANDARD_ID_MAX = 0x7ff;

/* Private function declarations ---------------------------------------------*/

static uint16_t IMU_DM_Read_LE_U16(const uint8_t *Data)
{
    return (static_cast<uint16_t>(Data[0]) |
            (static_cast<uint16_t>(Data[1]) << 8));
}

static uint32_t IMU_DM_Read_LE_U32(const uint8_t *Data)
{
    return (static_cast<uint32_t>(Data[0]) |
            (static_cast<uint32_t>(Data[1]) << 8) |
            (static_cast<uint32_t>(Data[2]) << 16) |
            (static_cast<uint32_t>(Data[3]) << 24));
}

static void IMU_DM_Write_LE_U32(uint8_t *Data, const uint32_t &Value)
{
    Data[0] = static_cast<uint8_t>(Value);
    Data[1] = static_cast<uint8_t>(Value >> 8);
    Data[2] = static_cast<uint8_t>(Value >> 16);
    Data[3] = static_cast<uint8_t>(Value >> 24);
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 IMU 对象
 */
void Class_IMU_DM::Init(const FDCAN_HandleTypeDef *hcan,
                        const uint16_t &__CAN_ID,
                        const uint16_t &__Master_ID,
                        const uint16_t &__Offline_Timeout_100ms)
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

        if ((CAN_Manage_Object != nullptr) &&
            (CAN_Manage_Object->CAN_Handler == nullptr))
        {
            CAN_Manage_Object->CAN_Handler = CAN_Handler;
        }
    }

    if (__CAN_ID <= IMU_DM_STANDARD_ID_MAX)
    {
        CAN_ID = __CAN_ID;
    }
    if (__Master_ID <= IMU_DM_STANDARD_ID_MAX)
    {
        Master_ID = __Master_ID;
    }

    Offline_Timeout_100ms = (__Offline_Timeout_100ms == 0U) ? 1U : __Offline_Timeout_100ms;
    Offline_Countdown_100ms = 0;

    Pending_CAN_ID = 0;
    Pending_Master_ID = 0;
    Pending_CAN_ID_Valid = false;
    Pending_Master_ID_Valid = false;

    Flag = 0;
    Pre_Flag = 0;
    IMU_Status = IMU_DM_Status_DISABLE;
    Last_Rx_Packet_Type = IMU_DM_Rx_Packet_Type_NONE;
    Last_Rx_Timestamp = 0;

    Acceleration_Data = {};
    Gyroscope_Data = {};
    Euler_Angle_Data = {};
    Quaternion_Data = {};
    Response_Data = {};
    Response_Data.Ack = IMU_DM_Ack_NONE;
    Parser_Statistics = {};
}

/**
 * @brief 只修改驱动本地 CAN ID
 */
bool Class_IMU_DM::Set_Local_CAN_ID(const uint16_t &__CAN_ID)
{
    if (__CAN_ID > IMU_DM_STANDARD_ID_MAX)
    {
        return (false);
    }

    CAN_ID = __CAN_ID;
    Pending_CAN_ID_Valid = false;
    return (true);
}

/**
 * @brief 只修改驱动本地 Master ID
 */
bool Class_IMU_DM::Set_Local_Master_ID(const uint16_t &__Master_ID)
{
    if (__Master_ID > IMU_DM_STANDARD_ID_MAX)
    {
        return (false);
    }

    Master_ID = __Master_ID;
    Pending_Master_ID_Valid = false;
    return (true);
}

/**
 * @brief 发送 CAN 命令
 */
uint8_t Class_IMU_DM::Send_Command(const Enum_IMU_DM_Register &Register,
                                   const Enum_IMU_DM_Access &Access,
                                   const uint32_t &Data)
{
    if (CAN_Handler == nullptr)
    {
        return (static_cast<uint8_t>(HAL_ERROR));
    }

    uint8_t tx_data[8] = {
        IMU_DM_COMMAND_HEADER,
        static_cast<uint8_t>(Register),
        static_cast<uint8_t>(Access),
        IMU_DM_COMMAND_SEPARATOR,
        0,
        0,
        0,
        0,
    };
    IMU_DM_Write_LE_U32(&tx_data[4], Data);

    return (CAN_Transmit_Data(CAN_Handler,
                              CAN_ID,
                              tx_data,
                              FDCAN_DLC_BYTES_8));
}

/**
 * @brief 读取寄存器
 */
uint8_t Class_IMU_DM::CAN_Send_Read_Register(const Enum_IMU_DM_Register &Register)
{
    return (Send_Command(Register, IMU_DM_Access_READ, 0));
}

/**
 * @brief 写寄存器
 */
uint8_t Class_IMU_DM::CAN_Send_Write_Register(const Enum_IMU_DM_Register &Register,
                                              const uint32_t &Data)
{
    return (Send_Command(Register, IMU_DM_Access_WRITE, Data));
}

uint8_t Class_IMU_DM::CAN_Send_Request_Acceleration()
{
    return (CAN_Send_Read_Register(IMU_DM_Register_ACCELERATION));
}

uint8_t Class_IMU_DM::CAN_Send_Request_Gyroscope()
{
    return (CAN_Send_Read_Register(IMU_DM_Register_GYROSCOPE));
}

uint8_t Class_IMU_DM::CAN_Send_Request_Euler_Angle()
{
    return (CAN_Send_Read_Register(IMU_DM_Register_EULER_ANGLE));
}

uint8_t Class_IMU_DM::CAN_Send_Request_Quaternion()
{
    return (CAN_Send_Read_Register(IMU_DM_Register_QUATERNION));
}

uint8_t Class_IMU_DM::CAN_Send_Reboot()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_REBOOT, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Set_Zero()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_SET_ZERO, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Accelerometer_Calibration()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_ACCELEROMETER_CALIBRATION, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Gyroscope_Calibration()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_GYROSCOPE_CALIBRATION, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Magnetometer_Calibration()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_MAGNETOMETER_CALIBRATION, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Set_Active_Mode_Interval(const uint32_t &Interval)
{
    return (CAN_Send_Write_Register(IMU_DM_Register_ACTIVE_MODE_INTERVAL, Interval));
}

uint8_t Class_IMU_DM::CAN_Send_Set_Output_Mode(const Enum_IMU_DM_Output_Mode &Output_Mode)
{
    return (CAN_Send_Write_Register(IMU_DM_Register_OUTPUT_MODE,
                                    static_cast<uint32_t>(Output_Mode)));
}

uint8_t Class_IMU_DM::CAN_Send_Set_Baudrate(const Enum_IMU_DM_CAN_Baudrate &Baudrate)
{
    return (CAN_Send_Write_Register(IMU_DM_Register_CAN_BAUDRATE,
                                    static_cast<uint32_t>(Baudrate)));
}

uint8_t Class_IMU_DM::CAN_Send_Set_CAN_ID(const uint16_t &New_CAN_ID)
{
    if (New_CAN_ID > IMU_DM_STANDARD_ID_MAX)
    {
        return (static_cast<uint8_t>(HAL_ERROR));
    }

    const uint8_t result = CAN_Send_Write_Register(IMU_DM_Register_CAN_ID, New_CAN_ID);
    if (result == static_cast<uint8_t>(HAL_OK))
    {
        Pending_CAN_ID = New_CAN_ID;
        Pending_CAN_ID_Valid = true;
    }
    return (result);
}

uint8_t Class_IMU_DM::CAN_Send_Set_Master_ID(const uint16_t &New_Master_ID)
{
    if (New_Master_ID > IMU_DM_STANDARD_ID_MAX)
    {
        return (static_cast<uint8_t>(HAL_ERROR));
    }

    const uint8_t result = CAN_Send_Write_Register(IMU_DM_Register_MASTER_ID, New_Master_ID);
    if (result == static_cast<uint8_t>(HAL_OK))
    {
        Pending_Master_ID = New_Master_ID;
        Pending_Master_ID_Valid = true;
    }
    return (result);
}

uint8_t Class_IMU_DM::CAN_Send_Save_Parameters()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_SAVE_PARAMETERS, 0));
}

uint8_t Class_IMU_DM::CAN_Send_Restore_Settings()
{
    return (CAN_Send_Write_Register(IMU_DM_Register_RESTORE_SETTINGS, 0));
}

/**
 * @brief 使用 CAN 管理对象中的最新数据进行解析
 */
void Class_IMU_DM::CAN_RxCpltCallback()
{
    if (CAN_Manage_Object == nullptr)
    {
        return;
    }

    CAN_RxCpltCallback(CAN_Manage_Object->Rx_Header,
                       CAN_Manage_Object->Rx_Buffer);
}

/**
 * @brief 使用外部分发的 CAN 帧进行解析
 */
void Class_IMU_DM::CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header,
                                      const uint8_t *Buffer)
{
    const bool master_id_matches = (Header.Identifier == Master_ID);
    const bool pending_master_id_matches =
        Pending_Master_ID_Valid && (Header.Identifier == Pending_Master_ID);

    if (!master_id_matches && !pending_master_id_matches)
    {
        return;
    }

    if ((Buffer == nullptr) ||
        (Header.IdType != FDCAN_STANDARD_ID) ||
        (Header.RxFrameType != FDCAN_DATA_FRAME) ||
        (Header.DataLength != FDCAN_DLC_BYTES_8))
    {
        ++Parser_Statistics.Invalid_Frame_Count;
        return;
    }

    const Enum_IMU_DM_Rx_Packet_Type packet_type = Data_Process(Buffer);
    if (packet_type == IMU_DM_Rx_Packet_Type_NONE)
    {
        ++Parser_Statistics.Invalid_Frame_Count;
        return;
    }

    // 修改 Master ID 后，部分固件可能直接使用新 ID 返回第一帧传感器数据。
    if (pending_master_id_matches &&
        (packet_type != IMU_DM_Rx_Packet_Type_RESPONSE))
    {
        Master_ID = Pending_Master_ID;
        Pending_Master_ID_Valid = false;
    }

    Mark_Valid_Packet(packet_type);
}

/**
 * @brief 按首字节区分传感器数据和命令应答
 */
Enum_IMU_DM_Rx_Packet_Type Class_IMU_DM::Data_Process(const uint8_t *Buffer)
{
    switch (Buffer[0])
    {
    case IMU_DM_Register_ACCELERATION:
        Parse_Acceleration(Buffer);
        return (IMU_DM_Rx_Packet_Type_ACCELERATION);

    case IMU_DM_Register_GYROSCOPE:
        Parse_Gyroscope(Buffer);
        return (IMU_DM_Rx_Packet_Type_GYROSCOPE);

    case IMU_DM_Register_EULER_ANGLE:
        Parse_Euler_Angle(Buffer);
        return (IMU_DM_Rx_Packet_Type_EULER_ANGLE);

    case IMU_DM_Register_QUATERNION:
        Parse_Quaternion(Buffer);
        return (IMU_DM_Rx_Packet_Type_QUATERNION);

    case IMU_DM_COMMAND_HEADER:
        if (Parse_Response(Buffer))
        {
            return (IMU_DM_Rx_Packet_Type_RESPONSE);
        }
        break;

    default:
        break;
    }

    return (IMU_DM_Rx_Packet_Type_NONE);
}

/**
 * @brief 解析加速度和温度
 */
void Class_IMU_DM::Parse_Acceleration(const uint8_t *Buffer)
{
    Acceleration_Data.Temperature_Celsius =
        Uint_To_Float(Buffer[1],
                      IMU_DM_TEMPERATURE_MIN,
                      IMU_DM_TEMPERATURE_MAX,
                      8);
    Acceleration_Data.X =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[2]),
                      IMU_DM_ACCELERATION_MIN,
                      IMU_DM_ACCELERATION_MAX,
                      16);
    Acceleration_Data.Y =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[4]),
                      IMU_DM_ACCELERATION_MIN,
                      IMU_DM_ACCELERATION_MAX,
                      16);
    Acceleration_Data.Z =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[6]),
                      IMU_DM_ACCELERATION_MIN,
                      IMU_DM_ACCELERATION_MAX,
                      16);
    Acceleration_Data.Valid = true;
    ++Acceleration_Data.Sequence;
    ++Parser_Statistics.Valid_Acceleration_Count;
}

/**
 * @brief 解析角速度
 */
void Class_IMU_DM::Parse_Gyroscope(const uint8_t *Buffer)
{
    Gyroscope_Data.X =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[2]),
                      IMU_DM_GYROSCOPE_MIN,
                      IMU_DM_GYROSCOPE_MAX,
                      16);
    Gyroscope_Data.Y =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[4]),
                      IMU_DM_GYROSCOPE_MIN,
                      IMU_DM_GYROSCOPE_MAX,
                      16);
    Gyroscope_Data.Z =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[6]),
                      IMU_DM_GYROSCOPE_MIN,
                      IMU_DM_GYROSCOPE_MAX,
                      16);
    Gyroscope_Data.Valid = true;
    ++Gyroscope_Data.Sequence;
    ++Parser_Statistics.Valid_Gyroscope_Count;
}

/**
 * @brief 解析欧拉角
 */
void Class_IMU_DM::Parse_Euler_Angle(const uint8_t *Buffer)
{
    Euler_Angle_Data.Pitch =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[2]),
                      IMU_DM_PITCH_MIN,
                      IMU_DM_PITCH_MAX,
                      16);
    Euler_Angle_Data.Yaw =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[4]),
                      IMU_DM_YAW_MIN,
                      IMU_DM_YAW_MAX,
                      16);
    Euler_Angle_Data.Roll =
        Uint_To_Float(IMU_DM_Read_LE_U16(&Buffer[6]),
                      IMU_DM_ROLL_MIN,
                      IMU_DM_ROLL_MAX,
                      16);
    Euler_Angle_Data.Valid = true;
    ++Euler_Angle_Data.Sequence;
    ++Parser_Statistics.Valid_Euler_Angle_Count;
}

/**
 * @brief 解析四元数
 *
 * @note 每个分量为 14 位。手册位域表中 W 的低 6 位位于 Buffer[2] 的 bit7..2，
 *       因此掩码应为 0xfc。
 */
void Class_IMU_DM::Parse_Quaternion(const uint8_t *Buffer)
{
    const uint16_t w =
        (static_cast<uint16_t>(Buffer[1]) << 6) |
        ((static_cast<uint16_t>(Buffer[2]) & 0xfcU) >> 2);
    const uint16_t x =
        ((static_cast<uint16_t>(Buffer[2]) & 0x03U) << 12) |
        (static_cast<uint16_t>(Buffer[3]) << 4) |
        ((static_cast<uint16_t>(Buffer[4]) & 0xf0U) >> 4);
    const uint16_t y =
        ((static_cast<uint16_t>(Buffer[4]) & 0x0fU) << 10) |
        (static_cast<uint16_t>(Buffer[5]) << 2) |
        ((static_cast<uint16_t>(Buffer[6]) & 0xc0U) >> 6);
    const uint16_t z =
        ((static_cast<uint16_t>(Buffer[6]) & 0x3fU) << 8) |
        static_cast<uint16_t>(Buffer[7]);

    Quaternion_Data.W =
        Uint_To_Float(w, IMU_DM_QUATERNION_MIN, IMU_DM_QUATERNION_MAX, 14);
    Quaternion_Data.X =
        Uint_To_Float(x, IMU_DM_QUATERNION_MIN, IMU_DM_QUATERNION_MAX, 14);
    Quaternion_Data.Y =
        Uint_To_Float(y, IMU_DM_QUATERNION_MIN, IMU_DM_QUATERNION_MAX, 14);
    Quaternion_Data.Z =
        Uint_To_Float(z, IMU_DM_QUATERNION_MIN, IMU_DM_QUATERNION_MAX, 14);
    Quaternion_Data.Valid = true;
    ++Quaternion_Data.Sequence;
    ++Parser_Statistics.Valid_Quaternion_Count;
}

/**
 * @brief 解析命令应答
 */
bool Class_IMU_DM::Parse_Response(const uint8_t *Buffer)
{
    if ((Buffer[2] != IMU_DM_COMMAND_SEPARATOR) ||
        (Buffer[3] > IMU_DM_Ack_OPERATION_FAILED))
    {
        return (false);
    }

    Response_Data.Valid = true;
    Response_Data.Register = Buffer[1];
    Response_Data.Ack = static_cast<Enum_IMU_DM_Ack>(Buffer[3]);
    Response_Data.Data = IMU_DM_Read_LE_U32(&Buffer[4]);
    ++Response_Data.Sequence;
    ++Parser_Statistics.Valid_Response_Count;

    if (Buffer[1] == IMU_DM_Register_CAN_ID)
    {
        if ((Response_Data.Ack == IMU_DM_Ack_SUCCESS) &&
            Pending_CAN_ID_Valid)
        {
            CAN_ID = Pending_CAN_ID;
        }
        Pending_CAN_ID_Valid = false;
    }
    else if (Buffer[1] == IMU_DM_Register_MASTER_ID)
    {
        if ((Response_Data.Ack == IMU_DM_Ack_SUCCESS) &&
            Pending_Master_ID_Valid)
        {
            Master_ID = Pending_Master_ID;
        }
        Pending_Master_ID_Valid = false;
    }

    return (true);
}

/**
 * @brief 标记收到有效帧
 */
void Class_IMU_DM::Mark_Valid_Packet(const Enum_IMU_DM_Rx_Packet_Type &Packet_Type)
{
    ++Flag;
    IMU_Status = IMU_DM_Status_ENABLE;
    Offline_Countdown_100ms = Offline_Timeout_100ms;
    Last_Rx_Packet_Type = Packet_Type;

    if (CAN_Manage_Object != nullptr)
    {
        Last_Rx_Timestamp = CAN_Manage_Object->Rx_Timestamp;
    }
}

/**
 * @brief 每 100 ms 检测一次在线状态
 */
void Class_IMU_DM::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag != Pre_Flag)
    {
        IMU_Status = IMU_DM_Status_ENABLE;
        Offline_Countdown_100ms = Offline_Timeout_100ms;
    }
    else if (Offline_Countdown_100ms != 0U)
    {
        --Offline_Countdown_100ms;
    }

    if (Offline_Countdown_100ms == 0U)
    {
        IMU_Status = IMU_DM_Status_DISABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief 将无符号映射值还原为浮点数
 */
float Class_IMU_DM::Uint_To_Float(const uint32_t &Value,
                                  const float &Minimum,
                                  const float &Maximum,
                                  const uint8_t &Bits)
{
    const uint32_t integer_max = (1UL << Bits) - 1UL;
    return (static_cast<float>(Value) *
                (Maximum - Minimum) /
                static_cast<float>(integer_max) +
            Minimum);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
