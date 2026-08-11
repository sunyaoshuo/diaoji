/**
 * @file dvc_imu_dm.h
 * @author Project integration
 * @brief 达妙 DM-IMU-L1 的 CAN 驱动
 * @version 0.1
 * @date 2026-07-29 0.1 根据 DM-IMU-L1 V1.2 手册和官方 MC02 CAN 例程新建
 *
 * @note
 * 1. 本驱动只实现经典 CAN 通信，不包含 USB、UART 或 RS485。
 * 2. 默认 CAN 波特率为 1 Mbps，使用 11 位标准数据帧，DLC 固定为 8。
 * 3. 请求帧发送到 CAN_ID，传感器数据和命令应答从 Master_ID 返回。
 * 4. 加速度单位为 m/s^2，角速度单位为 rad/s，欧拉角单位为 degree。
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#ifndef DVC_IMU_DM_H
#define DVC_IMU_DM_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define IMU_DM_ACCELERATION_MIN (-235.2f)
#define IMU_DM_ACCELERATION_MAX (235.2f)
#define IMU_DM_GYROSCOPE_MIN (-34.88f)
#define IMU_DM_GYROSCOPE_MAX (34.88f)
#define IMU_DM_PITCH_MIN (-90.0f)
#define IMU_DM_PITCH_MAX (90.0f)
#define IMU_DM_YAW_MIN (-180.0f)
#define IMU_DM_YAW_MAX (180.0f)
#define IMU_DM_ROLL_MIN (-180.0f)
#define IMU_DM_ROLL_MAX (180.0f)
#define IMU_DM_QUATERNION_MIN (-1.0f)
#define IMU_DM_QUATERNION_MAX (1.0f)
#define IMU_DM_TEMPERATURE_MIN (0.0f)
#define IMU_DM_TEMPERATURE_MAX (60.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU 在线状态
 */
enum Enum_IMU_DM_Status : uint8_t
{
    IMU_DM_Status_DISABLE = 0,
    IMU_DM_Status_ENABLE,
};

/**
 * @brief CAN 寄存器
 */
enum Enum_IMU_DM_Register : uint8_t
{
    IMU_DM_Register_REBOOT = 0x00,
    IMU_DM_Register_ACCELERATION = 0x01,
    IMU_DM_Register_GYROSCOPE = 0x02,
    IMU_DM_Register_EULER_ANGLE = 0x03,
    IMU_DM_Register_QUATERNION = 0x04,
    IMU_DM_Register_SET_ZERO = 0x05,
    IMU_DM_Register_ACCELEROMETER_CALIBRATION = 0x06,
    IMU_DM_Register_GYROSCOPE_CALIBRATION = 0x07,
    IMU_DM_Register_MAGNETOMETER_CALIBRATION = 0x08,
    IMU_DM_Register_COMMUNICATION_PORT = 0x09,
    IMU_DM_Register_ACTIVE_MODE_INTERVAL = 0x0a,
    IMU_DM_Register_OUTPUT_MODE = 0x0b,
    IMU_DM_Register_CAN_BAUDRATE = 0x0c,
    IMU_DM_Register_CAN_ID = 0x0d,
    IMU_DM_Register_MASTER_ID = 0x0e,
    IMU_DM_Register_OUTPUT_SELECTION = 0x0f,
    IMU_DM_Register_SAVE_PARAMETERS = 0xfe,
    IMU_DM_Register_RESTORE_SETTINGS = 0xff,
};

/**
 * @brief CAN 命令访问类型
 */
enum Enum_IMU_DM_Access : uint8_t
{
    IMU_DM_Access_READ = 0x00,
    IMU_DM_Access_WRITE = 0x01,
};

/**
 * @brief CAN 输出模式
 */
enum Enum_IMU_DM_Output_Mode : uint8_t
{
    IMU_DM_Output_Mode_REQUEST = 0x00,
    IMU_DM_Output_Mode_ACTIVE = 0x01,
};

/**
 * @brief CAN 波特率配置值
 */
enum Enum_IMU_DM_CAN_Baudrate : uint8_t
{
    IMU_DM_CAN_Baudrate_1M = 0x00,
    IMU_DM_CAN_Baudrate_500K = 0x01,
    IMU_DM_CAN_Baudrate_400K = 0x02,
    IMU_DM_CAN_Baudrate_250K = 0x03,
    IMU_DM_CAN_Baudrate_200K = 0x04,
    IMU_DM_CAN_Baudrate_100K = 0x05,
    IMU_DM_CAN_Baudrate_50K = 0x06,
    IMU_DM_CAN_Baudrate_25K = 0x07,
};

/**
 * @brief 命令应答码
 */
enum Enum_IMU_DM_Ack : uint8_t
{
    IMU_DM_Ack_SUCCESS = 0x00,
    IMU_DM_Ack_REGISTER_NOT_FOUND = 0x01,
    IMU_DM_Ack_INVALID_DATA = 0x02,
    IMU_DM_Ack_OPERATION_FAILED = 0x03,
    IMU_DM_Ack_NONE = 0xff,
};

/**
 * @brief 最近一次有效接收帧类型
 */
enum Enum_IMU_DM_Rx_Packet_Type : uint8_t
{
    IMU_DM_Rx_Packet_Type_NONE = 0,
    IMU_DM_Rx_Packet_Type_ACCELERATION,
    IMU_DM_Rx_Packet_Type_GYROSCOPE,
    IMU_DM_Rx_Packet_Type_EULER_ANGLE,
    IMU_DM_Rx_Packet_Type_QUATERNION,
    IMU_DM_Rx_Packet_Type_RESPONSE,
};

/**
 * @brief 三轴加速度数据，单位 m/s^2
 */
struct Struct_IMU_DM_Acceleration_Data
{
    bool Valid;
    float X;
    float Y;
    float Z;
    float Temperature_Celsius;
    uint32_t Sequence;
};

/**
 * @brief 三轴角速度数据，单位 rad/s
 */
struct Struct_IMU_DM_Gyroscope_Data
{
    bool Valid;
    float X;
    float Y;
    float Z;
    uint32_t Sequence;
};

/**
 * @brief 欧拉角数据，单位 degree
 */
struct Struct_IMU_DM_Euler_Angle_Data
{
    bool Valid;
    float Pitch;
    float Yaw;
    float Roll;
    uint32_t Sequence;
};

/**
 * @brief 四元数数据，顺序为 W、X、Y、Z
 */
struct Struct_IMU_DM_Quaternion_Data
{
    bool Valid;
    float W;
    float X;
    float Y;
    float Z;
    uint32_t Sequence;
};

/**
 * @brief 最近一次命令应答
 */
struct Struct_IMU_DM_Response_Data
{
    bool Valid;
    uint8_t Register;
    Enum_IMU_DM_Ack Ack;
    uint32_t Data;
    uint32_t Sequence;
};

/**
 * @brief 接收解析统计
 */
struct Struct_IMU_DM_Parser_Statistics
{
    uint32_t Valid_Acceleration_Count;
    uint32_t Valid_Gyroscope_Count;
    uint32_t Valid_Euler_Angle_Count;
    uint32_t Valid_Quaternion_Count;
    uint32_t Valid_Response_Count;
    uint32_t Invalid_Frame_Count;
};

/**
 * @brief Reusable，达妙 DM-IMU-L1 CAN 驱动
 */
class Class_IMU_DM
{
public:
    /**
     * @brief 初始化 IMU 对象
     *
     * @param hcan 绑定的 FDCAN
     * @param __CAN_ID IMU 接收请求所使用的标准帧 ID
     * @param __Master_ID IMU 返回数据所使用的标准帧 ID
     * @param __Offline_Timeout_100ms 连续多少个 100 ms 周期未收到有效帧后判定离线
     */
    void Init(const FDCAN_HandleTypeDef *hcan,
              const uint16_t &__CAN_ID = 0x01,
              const uint16_t &__Master_ID = 0x11,
              const uint16_t &__Offline_Timeout_100ms = 5);

    inline Enum_IMU_DM_Status Get_Status() const;
    inline uint16_t Get_CAN_ID() const;
    inline uint16_t Get_Master_ID() const;
    inline uint64_t Get_Last_Rx_Timestamp() const;
    inline Enum_IMU_DM_Rx_Packet_Type Get_Last_Rx_Packet_Type() const;

    inline const Struct_IMU_DM_Acceleration_Data &Get_Acceleration_Data() const;
    inline const Struct_IMU_DM_Gyroscope_Data &Get_Gyroscope_Data() const;
    inline const Struct_IMU_DM_Euler_Angle_Data &Get_Euler_Angle_Data() const;
    inline const Struct_IMU_DM_Quaternion_Data &Get_Quaternion_Data() const;
    inline const Struct_IMU_DM_Response_Data &Get_Response_Data() const;
    inline const Struct_IMU_DM_Parser_Statistics &Get_Parser_Statistics() const;

    /**
     * @brief 只修改驱动本地使用的 CAN ID，不向 IMU 发送配置命令
     */
    bool Set_Local_CAN_ID(const uint16_t &__CAN_ID);

    /**
     * @brief 只修改驱动本地使用的 Master ID，不向 IMU 发送配置命令
     */
    bool Set_Local_Master_ID(const uint16_t &__Master_ID);

    uint8_t CAN_Send_Read_Register(const Enum_IMU_DM_Register &Register);
    uint8_t CAN_Send_Write_Register(const Enum_IMU_DM_Register &Register, const uint32_t &Data);

    uint8_t CAN_Send_Request_Acceleration();
    uint8_t CAN_Send_Request_Gyroscope();
    uint8_t CAN_Send_Request_Euler_Angle();
    uint8_t CAN_Send_Request_Quaternion();

    uint8_t CAN_Send_Reboot();
    uint8_t CAN_Send_Set_Zero();
    uint8_t CAN_Send_Accelerometer_Calibration();
    uint8_t CAN_Send_Gyroscope_Calibration();
    uint8_t CAN_Send_Magnetometer_Calibration();
    uint8_t CAN_Send_Set_Active_Mode_Interval(const uint32_t &Interval);
    uint8_t CAN_Send_Set_Output_Mode(const Enum_IMU_DM_Output_Mode &Output_Mode);
    uint8_t CAN_Send_Set_Baudrate(const Enum_IMU_DM_CAN_Baudrate &Baudrate);
    uint8_t CAN_Send_Set_CAN_ID(const uint16_t &New_CAN_ID);
    uint8_t CAN_Send_Set_Master_ID(const uint16_t &New_Master_ID);
    uint8_t CAN_Send_Save_Parameters();
    uint8_t CAN_Send_Restore_Settings();

    /**
     * @brief 使用 CAN 驱动管理对象中的最新帧进行解析
     */
    void CAN_RxCpltCallback();

    /**
     * @brief 使用外部分发的帧进行解析，推荐在多设备共用 CAN 回调时使用
     */
    void CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header, const uint8_t *Buffer);

    /**
     * @brief 每 100 ms 调用一次的在线检测
     */
    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    FDCAN_HandleTypeDef *CAN_Handler = nullptr;

    uint16_t CAN_ID = 0x01;
    uint16_t Master_ID = 0x11;
    uint16_t Offline_Timeout_100ms = 5;
    uint16_t Offline_Countdown_100ms = 0;

    uint16_t Pending_CAN_ID = 0;
    uint16_t Pending_Master_ID = 0;
    bool Pending_CAN_ID_Valid = false;
    bool Pending_Master_ID_Valid = false;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Enum_IMU_DM_Status IMU_Status = IMU_DM_Status_DISABLE;
    Enum_IMU_DM_Rx_Packet_Type Last_Rx_Packet_Type = IMU_DM_Rx_Packet_Type_NONE;
    uint64_t Last_Rx_Timestamp = 0;

    Struct_IMU_DM_Acceleration_Data Acceleration_Data = {};
    Struct_IMU_DM_Gyroscope_Data Gyroscope_Data = {};
    Struct_IMU_DM_Euler_Angle_Data Euler_Angle_Data = {};
    Struct_IMU_DM_Quaternion_Data Quaternion_Data = {};
    Struct_IMU_DM_Response_Data Response_Data = {};
    Struct_IMU_DM_Parser_Statistics Parser_Statistics = {};

    uint8_t Send_Command(const Enum_IMU_DM_Register &Register,
                         const Enum_IMU_DM_Access &Access,
                         const uint32_t &Data);

    Enum_IMU_DM_Rx_Packet_Type Data_Process(const uint8_t *Buffer);
    void Parse_Acceleration(const uint8_t *Buffer);
    void Parse_Gyroscope(const uint8_t *Buffer);
    void Parse_Euler_Angle(const uint8_t *Buffer);
    void Parse_Quaternion(const uint8_t *Buffer);
    bool Parse_Response(const uint8_t *Buffer);
    void Mark_Valid_Packet(const Enum_IMU_DM_Rx_Packet_Type &Packet_Type);

    static float Uint_To_Float(const uint32_t &Value,
                               const float &Minimum,
                               const float &Maximum,
                               const uint8_t &Bits);
};

/* Exported function declarations --------------------------------------------*/

inline Enum_IMU_DM_Status Class_IMU_DM::Get_Status() const
{
    return (IMU_Status);
}

inline uint16_t Class_IMU_DM::Get_CAN_ID() const
{
    return (CAN_ID);
}

inline uint16_t Class_IMU_DM::Get_Master_ID() const
{
    return (Master_ID);
}

inline uint64_t Class_IMU_DM::Get_Last_Rx_Timestamp() const
{
    return (Last_Rx_Timestamp);
}

inline Enum_IMU_DM_Rx_Packet_Type Class_IMU_DM::Get_Last_Rx_Packet_Type() const
{
    return (Last_Rx_Packet_Type);
}

inline const Struct_IMU_DM_Acceleration_Data &Class_IMU_DM::Get_Acceleration_Data() const
{
    return (Acceleration_Data);
}

inline const Struct_IMU_DM_Gyroscope_Data &Class_IMU_DM::Get_Gyroscope_Data() const
{
    return (Gyroscope_Data);
}

inline const Struct_IMU_DM_Euler_Angle_Data &Class_IMU_DM::Get_Euler_Angle_Data() const
{
    return (Euler_Angle_Data);
}

inline const Struct_IMU_DM_Quaternion_Data &Class_IMU_DM::Get_Quaternion_Data() const
{
    return (Quaternion_Data);
}

inline const Struct_IMU_DM_Response_Data &Class_IMU_DM::Get_Response_Data() const
{
    return (Response_Data);
}

inline const Struct_IMU_DM_Parser_Statistics &Class_IMU_DM::Get_Parser_Statistics() const
{
    return (Parser_Statistics);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
