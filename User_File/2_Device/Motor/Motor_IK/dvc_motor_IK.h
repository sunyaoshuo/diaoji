/**
 * @file dvc_motor_IK.h
 * @author Project integration
 * @brief 上海瓴控/KMTECH 电机 CAN 通信驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《电机 CAN 总线通讯协议 V2.36》新增
 *
 * @note
 * 1. 单电机标准帧 ID = 0x140 + Motor_ID，Motor_ID 范围 1~32，DLC 固定为 8。
 * 2. 对外物理量统一使用：角度 rad、角速度 rad/s、电流 A、电压 V、温度 K。
 * 3. 协议规定角度和速度正方向为顺时针，本驱动保持该符号定义，不做方向翻转。
 * 4. MS 系列状态 2 的 DATA[2:3] 是输出功率原始值；MF/MH/MHF/MG 系列是转矩电流。
 * 5. 协议只明确给出 MF 与 MG 的电流分辨率；MH/MHF 默认沿用 MF 的 33/4096 A/LSB，
 *    可通过 Init() 的 __Current_Resolution 参数覆盖。
 */

#ifndef DVC_MOTOR_IK_H
#define DVC_MOTOR_IK_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机通信在线状态
 */
enum Enum_Motor_IK_Status : uint8_t
{
    Motor_IK_Status_DISABLE = 0,
    Motor_IK_Status_ENABLE,
};

/**
 * @brief 电机系列，用于选择电流反馈/控制换算系数
 */
enum Enum_Motor_IK_Series : uint8_t
{
    Motor_IK_Series_MF = 0,
    Motor_IK_Series_MH,
    Motor_IK_Series_MHF,
    Motor_IK_Series_MG,
    Motor_IK_Series_MS,
};

/**
 * @brief 状态 1 中的电机开启/关闭状态
 */
enum Enum_Motor_IK_Motor_State : uint8_t
{
    Motor_IK_Motor_State_ENABLE = 0x00,
    Motor_IK_Motor_State_DISABLE = 0x10,
    Motor_IK_Motor_State_UNKNOWN = 0xff,
};

/**
 * @brief 错误标志位，可通过 Get_Error_Flag() 查询
 */
enum Enum_Motor_IK_Error_Flag : uint8_t
{
    Motor_IK_Error_NONE = 0x00,
    Motor_IK_Error_UNDERVOLTAGE = 0x01,
    Motor_IK_Error_OVERVOLTAGE = 0x02,
    Motor_IK_Error_DRIVER_OVERTEMPERATURE = 0x04,
    Motor_IK_Error_MOTOR_OVERTEMPERATURE = 0x08,
    Motor_IK_Error_OVERCURRENT = 0x10,
    Motor_IK_Error_SHORT_CIRCUIT = 0x20,
    Motor_IK_Error_STALL = 0x40,
    Motor_IK_Error_INPUT_TIMEOUT = 0x80,
};

/**
 * @brief 抱闸器命令
 */
enum Enum_Motor_IK_Brake_Command : uint8_t
{
    Motor_IK_Brake_Command_POWER_OFF = 0x00,
    Motor_IK_Brake_Command_POWER_ON = 0x01,
    Motor_IK_Brake_Command_READ = 0x10,
};

/**
 * @brief 抱闸器反馈状态
 */
enum Enum_Motor_IK_Brake_Status : uint8_t
{
    Motor_IK_Brake_Status_POWER_OFF = 0x00,
    Motor_IK_Brake_Status_POWER_ON = 0x01,
    Motor_IK_Brake_Status_UNKNOWN = 0xff,
};

/**
 * @brief 单圈位置控制方向；协议规定 0 为顺时针、1 为逆时针
 */
enum Enum_Motor_IK_Spin_Direction : uint8_t
{
    Motor_IK_Spin_Direction_CLOCKWISE = 0x00,
    Motor_IK_Spin_Direction_COUNTERCLOCKWISE = 0x01,
};

/**
 * @brief 周期发送时使用的控制模式
 */
enum Enum_Motor_IK_Control_Method : uint8_t
{
    Motor_IK_Control_Method_NONE = 0,
    Motor_IK_Control_Method_OPEN_LOOP,
    Motor_IK_Control_Method_TORQUE,
    Motor_IK_Control_Method_SPEED,
    Motor_IK_Control_Method_MULTI_TURN_ANGLE_1,
    Motor_IK_Control_Method_MULTI_TURN_ANGLE_2,
    Motor_IK_Control_Method_SINGLE_TURN_ANGLE_1,
    Motor_IK_Control_Method_SINGLE_TURN_ANGLE_2,
    Motor_IK_Control_Method_INCREMENT_ANGLE_1,
    Motor_IK_Control_Method_INCREMENT_ANGLE_2,
};

/**
 * @brief 协议命令字
 */
enum Enum_Motor_IK_Command : uint8_t
{
    Motor_IK_Command_RESTART = 0x07,
    Motor_IK_Command_CALIBRATE_ENCODER = 0x18,
    Motor_IK_Command_SET_ZERO_ROM = 0x19,
    Motor_IK_Command_READ_SETTING_PARAMETER = 0x40,
    Motor_IK_Command_WRITE_SETTING_PARAMETER = 0x42,
    Motor_IK_Command_SAVE_SETTING_PARAMETER = 0x44,
    Motor_IK_Command_SHUTDOWN = 0x80,
    Motor_IK_Command_STOP = 0x81,
    Motor_IK_Command_BRAKE = 0x8c,
    Motor_IK_Command_RUN = 0x88,
    Motor_IK_Command_READ_ENCODER = 0x90,
    Motor_IK_Command_READ_MULTI_TURN_ANGLE = 0x92,
    Motor_IK_Command_READ_SINGLE_TURN_ANGLE = 0x94,
    Motor_IK_Command_SET_ZERO_RAM = 0x95,
    Motor_IK_Command_READ_STATUS_1 = 0x9a,
    Motor_IK_Command_CLEAR_ERROR = 0x9b,
    Motor_IK_Command_READ_STATUS_2 = 0x9c,
    Motor_IK_Command_READ_STATUS_3 = 0x9d,
    Motor_IK_Command_OPEN_LOOP = 0xa0,
    Motor_IK_Command_TORQUE = 0xa1,
    Motor_IK_Command_SPEED = 0xa2,
    Motor_IK_Command_MULTI_TURN_ANGLE_1 = 0xa3,
    Motor_IK_Command_MULTI_TURN_ANGLE_2 = 0xa4,
    Motor_IK_Command_SINGLE_TURN_ANGLE_1 = 0xa5,
    Motor_IK_Command_SINGLE_TURN_ANGLE_2 = 0xa6,
    Motor_IK_Command_INCREMENT_ANGLE_1 = 0xa7,
    Motor_IK_Command_INCREMENT_ANGLE_2 = 0xa8,
    Motor_IK_Command_READ_CONTROL_PARAMETER = 0xc0,
    Motor_IK_Command_WRITE_CONTROL_PARAMETER = 0xc1,
};

/**
 * @brief RAM 控制参数序号
 */
enum Enum_Motor_IK_Control_Parameter_ID : uint8_t
{
    Motor_IK_Control_Parameter_POSITION_PID = 0x0a,
    Motor_IK_Control_Parameter_SPEED_PID = 0x0b,
    Motor_IK_Control_Parameter_CURRENT_PID = 0x0c,
    Motor_IK_Control_Parameter_TORQUE_LIMIT = 0x1e,
    Motor_IK_Control_Parameter_SPEED_LIMIT = 0x20,
    Motor_IK_Control_Parameter_ANGLE_LIMIT = 0x22,
    Motor_IK_Control_Parameter_CURRENT_RAMP = 0x24,
    Motor_IK_Control_Parameter_SPEED_RAMP = 0x26,
};

/**
 * @brief 单个设定参数的二级编号，DATA[1] 固定为 0x05
 */
enum Enum_Motor_IK_Setting_Parameter_ID : uint8_t
{
    Motor_IK_Setting_Parameter_DRIVER_ID = 0x0a,
    Motor_IK_Setting_Parameter_BUS_TYPE = 0x0b,
    Motor_IK_Setting_Parameter_RS485_BAUDRATE = 0x0c,
    Motor_IK_Setting_Parameter_CAN_BAUDRATE = 0x0d,
    Motor_IK_Setting_Parameter_MAX_POWER = 0xe0,
    Motor_IK_Setting_Parameter_MAX_SPEED = 0xe2,
    Motor_IK_Setting_Parameter_MAX_ANGLE = 0xe4,
    Motor_IK_Setting_Parameter_CURRENT_RAMP = 0xea,
    Motor_IK_Setting_Parameter_SPEED_RAMP = 0xec,
};

/**
 * @brief 多参数 PID 设定编号，直接放在 DATA[1]
 */
enum Enum_Motor_IK_Setting_PID_ID : uint8_t
{
    Motor_IK_Setting_PID_POSITION = 0xa0,
    Motor_IK_Setting_PID_SPEED = 0xa4,
    Motor_IK_Setting_PID_CURRENT = 0xa8,
};

/**
 * @brief 总线类型设定值
 */
enum Enum_Motor_IK_Bus_Type : uint8_t
{
    Motor_IK_Bus_Type_NONE = 0,
    Motor_IK_Bus_Type_RS485 = 1,
    Motor_IK_Bus_Type_CAN = 2,
};

/**
 * @brief CAN 波特率设定值
 */
enum Enum_Motor_IK_CAN_Baudrate : uint8_t
{
    Motor_IK_CAN_Baudrate_100K = 0,
    Motor_IK_CAN_Baudrate_125K = 1,
    Motor_IK_CAN_Baudrate_250K = 2,
    Motor_IK_CAN_Baudrate_500K = 3,
    Motor_IK_CAN_Baudrate_1M = 4,
};

/**
 * @brief PID 参数
 */
struct Struct_Motor_IK_PID
{
    uint16_t K_P;
    uint16_t K_I;
    uint16_t K_D;
};

/**
 * @brief 电机接收数据；物理量单位见文件头说明
 */
struct Struct_Motor_IK_Rx_Data
{
    uint8_t Last_Command;
    uint8_t Last_Raw_Data[8];

    int8_t Temperature_Raw;
    float Temperature;

    int16_t Bus_Voltage_Raw;
    float Bus_Voltage;
    int16_t Bus_Current_Raw;
    float Bus_Current;

    Enum_Motor_IK_Motor_State Motor_State;
    uint8_t Error_State;

    int16_t Torque_Current_Raw;
    float Torque_Current;
    int16_t Output_Power;

    int16_t Speed_Raw;
    float Now_Omega;

    uint16_t Encoder;
    uint16_t Encoder_Raw;
    uint16_t Encoder_Offset;
    uint32_t Encoder_Offset_ROM;
    float Encoder_Angle;

    int16_t Phase_Current_Raw[3];
    float Phase_Current[3];

    Enum_Motor_IK_Brake_Status Brake_Status;

    uint32_t Align_Value;
    uint16_t Align_Ratio;
    uint8_t Align_State;
    bool Align_Success;
    bool Align_Phase_Reversed;

    // 协议 DATA[1:7] 实际承载 56 位有符号角度，本驱动已进行符号扩展
    int64_t Multi_Turn_Angle_Raw;
    float Multi_Turn_Angle;

    uint32_t Single_Turn_Angle_Raw;
    float Single_Turn_Angle;

    bool Save_Setting_Result_Valid;
    bool Save_Setting_Success;
};

/**
 * @brief 最近一次控制参数（0xC0/0xC1）反馈
 */
struct Struct_Motor_IK_Control_Parameter_Data
{
    bool Valid;
    uint8_t Command;
    uint8_t ID;
    uint8_t Raw[6];
    Struct_Motor_IK_PID PID;
    int16_t Value_I16;
    int32_t Value_I32;
};

/**
 * @brief 最近一次设定参数（0x40/0x42）反馈
 */
struct Struct_Motor_IK_Setting_Parameter_Data
{
    bool Valid;
    uint8_t Command;
    uint8_t Parameter_1;
    uint8_t Parameter_2;
    uint8_t Raw[7];
    uint8_t Value_U8;
    int16_t Value_I16;
    int32_t Value_I32;
    Struct_Motor_IK_PID PID;
};

/**
 * @brief Reusable, 上海瓴控/KMTECH 单电机 CAN 驱动
 */
class Class_Motor_IK
{
public:
    /**
     * @brief 初始化电机对象
     *
     * @param hcan 绑定的 FDCAN
     * @param __Motor_ID 电机 ID，范围 1~32
     * @param __Motor_Series 电机系列，用于电流换算
     * @param __Encoder_Bits 编码器位数，协议支持 14/15/16 bit
     * @param __Current_Resolution 电流分辨率覆盖值，单位 A/LSB；0 表示按系列自动选择
     */
    void Init(const FDCAN_HandleTypeDef *hcan,
              const uint8_t &__Motor_ID = 1,
              const Enum_Motor_IK_Series &__Motor_Series = Motor_IK_Series_MF,
              const uint8_t &__Encoder_Bits = 14,
              const float &__Current_Resolution = 0.0f);

    inline Enum_Motor_IK_Status Get_Status() const;
    inline uint8_t Get_Motor_ID() const;
    inline uint16_t Get_CAN_ID() const;
    inline Enum_Motor_IK_Series Get_Motor_Series() const;
    inline float Get_Current_Resolution() const;

    inline Enum_Motor_IK_Motor_State Get_Motor_State() const;
    inline uint8_t Get_Error_State() const;
    inline bool Get_Error_Flag(const Enum_Motor_IK_Error_Flag &__Error_Flag) const;

    inline float Get_Now_Temperature() const;
    inline float Get_Now_Temperature_Celsius() const;
    inline float Get_Bus_Voltage() const;
    inline float Get_Bus_Current() const;
    inline float Get_Now_Current() const;
    inline int16_t Get_Now_Power() const;
    inline float Get_Now_Omega() const;
    inline uint16_t Get_Now_Encoder() const;
    inline float Get_Now_Encoder_Angle() const;
    inline float Get_Phase_Current_A() const;
    inline float Get_Phase_Current_B() const;
    inline float Get_Phase_Current_C() const;
    inline float Get_Multi_Turn_Angle() const;
    inline float Get_Single_Turn_Angle() const;
    inline Enum_Motor_IK_Brake_Status Get_Brake_Status() const;
    inline uint8_t Get_Last_Command() const;

    inline const Struct_Motor_IK_Rx_Data &Get_Rx_Data() const;
    inline const Struct_Motor_IK_Control_Parameter_Data &Get_Control_Parameter_Data() const;
    inline const Struct_Motor_IK_Setting_Parameter_Data &Get_Setting_Parameter_Data() const;

    inline Enum_Motor_IK_Control_Method Get_Control_Method() const;
    inline bool Get_Control_Enable() const;
    inline int16_t Get_Control_Power() const;
    inline float Get_Control_Current() const;
    inline float Get_Control_Omega() const;
    inline float Get_Control_Current_Limit() const;
    inline float Get_Control_Angle() const;
    inline float Get_Control_Angle_Increment() const;
    inline float Get_Control_Max_Omega() const;
    inline Enum_Motor_IK_Spin_Direction Get_Control_Spin_Direction() const;

    inline void Set_Control_Method(const Enum_Motor_IK_Control_Method &__Control_Method);
    inline void Set_Control_Enable(const bool &__Control_Enable);
    inline void Set_Control_Power(const int16_t &__Control_Power);
    inline void Set_Control_Current(const float &__Control_Current);
    inline void Set_Control_Omega(const float &__Control_Omega);
    inline void Set_Control_Current_Limit(const float &__Control_Current_Limit);
    inline void Set_Control_Angle(const float &__Control_Angle);
    inline void Set_Control_Angle_Increment(const float &__Control_Angle_Increment);
    inline void Set_Control_Max_Omega(const float &__Control_Max_Omega);
    inline void Set_Control_Spin_Direction(const Enum_Motor_IK_Spin_Direction &__Control_Spin_Direction);

    /**
     * @brief 使用当前 CAN 管理对象中的 Header/Buffer 处理接收帧
     */
    void CAN_RxCpltCallback();

    /**
     * @brief 直接处理回调传入的 Header/Buffer；便于在总线总回调中直接调用
     */
    void CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header, const uint8_t *Buffer);

    // 1~8：状态、错误、运行与抱闸
    void CAN_Send_Read_Status_1() const;
    void CAN_Send_Clear_Error() const;
    void CAN_Send_Read_Status_2() const;
    void CAN_Send_Read_Status_3() const;
    void CAN_Send_Motor_Shutdown() const;
    void CAN_Send_Motor_Run() const;
    void CAN_Send_Motor_Stop() const;
    void CAN_Send_Brake(const Enum_Motor_IK_Brake_Command &__Brake_Command) const;
    void CAN_Send_Read_Brake() const;

    // 9~17：控制命令
    void CAN_Send_Open_Loop_Control(const int16_t &__Power_Control) const;
    void CAN_Send_Torque_Control(const float &__Current) const;
    void CAN_Send_Torque_Control_Raw(const int16_t &__IQ_Control) const;
    void CAN_Send_Speed_Control(const float &__Omega, const float &__Current_Limit) const;
    void CAN_Send_Speed_Control_Raw(const int32_t &__Speed_Control, const int16_t &__IQ_Limit) const;
    void CAN_Send_Multi_Turn_Angle_Control_1(const float &__Angle) const;
    void CAN_Send_Multi_Turn_Angle_Control_2(const float &__Angle, const float &__Max_Omega) const;
    void CAN_Send_Single_Turn_Angle_Control_1(const float &__Angle, const Enum_Motor_IK_Spin_Direction &__Direction) const;
    void CAN_Send_Single_Turn_Angle_Control_2(const float &__Angle, const float &__Max_Omega, const Enum_Motor_IK_Spin_Direction &__Direction) const;
    void CAN_Send_Increment_Angle_Control_1(const float &__Angle_Increment) const;
    void CAN_Send_Increment_Angle_Control_2(const float &__Angle_Increment, const float &__Max_Omega) const;

    // 18~19：RAM 控制参数
    void CAN_Send_Read_Control_Parameter(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID) const;
    void CAN_Send_Write_Control_Parameter_Raw(const uint8_t &__Parameter_ID, const uint8_t __Parameter_Data[6]) const;
    void CAN_Send_Write_Control_PID(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID, const uint16_t &__K_P, const uint16_t &__K_I, const uint16_t &__K_D) const;
    void CAN_Send_Write_Control_Parameter_I16(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID, const int16_t &__Value) const;
    void CAN_Send_Write_Control_Parameter_I32(const Enum_Motor_IK_Control_Parameter_ID &__Parameter_ID, const int32_t &__Value) const;

    // 20~25：编码器和角度
    void CAN_Send_Read_Encoder() const;
    void CAN_Send_Calibrate_Encoder() const;
    void CAN_Send_Set_Zero_ROM() const;
    void CAN_Send_Read_Multi_Turn_Angle() const;
    void CAN_Send_Read_Single_Turn_Angle() const;
    void CAN_Send_Set_Zero_RAM() const;

    // 26~29：设定参数、保存与重启
    void CAN_Send_Read_Setting_Parameter_Raw(const uint8_t &__Parameter_1, const uint8_t &__Parameter_2 = 0x00) const;
    void CAN_Send_Write_Setting_Parameter_Raw(const uint8_t __Parameter_Data[7]) const;
    void CAN_Send_Read_Single_Setting(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID) const;
    void CAN_Send_Write_Single_Setting_U8(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const uint8_t &__Value) const;
    void CAN_Send_Write_Single_Setting_I16(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const int16_t &__Value) const;
    void CAN_Send_Write_Single_Setting_I32(const Enum_Motor_IK_Setting_Parameter_ID &__Parameter_ID, const int32_t &__Value) const;
    void CAN_Send_Read_Setting_PID(const Enum_Motor_IK_Setting_PID_ID &__PID_ID) const;
    void CAN_Send_Write_Setting_PID(const Enum_Motor_IK_Setting_PID_ID &__PID_ID, const uint16_t &__K_P, const uint16_t &__K_I, const uint16_t &__K_D) const;
    void CAN_Send_Save_Setting_Parameters() const;
    void CAN_Send_Restart() const;

    /**
     * @brief 周期检测是否在窗口内收到过有效反馈
     */
    void TIM_100ms_Alive_PeriodElapsedCallback();

    /**
     * @brief 按 Control_Method 周期发送控制命令
     * @note 增量位置命令是一次性命令；Set_Control_Angle_Increment() 后仅发送一次。
     */
    void TIM_Send_PeriodElapsedCallback();

protected:
    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    FDCAN_HandleTypeDef *CAN_Handler = nullptr;
    uint8_t Motor_ID = 1;
    uint16_t CAN_ID = 0x141;
    Enum_Motor_IK_Series Motor_Series = Motor_IK_Series_MF;
    uint8_t Encoder_Bits = 14;
    uint32_t Encoder_Counts = 16384;
    float Current_Resolution = 33.0f / 4096.0f;

    // 在线检测
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    Enum_Motor_IK_Status Motor_IK_Status = Motor_IK_Status_DISABLE;

    // 接收数据
    Struct_Motor_IK_Rx_Data Rx_Data = {};
    Struct_Motor_IK_Control_Parameter_Data Control_Parameter_Data = {};
    Struct_Motor_IK_Setting_Parameter_Data Setting_Parameter_Data = {};

    // 周期控制参数
    Enum_Motor_IK_Control_Method Control_Method = Motor_IK_Control_Method_NONE;
    bool Control_Enable = false;
    int16_t Control_Power = 0;
    float Control_Current = 0.0f;
    float Control_Omega = 0.0f;
    float Control_Current_Limit = 0.0f;
    float Control_Angle = 0.0f;
    float Control_Angle_Increment = 0.0f;
    float Control_Max_Omega = 0.0f;
    Enum_Motor_IK_Spin_Direction Control_Spin_Direction = Motor_IK_Spin_Direction_CLOCKWISE;
    bool Increment_Command_Pending = false;

    // 内部函数
    void Reset_Data();
    void Data_Process(const uint8_t *Buffer);
    void Parse_Status_1(const uint8_t *Buffer);
    void Parse_Status_2(const uint8_t *Buffer);
    void Parse_Status_3(const uint8_t *Buffer);
    void Parse_Control_Parameter(const uint8_t *Buffer);
    void Parse_Setting_Parameter(const uint8_t *Buffer);
    void Output();

    void Send_Frame(uint8_t *Data) const;
    void Send_Empty_Command(const uint8_t &Command) const;

    int16_t Current_To_Raw(const float &Current) const;
    float Raw_To_Current(const int16_t &Raw) const;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Enum_Motor_IK_Status Class_Motor_IK::Get_Status() const
{
    return (Motor_IK_Status);
}

inline uint8_t Class_Motor_IK::Get_Motor_ID() const
{
    return (Motor_ID);
}

inline uint16_t Class_Motor_IK::Get_CAN_ID() const
{
    return (CAN_ID);
}

inline Enum_Motor_IK_Series Class_Motor_IK::Get_Motor_Series() const
{
    return (Motor_Series);
}

inline float Class_Motor_IK::Get_Current_Resolution() const
{
    return (Current_Resolution);
}

inline Enum_Motor_IK_Motor_State Class_Motor_IK::Get_Motor_State() const
{
    return (Rx_Data.Motor_State);
}

inline uint8_t Class_Motor_IK::Get_Error_State() const
{
    return (Rx_Data.Error_State);
}

inline bool Class_Motor_IK::Get_Error_Flag(const Enum_Motor_IK_Error_Flag &__Error_Flag) const
{
    return ((Rx_Data.Error_State & static_cast<uint8_t>(__Error_Flag)) != 0u);
}

inline float Class_Motor_IK::Get_Now_Temperature() const
{
    return (Rx_Data.Temperature);
}

inline float Class_Motor_IK::Get_Now_Temperature_Celsius() const
{
    return (Rx_Data.Temperature - 273.15f);
}

inline float Class_Motor_IK::Get_Bus_Voltage() const
{
    return (Rx_Data.Bus_Voltage);
}

inline float Class_Motor_IK::Get_Bus_Current() const
{
    return (Rx_Data.Bus_Current);
}

inline float Class_Motor_IK::Get_Now_Current() const
{
    return (Rx_Data.Torque_Current);
}

inline int16_t Class_Motor_IK::Get_Now_Power() const
{
    return (Rx_Data.Output_Power);
}

inline float Class_Motor_IK::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

inline uint16_t Class_Motor_IK::Get_Now_Encoder() const
{
    return (Rx_Data.Encoder);
}

inline float Class_Motor_IK::Get_Now_Encoder_Angle() const
{
    return (Rx_Data.Encoder_Angle);
}

inline float Class_Motor_IK::Get_Phase_Current_A() const
{
    return (Rx_Data.Phase_Current[0]);
}

inline float Class_Motor_IK::Get_Phase_Current_B() const
{
    return (Rx_Data.Phase_Current[1]);
}

inline float Class_Motor_IK::Get_Phase_Current_C() const
{
    return (Rx_Data.Phase_Current[2]);
}

inline float Class_Motor_IK::Get_Multi_Turn_Angle() const
{
    return (Rx_Data.Multi_Turn_Angle);
}

inline float Class_Motor_IK::Get_Single_Turn_Angle() const
{
    return (Rx_Data.Single_Turn_Angle);
}

inline Enum_Motor_IK_Brake_Status Class_Motor_IK::Get_Brake_Status() const
{
    return (Rx_Data.Brake_Status);
}

inline uint8_t Class_Motor_IK::Get_Last_Command() const
{
    return (Rx_Data.Last_Command);
}

inline const Struct_Motor_IK_Rx_Data &Class_Motor_IK::Get_Rx_Data() const
{
    return (Rx_Data);
}

inline const Struct_Motor_IK_Control_Parameter_Data &Class_Motor_IK::Get_Control_Parameter_Data() const
{
    return (Control_Parameter_Data);
}

inline const Struct_Motor_IK_Setting_Parameter_Data &Class_Motor_IK::Get_Setting_Parameter_Data() const
{
    return (Setting_Parameter_Data);
}

inline Enum_Motor_IK_Control_Method Class_Motor_IK::Get_Control_Method() const
{
    return (Control_Method);
}

inline bool Class_Motor_IK::Get_Control_Enable() const
{
    return (Control_Enable);
}

inline int16_t Class_Motor_IK::Get_Control_Power() const
{
    return (Control_Power);
}

inline float Class_Motor_IK::Get_Control_Current() const
{
    return (Control_Current);
}

inline float Class_Motor_IK::Get_Control_Omega() const
{
    return (Control_Omega);
}

inline float Class_Motor_IK::Get_Control_Current_Limit() const
{
    return (Control_Current_Limit);
}

inline float Class_Motor_IK::Get_Control_Angle() const
{
    return (Control_Angle);
}

inline float Class_Motor_IK::Get_Control_Angle_Increment() const
{
    return (Control_Angle_Increment);
}

inline float Class_Motor_IK::Get_Control_Max_Omega() const
{
    return (Control_Max_Omega);
}

inline Enum_Motor_IK_Spin_Direction Class_Motor_IK::Get_Control_Spin_Direction() const
{
    return (Control_Spin_Direction);
}

inline void Class_Motor_IK::Set_Control_Method(const Enum_Motor_IK_Control_Method &__Control_Method)
{
    Control_Method = __Control_Method;
    if ((Control_Method != Motor_IK_Control_Method_INCREMENT_ANGLE_1) &&
        (Control_Method != Motor_IK_Control_Method_INCREMENT_ANGLE_2))
    {
        Increment_Command_Pending = false;
    }
}

inline void Class_Motor_IK::Set_Control_Enable(const bool &__Control_Enable)
{
    Control_Enable = __Control_Enable;
}

inline void Class_Motor_IK::Set_Control_Power(const int16_t &__Control_Power)
{
    Control_Power = __Control_Power;
}

inline void Class_Motor_IK::Set_Control_Current(const float &__Control_Current)
{
    Control_Current = __Control_Current;
}

inline void Class_Motor_IK::Set_Control_Omega(const float &__Control_Omega)
{
    Control_Omega = __Control_Omega;
}

inline void Class_Motor_IK::Set_Control_Current_Limit(const float &__Control_Current_Limit)
{
    Control_Current_Limit = __Control_Current_Limit;
}

inline void Class_Motor_IK::Set_Control_Angle(const float &__Control_Angle)
{
    Control_Angle = __Control_Angle;
}

inline void Class_Motor_IK::Set_Control_Angle_Increment(const float &__Control_Angle_Increment)
{
    Control_Angle_Increment = __Control_Angle_Increment;
    Increment_Command_Pending = true;
}

inline void Class_Motor_IK::Set_Control_Max_Omega(const float &__Control_Max_Omega)
{
    Control_Max_Omega = __Control_Max_Omega;
}

inline void Class_Motor_IK::Set_Control_Spin_Direction(const Enum_Motor_IK_Spin_Direction &__Control_Spin_Direction)
{
    Control_Spin_Direction = __Control_Spin_Direction;
}

#endif

/* End of file ---------------------------------------------------------------*/
