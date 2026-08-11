/**
 * @file dvc_motor_RS.h
 * @author Project integration
 * @brief RobStride/灵足 EL05 私有扩展 CAN 协议电机驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《EL05 使用说明书 260713》新增
 *
 * @note
 * 1. 本驱动实现手册第 4 章 RobStride 私有协议：CAN 2.0、1 Mbps、29 位扩展数据帧、DLC=8。
 * 2. 扩展帧 ID = (Communication_Type << 24) | (Data_Field << 8) | Target_ID。
 * 3. 对外物理量统一使用：角度 rad、角速度 rad/s、转矩 N.m、电流 A、温度 K。
 * 4. 用户提供的 drv_can 默认拒绝未匹配扩展帧；接入前请应用随库附带的 drv_can 扩展帧补丁。
 * 5. 本驱动仅实现私有协议；可发送 CANopen/MIT 协议切换命令，但不实现对应控制栈。
 */

#ifndef DVC_MOTOR_RS_H
#define DVC_MOTOR_RS_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机通信在线状态
 */
enum Enum_Motor_RS_Status : uint8_t
{
    Motor_RS_Status_DISABLE = 0,
    Motor_RS_Status_ENABLE,
};

/**
 * @brief RobStride 私有协议通信类型
 */
enum Enum_Motor_RS_Communication_Type : uint8_t
{
    Motor_RS_Communication_Type_GET_DEVICE_ID = 0x00,
    Motor_RS_Communication_Type_MOTION_CONTROL = 0x01,
    Motor_RS_Communication_Type_FEEDBACK = 0x02,
    Motor_RS_Communication_Type_ENABLE = 0x03,
    Motor_RS_Communication_Type_STOP = 0x04,
    Motor_RS_Communication_Type_SET_ZERO = 0x06,
    Motor_RS_Communication_Type_SET_CAN_ID = 0x07,
    Motor_RS_Communication_Type_READ_PARAMETER = 0x11,
    Motor_RS_Communication_Type_WRITE_PARAMETER = 0x12,
    Motor_RS_Communication_Type_FAULT = 0x15,
    Motor_RS_Communication_Type_SAVE = 0x16,
    Motor_RS_Communication_Type_SET_BAUDRATE = 0x17,
    Motor_RS_Communication_Type_ACTIVE_REPORT = 0x18,
    Motor_RS_Communication_Type_SET_PROTOCOL = 0x19,
};

/**
 * @brief 电机内部运行模式，对应参数 0x7005
 */
enum Enum_Motor_RS_Run_Mode : uint8_t
{
    Motor_RS_Run_Mode_MOTION = 0,
    Motor_RS_Run_Mode_POSITION_PP = 1,
    Motor_RS_Run_Mode_SPEED = 2,
    Motor_RS_Run_Mode_CURRENT = 3,
    Motor_RS_Run_Mode_POSITION_CSP = 5,
};

/**
 * @brief 反馈帧 ID 中的电机状态
 */
enum Enum_Motor_RS_Mode_State : uint8_t
{
    Motor_RS_Mode_State_RESET = 0,
    Motor_RS_Mode_State_CALIBRATION = 1,
    Motor_RS_Mode_State_MOTOR = 2,
    Motor_RS_Mode_State_RESERVED = 3,
};

/**
 * @brief 周期发送使用的控制方式
 */
enum Enum_Motor_RS_Control_Method : uint8_t
{
    Motor_RS_Control_Method_NONE = 0,
    Motor_RS_Control_Method_MOTION,
    Motor_RS_Control_Method_CURRENT,
    Motor_RS_Control_Method_SPEED,
    Motor_RS_Control_Method_POSITION_PP,
    Motor_RS_Control_Method_POSITION_CSP,
};

/**
 * @brief 反馈帧 ID 中的简要故障位
 */
enum Enum_Motor_RS_Feedback_Fault_Flag : uint8_t
{
    Motor_RS_Feedback_Fault_NONE = 0x00,
    Motor_RS_Feedback_Fault_UNDERVOLTAGE = 0x01,
    Motor_RS_Feedback_Fault_PHASE_CURRENT = 0x02,
    Motor_RS_Feedback_Fault_OVERTEMPERATURE = 0x04,
    Motor_RS_Feedback_Fault_MAGNETIC_ENCODER = 0x08,
    Motor_RS_Feedback_Fault_STALL_OVERLOAD = 0x10,
    Motor_RS_Feedback_Fault_UNCALIBRATED = 0x20,
};

/**
 * @brief 通信类型 21 的详细 fault 位
 */
enum Enum_Motor_RS_Fault_Flag : uint32_t
{
    Motor_RS_Fault_NONE = 0x00000000UL,
    Motor_RS_Fault_MOTOR_OVERTEMPERATURE = 1UL << 0,
    Motor_RS_Fault_DRIVER_CHIP = 1UL << 1,
    Motor_RS_Fault_UNDERVOLTAGE = 1UL << 2,
    Motor_RS_Fault_OVERVOLTAGE = 1UL << 3,
    Motor_RS_Fault_PHASE_B_OVERCURRENT = 1UL << 4,
    Motor_RS_Fault_PHASE_C_OVERCURRENT = 1UL << 5,
    Motor_RS_Fault_ENCODER_UNCALIBRATED = 1UL << 7,
    Motor_RS_Fault_HARDWARE_IDENTIFICATION = 1UL << 8,
    Motor_RS_Fault_POSITION_INITIALIZATION = 1UL << 9,
    Motor_RS_Fault_STALL_OVERLOAD = 1UL << 14,
    Motor_RS_Fault_PHASE_A_OVERCURRENT = 1UL << 16,
};

/**
 * @brief 通信类型 21 的 warning 位
 */
enum Enum_Motor_RS_Warning_Flag : uint32_t
{
    Motor_RS_Warning_NONE = 0x00000000UL,
    Motor_RS_Warning_MOTOR_OVERTEMPERATURE = 1UL << 0,
};

/**
 * @brief 电机波特率修改命令值
 */
enum Enum_Motor_RS_CAN_Baudrate : uint8_t
{
    Motor_RS_CAN_Baudrate_1M = 0x01,
    Motor_RS_CAN_Baudrate_500K = 0x02,
    Motor_RS_CAN_Baudrate_250K = 0x03,
    Motor_RS_CAN_Baudrate_125K = 0x04,
};

/**
 * @brief 电机协议类型
 */
enum Enum_Motor_RS_Protocol : uint8_t
{
    Motor_RS_Protocol_PRIVATE = 0,
    Motor_RS_Protocol_CANOPEN = 1,
    Motor_RS_Protocol_MIT = 2,
};

/**
 * @brief 零点位置范围标志，对应参数 0x7029
 */
enum Enum_Motor_RS_Zero_Range : uint8_t
{
    Motor_RS_Zero_Range_0_TO_2PI = 0,
    Motor_RS_Zero_Range_NEGATIVE_PI_TO_PI = 1,
};

/**
 * @brief 私有协议可读写单参数索引
 */
enum Enum_Motor_RS_Parameter_ID : uint16_t
{
    Motor_RS_Parameter_RUN_MODE = 0x7005,
    Motor_RS_Parameter_IQ_REFERENCE = 0x7006,
    Motor_RS_Parameter_SPEED_REFERENCE = 0x700A,
    Motor_RS_Parameter_TORQUE_LIMIT = 0x700B,
    Motor_RS_Parameter_CURRENT_KP = 0x7010,
    Motor_RS_Parameter_CURRENT_KI = 0x7011,
    Motor_RS_Parameter_CURRENT_FILTER_GAIN = 0x7014,
    Motor_RS_Parameter_POSITION_REFERENCE = 0x7016,
    Motor_RS_Parameter_CSP_SPEED_LIMIT = 0x7017,
    Motor_RS_Parameter_CURRENT_LIMIT = 0x7018,
    Motor_RS_Parameter_MECHANICAL_POSITION = 0x7019,
    Motor_RS_Parameter_IQ_FILTERED = 0x701A,
    Motor_RS_Parameter_MECHANICAL_VELOCITY = 0x701B,
    Motor_RS_Parameter_BUS_VOLTAGE = 0x701C,
    Motor_RS_Parameter_POSITION_KP = 0x701E,
    Motor_RS_Parameter_SPEED_KP = 0x701F,
    Motor_RS_Parameter_SPEED_KI = 0x7020,
    Motor_RS_Parameter_SPEED_FILTER_GAIN = 0x7021,
    Motor_RS_Parameter_SPEED_ACCELERATION = 0x7022,
    Motor_RS_Parameter_PP_MAX_SPEED = 0x7024,
    Motor_RS_Parameter_PP_ACCELERATION = 0x7025,
    Motor_RS_Parameter_ACTIVE_REPORT_TIME = 0x7026,
    Motor_RS_Parameter_CAN_TIMEOUT = 0x7028,
    Motor_RS_Parameter_ZERO_RANGE = 0x7029,
    Motor_RS_Parameter_ZERO_OFFSET = 0x702B,
    Motor_RS_Parameter_COGGING_COMPENSATION_ENABLE = 0x702C,
    Motor_RS_Parameter_INITIAL_CALIBRATION_ENABLE = 0x702D,
    Motor_RS_Parameter_PP_DECELERATION = 0x702E,
};

/**
 * @brief 参数值类型
 */
enum Enum_Motor_RS_Parameter_Type : uint8_t
{
    Motor_RS_Parameter_Type_UNKNOWN = 0,
    Motor_RS_Parameter_Type_UINT8,
    Motor_RS_Parameter_Type_UINT16,
    Motor_RS_Parameter_Type_UINT32,
    Motor_RS_Parameter_Type_INT32,
    Motor_RS_Parameter_Type_FLOAT,
};

/**
 * @brief 29 位扩展帧 ID 解析结果
 */
struct Struct_Motor_RS_Extended_ID
{
    uint32_t Raw_ID;
    uint8_t Communication_Type;
    uint16_t Data_Field;
    uint8_t Target_ID;
};

/**
 * @brief 电机反馈数据，物理量单位见文件头说明
 */
struct Struct_Motor_RS_Rx_Data
{
    uint32_t Last_Extended_ID;
    uint8_t Last_Communication_Type;
    uint16_t Last_Data_Field;
    uint8_t Last_Target_ID;
    uint8_t Last_Raw_Data[8];

    uint8_t Motor_ID;
    uint8_t Master_ID;
    Enum_Motor_RS_Mode_State Mode_State;
    uint8_t Feedback_Fault;

    uint16_t Position_Raw;
    uint16_t Velocity_Raw;
    uint16_t Torque_Raw;
    int16_t Temperature_Raw;

    // 类型 2 周期角度，范围约为 -4pi~4pi
    float Now_Angle;
    // 对周期角度做跨边界展开后的累计角度
    float Total_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;

    // 通过类型 17 读取到的常用观测量
    bool Mechanical_Position_Valid;
    float Mechanical_Position;
    bool IQ_Filtered_Valid;
    float IQ_Filtered;
    bool Mechanical_Velocity_Valid;
    float Mechanical_Velocity;
    bool Bus_Voltage_Valid;
    float Bus_Voltage;
};

/**
 * @brief 通信类型 0 的设备信息
 * @note 手册未规定 UID 字节序，因此保留原始 8 字节并提供两种整数拼接结果。
 */
struct Struct_Motor_RS_Device_Info
{
    bool Valid;
    uint8_t Motor_ID;
    uint8_t UID_Raw[8];
    uint64_t UID_Little_Endian;
    uint64_t UID_Big_Endian;
};

/**
 * @brief 版本号读取结果
 */
struct Struct_Motor_RS_Version_Data
{
    bool Valid;
    uint8_t Motor_ID;
    uint8_t Version_Byte[4];
    uint32_t Version_U32;
};

/**
 * @brief 最近一次参数读取结果
 */
struct Struct_Motor_RS_Parameter_Data
{
    bool Valid;
    bool Success;
    uint8_t Result_Code;
    uint8_t Motor_ID;
    uint16_t Index;
    Enum_Motor_RS_Parameter_Type Type;
    uint8_t Raw[4];

    uint8_t Value_U8;
    uint16_t Value_U16;
    uint32_t Value_U32;
    int32_t Value_I32;
    float Value_Float;
};

/**
 * @brief 最近一次详细故障反馈
 */
struct Struct_Motor_RS_Fault_Data
{
    bool Valid;
    uint8_t Motor_ID;
    uint32_t Fault;
    uint32_t Warning;
};

/**
 * @brief Reusable, RobStride EL05 私有扩展 CAN 协议驱动
 */
class Class_Motor_RS
{
public:
    /**
     * @brief 配置一个扩展帧全通过滤器
     * @note 仅在不使用随库 drv_can 补丁时需要；必须在 CAN_Init()/HAL_FDCAN_Start() 前调用，
     *       且 CubeMX 的 Ext Filters Nbr 至少为 1。
     */
    static uint8_t CAN_Config_Extended_Filter(FDCAN_HandleTypeDef *hcan,
                                              const uint32_t &__Filter_Index = 0);

    static uint32_t Build_Extended_ID(const uint8_t &__Communication_Type,
                                      const uint16_t &__Data_Field,
                                      const uint8_t &__Target_ID);
    static Struct_Motor_RS_Extended_ID Parse_Extended_ID(const uint32_t &__Extended_ID);

    void Init(const FDCAN_HandleTypeDef *hcan,
              const uint8_t &__Motor_ID = 1,
              const uint8_t &__Master_ID = 0xFD);

    inline Enum_Motor_RS_Status Get_Status() const;
    inline uint8_t Get_Motor_ID() const;
    inline uint8_t Get_Master_ID() const;
    inline uint8_t Get_Last_Tx_Status() const;
    inline bool Get_CAN_ID_Change_Pending() const;
    inline uint8_t Get_Pending_Motor_ID() const;

    inline Enum_Motor_RS_Mode_State Get_Mode_State() const;
    inline uint8_t Get_Feedback_Fault() const;
    inline bool Get_Feedback_Fault_Flag(const Enum_Motor_RS_Feedback_Fault_Flag &__Fault_Flag) const;
    inline float Get_Now_Angle() const;
    inline float Get_Total_Angle() const;
    inline float Get_Now_Omega() const;
    inline float Get_Now_Torque() const;
    inline float Get_Now_Temperature() const;
    inline float Get_Now_Temperature_Celsius() const;

    inline const Struct_Motor_RS_Rx_Data &Get_Rx_Data() const;
    inline const Struct_Motor_RS_Device_Info &Get_Device_Info() const;
    inline const Struct_Motor_RS_Version_Data &Get_Version_Data() const;
    inline const Struct_Motor_RS_Parameter_Data &Get_Parameter_Data() const;
    inline const Struct_Motor_RS_Fault_Data &Get_Fault_Data() const;

    inline bool Get_Fault_Flag(const Enum_Motor_RS_Fault_Flag &__Fault_Flag) const;
    inline bool Get_Warning_Flag(const Enum_Motor_RS_Warning_Flag &__Warning_Flag) const;

    inline Enum_Motor_RS_Control_Method Get_Control_Method() const;
    inline bool Get_Control_Enable() const;
    inline float Get_Control_Angle() const;
    inline float Get_Control_Omega() const;
    inline float Get_Control_Torque() const;
    inline float Get_Control_Current() const;
    inline float Get_Control_K_P() const;
    inline float Get_Control_K_D() const;
    inline float Get_Control_Current_Limit() const;
    inline float Get_Control_Speed_Limit() const;
    inline float Get_Control_Acceleration() const;
    inline float Get_Control_Deceleration() const;

    inline void Set_Motor_ID_Local(const uint8_t &__Motor_ID);
    inline void Set_Motor_ID(const uint8_t &__Motor_ID);
    inline void Set_Master_ID(const uint8_t &__Master_ID);
    inline void Set_Control_Method(const Enum_Motor_RS_Control_Method &__Control_Method);
    inline void Set_Control_Enable(const bool &__Control_Enable);
    inline void Set_Control_Angle(const float &__Control_Angle);
    inline void Set_Control_Omega(const float &__Control_Omega);
    inline void Set_Control_Torque(const float &__Control_Torque);
    inline void Set_Control_Current(const float &__Control_Current);
    inline void Set_Control_K_P(const float &__Control_K_P);
    inline void Set_Control_K_D(const float &__Control_K_D);
    inline void Set_Control_Current_Limit(const float &__Control_Current_Limit);
    inline void Set_Control_Speed_Limit(const float &__Control_Speed_Limit);
    inline void Set_Control_Acceleration(const float &__Control_Acceleration);
    inline void Set_Control_Deceleration(const float &__Control_Deceleration);

    void CAN_RxCpltCallback();
    void CAN_RxCpltCallback(const FDCAN_RxHeaderTypeDef &Header, const uint8_t *Buffer);

    // 通信类型 0~7
    void CAN_Send_Get_Device_ID();
    void CAN_Send_Get_Device_ID(const uint8_t &__Target_ID);
    void CAN_Send_Motion_Control(const float &__Angle,
                                 const float &__Omega,
                                 const float &__K_P,
                                 const float &__K_D,
                                 const float &__Torque) const;
    void CAN_Send_Enable() const;
    void CAN_Send_Stop(const bool &__Clear_Error = false) const;
    void CAN_Send_Clear_Error() const;

    // 与 dvc_motor_IK/dvc_motor_dm 风格一致的别名
    inline void CAN_Send_Motor_Run() const { CAN_Send_Enable(); }
    inline void CAN_Send_Motor_Stop(const bool &__Clear_Error = false) const { CAN_Send_Stop(__Clear_Error); }
    inline void CAN_Send_Clear_Fault() const { CAN_Send_Clear_Error(); }

    void CAN_Send_Read_Version() const;
    void CAN_Send_Set_Zero() const;
    void CAN_Send_Set_CAN_ID(const uint8_t &__New_Motor_ID);

    // 通信类型 17/18：单参数读写
    void CAN_Send_Read_Parameter(const uint16_t &__Index) const;
    void CAN_Send_Write_Parameter_Raw(const uint16_t &__Index, const uint8_t __Raw[4]) const;
    void CAN_Send_Write_Parameter_U8(const uint16_t &__Index, const uint8_t &__Value) const;
    void CAN_Send_Write_Parameter_U16(const uint16_t &__Index, const uint16_t &__Value) const;
    void CAN_Send_Write_Parameter_U32(const uint16_t &__Index, const uint32_t &__Value) const;
    void CAN_Send_Write_Parameter_I32(const uint16_t &__Index, const int32_t &__Value) const;
    void CAN_Send_Write_Parameter_Float(const uint16_t &__Index, const float &__Value) const;

    // 常用参数快捷接口
    void CAN_Send_Set_Run_Mode(const Enum_Motor_RS_Run_Mode &__Run_Mode) const;
    void CAN_Send_Set_Current_Reference(const float &__Current) const;
    void CAN_Send_Set_Speed_Reference(const float &__Omega) const;
    void CAN_Send_Set_Position_Reference(const float &__Angle) const;
    void CAN_Send_Set_Torque_Limit(const float &__Torque_Limit) const;
    void CAN_Send_Set_Current_Limit(const float &__Current_Limit) const;
    void CAN_Send_Set_CSP_Speed_Limit(const float &__Omega_Limit) const;
    void CAN_Send_Set_Speed_Acceleration(const float &__Acceleration) const;
    void CAN_Send_Set_PP_Max_Speed(const float &__Omega_Limit) const;
    void CAN_Send_Set_PP_Acceleration(const float &__Acceleration) const;
    void CAN_Send_Set_PP_Deceleration(const float &__Deceleration) const;
    void CAN_Send_Set_Active_Report_Period_Raw(const uint16_t &__Period_Raw) const;
    void CAN_Send_Set_Active_Report_Period_MS(const uint16_t &__Period_MS) const;
    void CAN_Send_Set_CAN_Timeout_Raw(const uint32_t &__Timeout_Raw) const;
    void CAN_Send_Set_CAN_Timeout_Seconds(const float &__Timeout_Seconds) const;
    void CAN_Send_Set_Zero_Range(const Enum_Motor_RS_Zero_Range &__Zero_Range) const;
    void CAN_Send_Set_Zero_Offset(const float &__Offset) const;
    void CAN_Send_Set_Cogging_Compensation(const bool &__Enable) const;
    void CAN_Send_Set_Initial_Calibration(const bool &__Enable) const;

    // 通信类型 21~25
    void CAN_Send_Read_Fault_State() const;
    void CAN_Send_Save_Parameters() const;
    void CAN_Send_Set_Baudrate(const Enum_Motor_RS_CAN_Baudrate &__Baudrate) const;
    void CAN_Send_Set_Active_Report(const bool &__Enable) const;
    void CAN_Send_Set_Protocol(const Enum_Motor_RS_Protocol &__Protocol) const;

    /**
     * @brief 根据当前 Control_Method 写入 run_mode 和限制参数，不自动使能
     */
    void CAN_Send_Apply_Control_Configuration() const;

    /**
     * @brief 发送“停止 -> 模式/限制配置 -> 使能”序列
     * @note 模式切换应在电机停止时进行。若总线负载较高，可在工程中按步骤分时调用。
     */
    void CAN_Send_Start_Control();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    /**
     * @brief 按 Control_Method 周期发送目标值
     * @note PP 模式目标仅在 Set_Control_Angle() 或重新启用后发送一次，避免重复启动轨迹规划。
     */
    void TIM_Send_PeriodElapsedCallback();

private:
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    FDCAN_HandleTypeDef *CAN_Handler = nullptr;
    uint8_t Motor_ID = 1;
    uint8_t Master_ID = 0xFD;
    mutable uint8_t Last_Tx_Status = 0xFF;

    bool Device_Query_Pending = false;
    uint8_t Device_Query_Target = 0x7F;
    bool CAN_ID_Change_Pending = false;
    uint8_t Pending_Motor_ID = 0;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    Enum_Motor_RS_Status Motor_RS_Status = Motor_RS_Status_DISABLE;

    Struct_Motor_RS_Rx_Data Rx_Data = {};
    Struct_Motor_RS_Device_Info Device_Info = {};
    Struct_Motor_RS_Version_Data Version_Data = {};
    Struct_Motor_RS_Parameter_Data Parameter_Data = {};
    Struct_Motor_RS_Fault_Data Fault_Data = {};

    bool Angle_Unwrap_Initialized = false;
    float Previous_Cyclic_Angle = 0.0f;

    Enum_Motor_RS_Control_Method Control_Method = Motor_RS_Control_Method_NONE;
    bool Control_Enable = false;
    float Control_Angle = 0.0f;
    float Control_Omega = 0.0f;
    float Control_Torque = 0.0f;
    float Control_Current = 0.0f;
    float Control_K_P = 0.0f;
    float Control_K_D = 0.0f;
    float Control_Current_Limit = 0.0f;
    float Control_Speed_Limit = 0.0f;
    float Control_Acceleration = 0.0f;
    float Control_Deceleration = 0.0f;
    bool PP_Command_Pending = false;

    void Reset_Data();
    bool Is_Frame_For_This_Motor(const Struct_Motor_RS_Extended_ID &__ID) const;
    void Data_Process(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Parse_Device_Info(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Parse_Feedback(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Parse_Version(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Parse_Parameter(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Parse_Fault(const Struct_Motor_RS_Extended_ID &__ID, const uint8_t *Buffer);
    void Output();

    void Send_Frame(const uint32_t &__Extended_ID, const uint8_t __Data[8]) const;
    void Send_Command(const uint8_t &__Communication_Type,
                      const uint16_t &__Data_Field,
                      const uint8_t &__Target_ID,
                      const uint8_t __Data[8]) const;

    static Enum_Motor_RS_Parameter_Type Get_Parameter_Type(const uint16_t &__Index);
    static uint8_t Clamp_Motor_ID(const uint8_t &__Motor_ID);
};

/* Inline methods ------------------------------------------------------------*/

inline Enum_Motor_RS_Status Class_Motor_RS::Get_Status() const { return (Motor_RS_Status); }
inline uint8_t Class_Motor_RS::Get_Motor_ID() const { return (Motor_ID); }
inline uint8_t Class_Motor_RS::Get_Master_ID() const { return (Master_ID); }
inline uint8_t Class_Motor_RS::Get_Last_Tx_Status() const { return (Last_Tx_Status); }
inline bool Class_Motor_RS::Get_CAN_ID_Change_Pending() const { return (CAN_ID_Change_Pending); }
inline uint8_t Class_Motor_RS::Get_Pending_Motor_ID() const { return (Pending_Motor_ID); }
inline Enum_Motor_RS_Mode_State Class_Motor_RS::Get_Mode_State() const { return (Rx_Data.Mode_State); }
inline uint8_t Class_Motor_RS::Get_Feedback_Fault() const { return (Rx_Data.Feedback_Fault); }
inline bool Class_Motor_RS::Get_Feedback_Fault_Flag(const Enum_Motor_RS_Feedback_Fault_Flag &__Fault_Flag) const { return ((Rx_Data.Feedback_Fault & static_cast<uint8_t>(__Fault_Flag)) != 0u); }
inline float Class_Motor_RS::Get_Now_Angle() const { return (Rx_Data.Now_Angle); }
inline float Class_Motor_RS::Get_Total_Angle() const { return (Rx_Data.Total_Angle); }
inline float Class_Motor_RS::Get_Now_Omega() const { return (Rx_Data.Now_Omega); }
inline float Class_Motor_RS::Get_Now_Torque() const { return (Rx_Data.Now_Torque); }
inline float Class_Motor_RS::Get_Now_Temperature() const { return (Rx_Data.Now_Temperature); }
inline float Class_Motor_RS::Get_Now_Temperature_Celsius() const { return (Rx_Data.Now_Temperature - 273.15f); }
inline const Struct_Motor_RS_Rx_Data &Class_Motor_RS::Get_Rx_Data() const { return (Rx_Data); }
inline const Struct_Motor_RS_Device_Info &Class_Motor_RS::Get_Device_Info() const { return (Device_Info); }
inline const Struct_Motor_RS_Version_Data &Class_Motor_RS::Get_Version_Data() const { return (Version_Data); }
inline const Struct_Motor_RS_Parameter_Data &Class_Motor_RS::Get_Parameter_Data() const { return (Parameter_Data); }
inline const Struct_Motor_RS_Fault_Data &Class_Motor_RS::Get_Fault_Data() const { return (Fault_Data); }
inline bool Class_Motor_RS::Get_Fault_Flag(const Enum_Motor_RS_Fault_Flag &__Fault_Flag) const { return ((Fault_Data.Fault & static_cast<uint32_t>(__Fault_Flag)) != 0UL); }
inline bool Class_Motor_RS::Get_Warning_Flag(const Enum_Motor_RS_Warning_Flag &__Warning_Flag) const { return ((Fault_Data.Warning & static_cast<uint32_t>(__Warning_Flag)) != 0UL); }
inline Enum_Motor_RS_Control_Method Class_Motor_RS::Get_Control_Method() const { return (Control_Method); }
inline bool Class_Motor_RS::Get_Control_Enable() const { return (Control_Enable); }
inline float Class_Motor_RS::Get_Control_Angle() const { return (Control_Angle); }
inline float Class_Motor_RS::Get_Control_Omega() const { return (Control_Omega); }
inline float Class_Motor_RS::Get_Control_Torque() const { return (Control_Torque); }
inline float Class_Motor_RS::Get_Control_Current() const { return (Control_Current); }
inline float Class_Motor_RS::Get_Control_K_P() const { return (Control_K_P); }
inline float Class_Motor_RS::Get_Control_K_D() const { return (Control_K_D); }
inline float Class_Motor_RS::Get_Control_Current_Limit() const { return (Control_Current_Limit); }
inline float Class_Motor_RS::Get_Control_Speed_Limit() const { return (Control_Speed_Limit); }
inline float Class_Motor_RS::Get_Control_Acceleration() const { return (Control_Acceleration); }
inline float Class_Motor_RS::Get_Control_Deceleration() const { return (Control_Deceleration); }
inline void Class_Motor_RS::Set_Motor_ID_Local(const uint8_t &__Motor_ID) { Motor_ID = Clamp_Motor_ID(__Motor_ID); Rx_Data.Motor_ID = Motor_ID; CAN_ID_Change_Pending = false; }
inline void Class_Motor_RS::Set_Motor_ID(const uint8_t &__Motor_ID) { Set_Motor_ID_Local(__Motor_ID); }
inline void Class_Motor_RS::Set_Master_ID(const uint8_t &__Master_ID) { Master_ID = __Master_ID; Rx_Data.Master_ID = Master_ID; }
inline void Class_Motor_RS::Set_Control_Method(const Enum_Motor_RS_Control_Method &__Control_Method) { Control_Method = __Control_Method; if (Control_Method == Motor_RS_Control_Method_POSITION_PP) { PP_Command_Pending = true; } }
inline void Class_Motor_RS::Set_Control_Enable(const bool &__Control_Enable) { Control_Enable = __Control_Enable; if (Control_Enable && (Control_Method == Motor_RS_Control_Method_POSITION_PP)) { PP_Command_Pending = true; } }
inline void Class_Motor_RS::Set_Control_Angle(const float &__Control_Angle) { Control_Angle = __Control_Angle; if (Control_Method == Motor_RS_Control_Method_POSITION_PP) { PP_Command_Pending = true; } }
inline void Class_Motor_RS::Set_Control_Omega(const float &__Control_Omega) { Control_Omega = __Control_Omega; }
inline void Class_Motor_RS::Set_Control_Torque(const float &__Control_Torque) { Control_Torque = __Control_Torque; }
inline void Class_Motor_RS::Set_Control_Current(const float &__Control_Current) { Control_Current = __Control_Current; }
inline void Class_Motor_RS::Set_Control_K_P(const float &__Control_K_P) { Control_K_P = __Control_K_P; }
inline void Class_Motor_RS::Set_Control_K_D(const float &__Control_K_D) { Control_K_D = __Control_K_D; }
inline void Class_Motor_RS::Set_Control_Current_Limit(const float &__Control_Current_Limit) { Control_Current_Limit = __Control_Current_Limit; }
inline void Class_Motor_RS::Set_Control_Speed_Limit(const float &__Control_Speed_Limit) { Control_Speed_Limit = __Control_Speed_Limit; }
inline void Class_Motor_RS::Set_Control_Acceleration(const float &__Control_Acceleration) { Control_Acceleration = __Control_Acceleration; }
inline void Class_Motor_RS::Set_Control_Deceleration(const float &__Control_Deceleration) { Control_Deceleration = __Control_Deceleration; }

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
