/**
 * @file dvc_MVR2TB.h
 * @author Project integration
 * @brief MOEVISION MVR2TB TOF 激光测距模组 UART 驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《MVR2TB_Datasheet_CHS_V1.0.4》新增
 *
 * @note
 * 1. UART 参数：115200 bit/s、8 数据位、1 停止位、无校验（8N1）、LVTTL 3.3V。
 * 2. 命令帧：0x56 + CMD + LEN + CONTENT + XOR；响应帧：0x89 + CMD + LEN + CONTENT + XOR。
 * 3. HEX 测量帧为特殊固定 11 字节格式：0x89 0x81 ... XOR，不包含独立 LEN 字段。
 * 4. 接收解析器支持 DMA 空闲中断产生的任意分包、粘包，并同时识别 HEX 与 ASCII 测量数据。
 * 5. 对外距离同时提供 mm 原始值和 m 浮点值；温度同时提供摄氏度与开尔文。
 */

#ifndef DVC_MVR2TB_H
#define DVC_MVR2TB_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/UART/drv_uart.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// 已知协议响应的最大内容长度。当前手册中最大值为 4，预留到 16。
#define MVR2TB_RESPONSE_CONTENT_MAX_SIZE 16U
// 自定义命令允许的最大内容长度。
#define MVR2TB_TX_CONTENT_MAX_SIZE 16U
// 0x56 + CMD + LEN + CONTENT + XOR。
#define MVR2TB_TX_FRAME_MAX_SIZE (MVR2TB_TX_CONTENT_MAX_SIZE + 4U)
// 二进制流重组缓冲区。
#define MVR2TB_RX_BINARY_BUFFER_SIZE (MVR2TB_RESPONSE_CONTENT_MAX_SIZE + 4U)
// ASCII 一行最大长度（不含结尾空字符）。
#define MVR2TB_RX_ASCII_BUFFER_SIZE 96U
// ASCII 模组 ID 保存长度（含结尾空字符）。
#define MVR2TB_MODULE_ID_BUFFER_SIZE 16U
// 版本号原始内容固定预留 4 字节加结尾空字符。
#define MVR2TB_VERSION_RAW_BUFFER_SIZE 5U
// 格式化版本号，例如 M1.0.7。
#define MVR2TB_VERSION_FORMATTED_BUFFER_SIZE 8U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 模组通信在线状态
 */
enum Enum_MVR2TB_Status : uint8_t
{
    MVR2TB_Status_DISABLE = 0,
    MVR2TB_Status_ENABLE,
};

/**
 * @brief UART 测量数据输出格式
 */
enum Enum_MVR2TB_Output_Format : uint8_t
{
    MVR2TB_Output_Format_HEX = 0,
    MVR2TB_Output_Format_ASCII = 1,
    MVR2TB_Output_Format_UNKNOWN = 0xff,
};

/**
 * @brief 模组初始化后的测量模式
 */
enum Enum_MVR2TB_Measurement_Mode : uint8_t
{
    MVR2TB_Measurement_Mode_SINGLE = 0,
    MVR2TB_Measurement_Mode_CONTINUOUS = 1,
    MVR2TB_Measurement_Mode_UNKNOWN = 0xff,
};

/**
 * @brief 模组连续测量频率，对应命令字
 */
enum Enum_MVR2TB_Measurement_Frequency : uint8_t
{
    MVR2TB_Measurement_Frequency_10HZ = 0x31,
    MVR2TB_Measurement_Frequency_20HZ = 0x32,
    MVR2TB_Measurement_Frequency_50HZ = 0x33,
    MVR2TB_Measurement_Frequency_100HZ = 0x34,
    MVR2TB_Measurement_Frequency_UNKNOWN = 0xff,
};

/**
 * @brief HEX 帧 Byte9 的照明驱动器状态
 */
enum Enum_MVR2TB_Illumination_Status : uint8_t
{
    MVR2TB_Illumination_Status_DAC_LOW = 0,
    MVR2TB_Illumination_Status_DAC_HIGH = 1,
    MVR2TB_Illumination_Status_UNKNOWN = 0xff,
};

/**
 * @brief 协议命令字
 */
enum Enum_MVR2TB_Command : uint8_t
{
    MVR2TB_Command_SINGLE_MEASUREMENT = 0x02,
    MVR2TB_Command_READ_IO_THRESHOLD = 0x03,
    MVR2TB_Command_STOP_CONTINUOUS_MEASUREMENT = 0x06,
    MVR2TB_Command_READ_HARDWARE_VERSION = 0x08,
    MVR2TB_Command_READ_FIRMWARE_VERSION = 0x0e,
    MVR2TB_Command_SELECT_SINGLE_MODE = 0x20,
    MVR2TB_Command_SELECT_CONTINUOUS_MODE = 0x21,
    MVR2TB_Command_SET_FREQUENCY_10HZ = 0x31,
    MVR2TB_Command_SET_FREQUENCY_20HZ = 0x32,
    MVR2TB_Command_SET_FREQUENCY_50HZ = 0x33,
    MVR2TB_Command_SET_FREQUENCY_100HZ = 0x34,
    MVR2TB_Command_ENTER_STANDBY = 0x40,
    MVR2TB_Command_START_CONTINUOUS_MEASUREMENT = 0x50,
    MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_DISABLE = 0x60,
    MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_ENABLE = 0x61,
    MVR2TB_Command_OUTPUT_HEX = 0x70,
    MVR2TB_Command_OUTPUT_ASCII = 0x71,
    MVR2TB_Command_MEASUREMENT_DATA = 0x81,
    MVR2TB_Command_SET_IO_THRESHOLD = 0xda,
    MVR2TB_Command_UNKNOWN_RESPONSE = 0xdd,
};

/**
 * @brief 最近一次成功解析的数据包类型
 */
enum Enum_MVR2TB_Rx_Packet_Type : uint8_t
{
    MVR2TB_Rx_Packet_Type_NONE = 0,
    MVR2TB_Rx_Packet_Type_MEASUREMENT_HEX,
    MVR2TB_Rx_Packet_Type_MEASUREMENT_ASCII,
    MVR2TB_Rx_Packet_Type_COMMAND_RESPONSE,
    MVR2TB_Rx_Packet_Type_UNKNOWN_COMMAND_RESPONSE,
};

/**
 * @brief 命令响应结果
 */
enum Enum_MVR2TB_Response_Result : uint8_t
{
    MVR2TB_Response_Result_NONE = 0,
    MVR2TB_Response_Result_ACKNOWLEDGED,
    MVR2TB_Response_Result_UNKNOWN_COMMAND,
};

/**
 * @brief 经过换算的测量数据
 */
struct Struct_MVR2TB_Measurement_Data
{
    bool Valid;
    Enum_MVR2TB_Output_Format Source_Format;

    // 距离，协议原始单位为 mm。
    uint16_t Distance_MM;
    float Distance_Meter;

    // 信号幅度，单位 LSB。
    uint16_t Signal_Amplitude;

    // 温度。
    float Temperature_Celsius;
    float Temperature_Kelvin;

    // 环境光强度，单位 LSB。
    uint16_t Ambient_Light;

    // HEX 模式为 Byte9；ASCII 模式保留完整数值。
    uint16_t Illumination_DAC;
    Enum_MVR2TB_Illumination_Status Illumination_Status;

    // ASCII 输出可能携带模组 ID；HEX 输出时为空或保留最近一次 ASCII ID。
    char Module_ID[MVR2TB_MODULE_ID_BUFFER_SIZE];

    // 每解析出一帧有效测量数据递增。
    uint32_t Sequence;
};

/**
 * @brief 通用命令响应
 */
struct Struct_MVR2TB_Response_Data
{
    bool Valid;
    Enum_MVR2TB_Response_Result Result;

    // 响应帧中的 CMD。
    uint8_t Command;
    // 根据待确认命令反推的原始命令；例如模式选择 0x20/0x21 的响应 CMD 固定为 0x50。
    uint8_t Matched_Tx_Command;

    uint8_t Content_Length;
    uint8_t Content[MVR2TB_RESPONSE_CONTENT_MAX_SIZE];
    uint8_t Checksum;
    uint32_t Sequence;
};

/**
 * @brief 固件或硬件版本信息
 */
struct Struct_MVR2TB_Version_Data
{
    bool Valid;
    uint8_t Content_Length;
    char Raw[MVR2TB_VERSION_RAW_BUFFER_SIZE];
    char Formatted[MVR2TB_VERSION_FORMATTED_BUFFER_SIZE];
};

/**
 * @brief 接收解析统计，用于现场排查串口噪声和分包问题
 */
struct Struct_MVR2TB_Parser_Statistics
{
    uint32_t Valid_HEX_Measurement_Count;
    uint32_t Valid_ASCII_Measurement_Count;
    uint32_t Valid_Response_Count;
    // HEX 测量样例的校验值未包含 Byte9；兼容该样例时递增。
    uint32_t Legacy_Measurement_Checksum_Count;
    uint32_t Checksum_Error_Count;
    uint32_t Malformed_Frame_Count;
    uint32_t Binary_Buffer_Overflow_Count;
    uint32_t ASCII_Buffer_Overflow_Count;
};

/**
 * @brief MVR2TB 测距模组驱动
 */
class Class_MVR2TB
{
public:
    /**
     * @brief 初始化并绑定 UART
     *
     * @param huart UART 句柄
     * @param __Offline_Timeout_100ms 连续多少个 100 ms 周期未收到有效帧后判定离线，最小为 1
     * @param __Initial_Output_Format 上电时对模组输出格式的先验值，模组默认 HEX
     */
    void Init(UART_HandleTypeDef *huart,
              const uint16_t &__Offline_Timeout_100ms = 5,
              const Enum_MVR2TB_Output_Format &__Initial_Output_Format = MVR2TB_Output_Format_HEX);

    inline Enum_MVR2TB_Status Get_Status() const;
    inline Enum_MVR2TB_Rx_Packet_Type Get_Last_Rx_Packet_Type() const;
    inline uint64_t Get_Last_Rx_Time_Stamp() const;

    inline const Struct_MVR2TB_Measurement_Data &Get_Measurement_Data() const;
    inline bool Get_Measurement_Updated() const;
    inline uint32_t Get_Measurement_Count() const;
    inline uint16_t Get_Now_Distance_MM() const;
    inline float Get_Now_Distance() const;
    inline uint16_t Get_Now_Signal_Amplitude() const;
    inline float Get_Now_Temperature() const;
    inline float Get_Now_Temperature_Celsius() const;
    inline uint16_t Get_Now_Ambient_Light() const;
    inline uint16_t Get_Now_Illumination_DAC() const;
    inline Enum_MVR2TB_Illumination_Status Get_Illumination_Status() const;
    inline const char *Get_Module_ID() const;

    inline const Struct_MVR2TB_Response_Data &Get_Response_Data() const;
    inline const Struct_MVR2TB_Version_Data &Get_Firmware_Version_Data() const;
    inline const Struct_MVR2TB_Version_Data &Get_Hardware_Version_Data() const;
    inline const char *Get_Firmware_Version() const;
    inline const char *Get_Firmware_Version_Raw() const;
    inline const char *Get_Hardware_Version() const;
    inline const char *Get_Hardware_Version_Raw() const;

    inline bool Get_IO_Threshold_Valid() const;
    inline uint16_t Get_IO_Threshold_MM() const;

    inline Enum_MVR2TB_Output_Format Get_Output_Format() const;
    inline Enum_MVR2TB_Measurement_Mode Get_Measurement_Mode() const;
    inline Enum_MVR2TB_Measurement_Frequency Get_Measurement_Frequency() const;
    inline bool Get_Continuous_Measurement_Running() const;
    inline bool Get_Standby() const;
    inline bool Get_Power_On_Auto_Measurement_Known() const;
    inline bool Get_Power_On_Auto_Measurement() const;

    inline uint8_t Get_Last_Tx_Command() const;
    inline uint8_t Get_Last_Acknowledged_Command() const;
    inline const Struct_MVR2TB_Parser_Statistics &Get_Parser_Statistics() const;

    inline void Clear_Measurement_Updated();
    void Clear_Parser_Statistics();
    void Reset_Parser();

    /**
     * @brief UART DMA 空闲中断回调入口，可接收任意长度分包/粘包
     */
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

    /**
     * @brief 100 ms 周期在线检测回调
     */
    void TIM_100ms_Alive_PeriodElapsedCallback();

    // 协议命令 ---------------------------------------------------------------

    uint8_t UART_Send_Select_Measurement_Mode(const Enum_MVR2TB_Measurement_Mode &Mode);
    uint8_t UART_Send_Start_Continuous_Measurement();
    uint8_t UART_Send_Stop_Continuous_Measurement();
    uint8_t UART_Send_Single_Measurement();
    uint8_t UART_Send_Set_Output_Format(const Enum_MVR2TB_Output_Format &Format);
    uint8_t UART_Send_Enter_Standby();

    /**
     * @brief 发送一个字节唤醒待机模组。唤醒后应等待模组稳定，再发送单次测量命令。
     */
    uint8_t UART_Send_Wake_Up(const uint8_t &Wake_Byte = 0x00);

    uint8_t UART_Send_Read_Firmware_Version();
    uint8_t UART_Send_Read_Hardware_Version();
    uint8_t UART_Send_Set_Power_On_Auto_Measurement(const bool &Enable);
    uint8_t UART_Send_Set_Measurement_Frequency(const Enum_MVR2TB_Measurement_Frequency &Frequency);
    uint8_t UART_Send_Read_IO_Threshold();
    uint8_t UART_Send_Set_IO_Threshold(const uint16_t &Threshold_MM);

    /**
     * @brief 发送自定义协议命令，帧头、长度和 XOR 由驱动生成
     */
    uint8_t UART_Send_Custom_Command(const uint8_t &Command,
                                     const uint8_t *Content = nullptr,
                                     const uint8_t &Content_Length = 0);

    /**
     * @brief 计算一段数据的逐字节 XOR
     */
    static uint8_t Calculate_XOR(const uint8_t *Data, const uint16_t &Length);

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
    UART_HandleTypeDef *UART_Handler = nullptr;

    Struct_MVR2TB_Measurement_Data Measurement_Data = {};
    Struct_MVR2TB_Response_Data Response_Data = {};
    Struct_MVR2TB_Version_Data Firmware_Version = {};
    Struct_MVR2TB_Version_Data Hardware_Version = {};
    Struct_MVR2TB_Parser_Statistics Parser_Statistics = {};

    Enum_MVR2TB_Status Sensor_Status = MVR2TB_Status_DISABLE;
    Enum_MVR2TB_Rx_Packet_Type Last_Rx_Packet_Type = MVR2TB_Rx_Packet_Type_NONE;
    Enum_MVR2TB_Output_Format Output_Format = MVR2TB_Output_Format_UNKNOWN;
    Enum_MVR2TB_Measurement_Mode Measurement_Mode = MVR2TB_Measurement_Mode_UNKNOWN;
    Enum_MVR2TB_Measurement_Frequency Measurement_Frequency = MVR2TB_Measurement_Frequency_UNKNOWN;

    bool Measurement_Updated = false;
    bool Continuous_Measurement_Running = false;
    bool Standby = false;
    bool Power_On_Auto_Measurement_Known = false;
    bool Power_On_Auto_Measurement = false;

    bool IO_Threshold_Valid = false;
    uint16_t IO_Threshold_MM = 0;
    bool Pending_IO_Threshold_Valid = false;
    uint16_t Pending_IO_Threshold_MM = 0;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint16_t Offline_Timeout_100ms = 5;
    uint16_t Offline_Countdown_100ms = 0;
    uint64_t Last_Rx_Time_Stamp = 0;

    uint8_t Last_Tx_Command = 0xff;
    uint8_t Pending_Command = 0xff;
    uint8_t Last_Acknowledged_Command = 0xff;

    // HAL_UART_Transmit_DMA 在发送完成前持续访问原缓冲区，故使用双持久缓冲。
    uint8_t Tx_Buffer[2][MVR2TB_TX_FRAME_MAX_SIZE] = {};
    uint8_t Tx_Buffer_Next = 0;

    uint8_t Binary_Buffer[MVR2TB_RX_BINARY_BUFFER_SIZE] = {};
    uint8_t Binary_Length = 0;
    uint8_t Binary_Expected_Length = 0;

    char ASCII_Buffer[MVR2TB_RX_ASCII_BUFFER_SIZE + 1U] = {};
    uint16_t ASCII_Length = 0;

    void Process_Rx_Byte(const uint8_t &Byte);
    void Process_Binary_Byte(const uint8_t &Byte);
    void Process_ASCII_Byte(const uint8_t &Byte);
    void Resynchronize_Binary_Buffer();

    bool Parse_Binary_Frame(const uint8_t *Frame, const uint8_t &Length);
    bool Parse_HEX_Measurement(const uint8_t *Frame, const uint8_t &Length);
    bool Parse_Command_Response(const uint8_t *Frame, const uint8_t &Length);
    bool Parse_ASCII_Measurement(char *Line, const uint16_t &Length);

    void Update_Measurement(const uint16_t &Distance_MM,
                            const uint16_t &Signal_Amplitude,
                            const float &Temperature_Celsius,
                            const uint16_t &Ambient_Light,
                            const uint16_t &Illumination_DAC,
                            const Enum_MVR2TB_Output_Format &Source_Format,
                            const char *Module_ID = nullptr);

    void Parse_Version(const uint8_t *Content,
                       const uint8_t &Content_Length,
                       Struct_MVR2TB_Version_Data *Version_Data);

    uint8_t Match_Response_Command(const uint8_t &Response_Command) const;
    void Handle_Acknowledgement(const uint8_t &Effective_Command,
                                const uint8_t *Content,
                                const uint8_t &Content_Length);
    void Mark_Valid_Packet(const Enum_MVR2TB_Rx_Packet_Type &Packet_Type);
};

/* Inline functions ----------------------------------------------------------*/

inline Enum_MVR2TB_Status Class_MVR2TB::Get_Status() const
{
    return (Sensor_Status);
}

inline Enum_MVR2TB_Rx_Packet_Type Class_MVR2TB::Get_Last_Rx_Packet_Type() const
{
    return (Last_Rx_Packet_Type);
}

inline uint64_t Class_MVR2TB::Get_Last_Rx_Time_Stamp() const
{
    return (Last_Rx_Time_Stamp);
}

inline const Struct_MVR2TB_Measurement_Data &Class_MVR2TB::Get_Measurement_Data() const
{
    return (Measurement_Data);
}

inline bool Class_MVR2TB::Get_Measurement_Updated() const
{
    return (Measurement_Updated);
}

inline uint32_t Class_MVR2TB::Get_Measurement_Count() const
{
    return (Measurement_Data.Sequence);
}

inline uint16_t Class_MVR2TB::Get_Now_Distance_MM() const
{
    return (Measurement_Data.Distance_MM);
}

inline float Class_MVR2TB::Get_Now_Distance() const
{
    return (Measurement_Data.Distance_Meter);
}

inline uint16_t Class_MVR2TB::Get_Now_Signal_Amplitude() const
{
    return (Measurement_Data.Signal_Amplitude);
}

inline float Class_MVR2TB::Get_Now_Temperature() const
{
    return (Measurement_Data.Temperature_Kelvin);
}

inline float Class_MVR2TB::Get_Now_Temperature_Celsius() const
{
    return (Measurement_Data.Temperature_Celsius);
}

inline uint16_t Class_MVR2TB::Get_Now_Ambient_Light() const
{
    return (Measurement_Data.Ambient_Light);
}

inline uint16_t Class_MVR2TB::Get_Now_Illumination_DAC() const
{
    return (Measurement_Data.Illumination_DAC);
}

inline Enum_MVR2TB_Illumination_Status Class_MVR2TB::Get_Illumination_Status() const
{
    return (Measurement_Data.Illumination_Status);
}

inline const char *Class_MVR2TB::Get_Module_ID() const
{
    return (Measurement_Data.Module_ID);
}

inline const Struct_MVR2TB_Response_Data &Class_MVR2TB::Get_Response_Data() const
{
    return (Response_Data);
}

inline const Struct_MVR2TB_Version_Data &Class_MVR2TB::Get_Firmware_Version_Data() const
{
    return (Firmware_Version);
}

inline const Struct_MVR2TB_Version_Data &Class_MVR2TB::Get_Hardware_Version_Data() const
{
    return (Hardware_Version);
}

inline const char *Class_MVR2TB::Get_Firmware_Version() const
{
    return (Firmware_Version.Formatted);
}

inline const char *Class_MVR2TB::Get_Firmware_Version_Raw() const
{
    return (Firmware_Version.Raw);
}

inline const char *Class_MVR2TB::Get_Hardware_Version() const
{
    return (Hardware_Version.Formatted);
}

inline const char *Class_MVR2TB::Get_Hardware_Version_Raw() const
{
    return (Hardware_Version.Raw);
}

inline bool Class_MVR2TB::Get_IO_Threshold_Valid() const
{
    return (IO_Threshold_Valid);
}

inline uint16_t Class_MVR2TB::Get_IO_Threshold_MM() const
{
    return (IO_Threshold_MM);
}

inline Enum_MVR2TB_Output_Format Class_MVR2TB::Get_Output_Format() const
{
    return (Output_Format);
}

inline Enum_MVR2TB_Measurement_Mode Class_MVR2TB::Get_Measurement_Mode() const
{
    return (Measurement_Mode);
}

inline Enum_MVR2TB_Measurement_Frequency Class_MVR2TB::Get_Measurement_Frequency() const
{
    return (Measurement_Frequency);
}

inline bool Class_MVR2TB::Get_Continuous_Measurement_Running() const
{
    return (Continuous_Measurement_Running);
}

inline bool Class_MVR2TB::Get_Standby() const
{
    return (Standby);
}

inline bool Class_MVR2TB::Get_Power_On_Auto_Measurement_Known() const
{
    return (Power_On_Auto_Measurement_Known);
}

inline bool Class_MVR2TB::Get_Power_On_Auto_Measurement() const
{
    return (Power_On_Auto_Measurement);
}

inline uint8_t Class_MVR2TB::Get_Last_Tx_Command() const
{
    return (Last_Tx_Command);
}

inline uint8_t Class_MVR2TB::Get_Last_Acknowledged_Command() const
{
    return (Last_Acknowledged_Command);
}

inline const Struct_MVR2TB_Parser_Statistics &Class_MVR2TB::Get_Parser_Statistics() const
{
    return (Parser_Statistics);
}

inline void Class_MVR2TB::Clear_Measurement_Updated()
{
    Measurement_Updated = false;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
