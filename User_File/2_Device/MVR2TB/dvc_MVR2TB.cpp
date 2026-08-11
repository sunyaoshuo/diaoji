/**
 * @file dvc_MVR2TB.cpp
 * @author Project integration
 * @brief MOEVISION MVR2TB TOF 激光测距模组 UART 驱动
 * @version 0.1
 * @date 2026-07-20 0.1 根据《MVR2TB_Datasheet_CHS_V1.0.4》新增
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_MVR2TB.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

#define MVR2TB_COMMAND_FRAME_HEADER 0x56U
#define MVR2TB_RESPONSE_FRAME_HEADER 0x89U
#define MVR2TB_HEX_MEASUREMENT_FRAME_SIZE 11U
#define MVR2TB_INVALID_COMMAND 0xffU
#define MVR2TB_KELVIN_OFFSET 273.15f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

namespace
{

inline uint16_t MVR2TB_Read_LE_U16(const uint8_t *Data)
{
    return (static_cast<uint16_t>(Data[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(Data[1]) << 8U));
}

inline bool MVR2TB_Is_Printable_ASCII(const uint8_t Byte)
{
    return ((Byte >= 0x20U) && (Byte <= 0x7eU));
}

inline bool MVR2TB_Is_Alphabetic(const char Character)
{
    return (((Character >= 'A') && (Character <= 'Z')) ||
            ((Character >= 'a') && (Character <= 'z')));
}

char *MVR2TB_Trim_ASCII_Token(char *Token)
{
    if (Token == nullptr)
    {
        return (nullptr);
    }

    while ((*Token == ' ') || (*Token == '\t'))
    {
        ++Token;
    }

    char *End = Token + strlen(Token);
    while ((End > Token) && ((End[-1] == ' ') || (End[-1] == '\t')))
    {
        --End;
    }
    *End = '\0';

    return (Token);
}

bool MVR2TB_Parse_Signed_Integer(const char *Text, int32_t *Value, uint8_t *Digit_Count = nullptr)
{
    if ((Text == nullptr) || (Value == nullptr) || (*Text == '\0'))
    {
        return (false);
    }

    bool Negative = false;
    if ((*Text == '+') || (*Text == '-'))
    {
        Negative = (*Text == '-');
        ++Text;
    }

    if (*Text == '\0')
    {
        return (false);
    }

    int64_t Result = 0;
    const int64_t Limit = Negative ? 2147483648LL : 2147483647LL;
    uint8_t Digits = 0;
    while (*Text != '\0')
    {
        if ((*Text < '0') || (*Text > '9'))
        {
            return (false);
        }

        const int64_t Digit = static_cast<int64_t>(*Text - '0');
        if (Result > ((Limit - Digit) / 10LL))
        {
            return (false);
        }

        Result = Result * 10LL + Digit;
        ++Digits;
        ++Text;
    }

    *Value = Negative ?
                 static_cast<int32_t>(-Result) :
                 static_cast<int32_t>(Result);

    if (Digit_Count != nullptr)
    {
        *Digit_Count = Digits;
    }

    return (true);
}

bool MVR2TB_Parse_Unsigned_U16(const char *Text, uint16_t *Value)
{
    int32_t Signed_Value = 0;
    if (!MVR2TB_Parse_Signed_Integer(Text, &Signed_Value))
    {
        return (false);
    }

    if ((Signed_Value < 0) || (Signed_Value > 65535))
    {
        return (false);
    }

    *Value = static_cast<uint16_t>(Signed_Value);
    return (true);
}

bool MVR2TB_Parse_Decimal(const char *Text, float *Value)
{
    if ((Text == nullptr) || (Value == nullptr) || (*Text == '\0'))
    {
        return (false);
    }

    bool Negative = false;
    if ((*Text == '+') || (*Text == '-'))
    {
        Negative = (*Text == '-');
        ++Text;
    }

    bool Has_Digit = false;
    float Result = 0.0f;
    while ((*Text >= '0') && (*Text <= '9'))
    {
        Result = Result * 10.0f + static_cast<float>(*Text - '0');
        Has_Digit = true;
        ++Text;
    }

    if (*Text == '.')
    {
        ++Text;
        float Scale = 0.1f;
        while ((*Text >= '0') && (*Text <= '9'))
        {
            Result += static_cast<float>(*Text - '0') * Scale;
            Scale *= 0.1f;
            Has_Digit = true;
            ++Text;
        }
    }

    if ((!Has_Digit) || (*Text != '\0'))
    {
        return (false);
    }

    *Value = Negative ? -Result : Result;
    return (true);
}

bool MVR2TB_Parse_ASCII_Temperature(const char *Text, float *Temperature_Celsius)
{
    if ((Text == nullptr) || (Temperature_Celsius == nullptr))
    {
        return (false);
    }

    const char *Cursor = Text;
    while (*Cursor != '\0')
    {
        if (*Cursor == '.')
        {
            return (MVR2TB_Parse_Decimal(Text, Temperature_Celsius));
        }
        ++Cursor;
    }

    int32_t Raw_Temperature = 0;
    uint8_t Digit_Count = 0;
    if (!MVR2TB_Parse_Signed_Integer(Text, &Raw_Temperature, &Digit_Count))
    {
        return (false);
    }

    // 手册表格示例 04737 表示 47.37 ℃，GUI 截图又出现 41 这种整摄氏度格式。
    if ((Digit_Count >= 4U) || (Raw_Temperature > 150) || (Raw_Temperature < -150))
    {
        *Temperature_Celsius = static_cast<float>(Raw_Temperature) * 0.01f;
    }
    else
    {
        *Temperature_Celsius = static_cast<float>(Raw_Temperature);
    }

    return (true);
}

void MVR2TB_Copy_String(char *Destination, const uint16_t Destination_Size, const char *Source)
{
    if ((Destination == nullptr) || (Destination_Size == 0U))
    {
        return;
    }

    uint16_t Index = 0;
    if (Source != nullptr)
    {
        while ((Source[Index] != '\0') && (Index + 1U < Destination_Size))
        {
            Destination[Index] = Source[Index];
            ++Index;
        }
    }
    Destination[Index] = '\0';
}

} // namespace

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化并绑定 UART
 */
void Class_MVR2TB::Init(UART_HandleTypeDef *huart,
                        const uint16_t &__Offline_Timeout_100ms,
                        const Enum_MVR2TB_Output_Format &__Initial_Output_Format)
{
    UART_Handler = huart;
    UART_Manage_Object = nullptr;

    if (huart != nullptr)
    {
        if (huart->Instance == USART1)
        {
            UART_Manage_Object = &UART1_Manage_Object;
        }
        else if (huart->Instance == USART2)
        {
            UART_Manage_Object = &UART2_Manage_Object;
        }
        else if (huart->Instance == USART3)
        {
            UART_Manage_Object = &UART3_Manage_Object;
        }
        else if (huart->Instance == UART4)
        {
            UART_Manage_Object = &UART4_Manage_Object;
        }
        else if (huart->Instance == UART5)
        {
            UART_Manage_Object = &UART5_Manage_Object;
        }
        else if (huart->Instance == USART6)
        {
            UART_Manage_Object = &UART6_Manage_Object;
        }
        else if (huart->Instance == UART7)
        {
            UART_Manage_Object = &UART7_Manage_Object;
        }
        else if (huart->Instance == UART8)
        {
            UART_Manage_Object = &UART8_Manage_Object;
        }
        else if (huart->Instance == UART9)
        {
            UART_Manage_Object = &UART9_Manage_Object;
        }
        else if (huart->Instance == USART10)
        {
            UART_Manage_Object = &UART10_Manage_Object;
        }
    }

    memset(&Measurement_Data, 0, sizeof(Measurement_Data));
    memset(&Response_Data, 0, sizeof(Response_Data));
    memset(&Firmware_Version, 0, sizeof(Firmware_Version));
    memset(&Hardware_Version, 0, sizeof(Hardware_Version));
    memset(&Parser_Statistics, 0, sizeof(Parser_Statistics));
    memset(Tx_Buffer, 0, sizeof(Tx_Buffer));

    Measurement_Data.Source_Format = MVR2TB_Output_Format_UNKNOWN;
    Measurement_Data.Illumination_Status = MVR2TB_Illumination_Status_UNKNOWN;
    Response_Data.Result = MVR2TB_Response_Result_NONE;
    Response_Data.Matched_Tx_Command = MVR2TB_INVALID_COMMAND;

    Sensor_Status = MVR2TB_Status_DISABLE;
    Last_Rx_Packet_Type = MVR2TB_Rx_Packet_Type_NONE;
    Output_Format = __Initial_Output_Format;
    Measurement_Mode = MVR2TB_Measurement_Mode_UNKNOWN;
    Measurement_Frequency = MVR2TB_Measurement_Frequency_UNKNOWN;

    Measurement_Updated = false;
    Continuous_Measurement_Running = false;
    Standby = false;
    Power_On_Auto_Measurement_Known = false;
    Power_On_Auto_Measurement = false;

    IO_Threshold_Valid = false;
    IO_Threshold_MM = 0;
    Pending_IO_Threshold_Valid = false;
    Pending_IO_Threshold_MM = 0;

    Flag = 0;
    Pre_Flag = 0;
    Offline_Timeout_100ms = (__Offline_Timeout_100ms == 0U) ? 1U : __Offline_Timeout_100ms;
    Offline_Countdown_100ms = 0;
    Last_Rx_Time_Stamp = 0;

    Last_Tx_Command = MVR2TB_INVALID_COMMAND;
    Pending_Command = MVR2TB_INVALID_COMMAND;
    Last_Acknowledged_Command = MVR2TB_INVALID_COMMAND;
    Tx_Buffer_Next = 0;

    Reset_Parser();
}

/**
 * @brief 清空解析统计
 */
void Class_MVR2TB::Clear_Parser_Statistics()
{
    memset(&Parser_Statistics, 0, sizeof(Parser_Statistics));
}

/**
 * @brief 清空未完成的流解析状态，不清除已经解析的数据
 */
void Class_MVR2TB::Reset_Parser()
{
    memset(Binary_Buffer, 0, sizeof(Binary_Buffer));
    Binary_Length = 0;
    Binary_Expected_Length = 0;

    memset(ASCII_Buffer, 0, sizeof(ASCII_Buffer));
    ASCII_Length = 0;
}

/**
 * @brief 计算逐字节 XOR
 */
uint8_t Class_MVR2TB::Calculate_XOR(const uint8_t *Data, const uint16_t &Length)
{
    if ((Data == nullptr) && (Length != 0U))
    {
        return (0U);
    }

    uint8_t Result = 0U;
    for (uint16_t Index = 0; Index < Length; ++Index)
    {
        Result ^= Data[Index];
    }
    return (Result);
}

/**
 * @brief 发送自定义命令
 */
uint8_t Class_MVR2TB::UART_Send_Custom_Command(const uint8_t &Command,
                                               const uint8_t *Content,
                                               const uint8_t &Content_Length)
{
    if ((UART_Handler == nullptr) ||
        (Content_Length > MVR2TB_TX_CONTENT_MAX_SIZE) ||
        ((Content_Length != 0U) && (Content == nullptr)))
    {
        return (static_cast<uint8_t>(HAL_ERROR));
    }

    const uint8_t Buffer_Index = static_cast<uint8_t>(Tx_Buffer_Next & 0x01U);
    uint8_t *Frame = Tx_Buffer[Buffer_Index];

    Frame[0] = MVR2TB_COMMAND_FRAME_HEADER;
    Frame[1] = Command;
    Frame[2] = Content_Length;
    for (uint8_t Index = 0; Index < Content_Length; ++Index)
    {
        Frame[3U + Index] = Content[Index];
    }

    const uint16_t Frame_Length = static_cast<uint16_t>(Content_Length) + 4U;
    Frame[Frame_Length - 1U] = Calculate_XOR(Frame, Frame_Length - 1U);

    const uint8_t Result = UART_Transmit_Data(UART_Handler, Frame, Frame_Length);
    if (Result == static_cast<uint8_t>(HAL_OK))
    {
        Tx_Buffer_Next ^= 0x01U;
        Last_Tx_Command = Command;

        // 只跟踪一个待确认命令。若应用在前一命令确认前又成功发送新命令，
        // 与旧阈值写入相关的暂存值不再用于后续 ACK。
        if ((Pending_Command == MVR2TB_Command_SET_IO_THRESHOLD) &&
            (Command != MVR2TB_Command_SET_IO_THRESHOLD))
        {
            Pending_IO_Threshold_Valid = false;
        }
        Pending_Command = Command;
    }

    return (Result);
}

/**
 * @brief 选择单次/连续测量模式
 */
uint8_t Class_MVR2TB::UART_Send_Select_Measurement_Mode(const Enum_MVR2TB_Measurement_Mode &Mode)
{
    if (Mode == MVR2TB_Measurement_Mode_SINGLE)
    {
        return (UART_Send_Custom_Command(MVR2TB_Command_SELECT_SINGLE_MODE));
    }
    if (Mode == MVR2TB_Measurement_Mode_CONTINUOUS)
    {
        return (UART_Send_Custom_Command(MVR2TB_Command_SELECT_CONTINUOUS_MODE));
    }
    return (static_cast<uint8_t>(HAL_ERROR));
}

uint8_t Class_MVR2TB::UART_Send_Start_Continuous_Measurement()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_START_CONTINUOUS_MEASUREMENT));
}

uint8_t Class_MVR2TB::UART_Send_Stop_Continuous_Measurement()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_STOP_CONTINUOUS_MEASUREMENT));
}

uint8_t Class_MVR2TB::UART_Send_Single_Measurement()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_SINGLE_MEASUREMENT));
}

uint8_t Class_MVR2TB::UART_Send_Set_Output_Format(const Enum_MVR2TB_Output_Format &Format)
{
    if (Format == MVR2TB_Output_Format_HEX)
    {
        return (UART_Send_Custom_Command(MVR2TB_Command_OUTPUT_HEX));
    }
    if (Format == MVR2TB_Output_Format_ASCII)
    {
        return (UART_Send_Custom_Command(MVR2TB_Command_OUTPUT_ASCII));
    }
    return (static_cast<uint8_t>(HAL_ERROR));
}

uint8_t Class_MVR2TB::UART_Send_Enter_Standby()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_ENTER_STANDBY));
}

/**
 * @brief 发送一个原始字节唤醒待机模组
 */
uint8_t Class_MVR2TB::UART_Send_Wake_Up(const uint8_t &Wake_Byte)
{
    if (UART_Handler == nullptr)
    {
        return (static_cast<uint8_t>(HAL_ERROR));
    }

    const uint8_t Buffer_Index = static_cast<uint8_t>(Tx_Buffer_Next & 0x01U);
    Tx_Buffer[Buffer_Index][0] = Wake_Byte;

    const uint8_t Result = UART_Transmit_Data(UART_Handler, Tx_Buffer[Buffer_Index], 1U);
    if (Result == static_cast<uint8_t>(HAL_OK))
    {
        Tx_Buffer_Next ^= 0x01U;
        Last_Tx_Command = Wake_Byte;
        Standby = false;
    }
    return (Result);
}

uint8_t Class_MVR2TB::UART_Send_Read_Firmware_Version()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_READ_FIRMWARE_VERSION));
}

uint8_t Class_MVR2TB::UART_Send_Read_Hardware_Version()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_READ_HARDWARE_VERSION));
}

uint8_t Class_MVR2TB::UART_Send_Set_Power_On_Auto_Measurement(const bool &Enable)
{
    return (UART_Send_Custom_Command(Enable ?
                                         MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_ENABLE :
                                         MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_DISABLE));
}

uint8_t Class_MVR2TB::UART_Send_Set_Measurement_Frequency(const Enum_MVR2TB_Measurement_Frequency &Frequency)
{
    switch (Frequency)
    {
    case MVR2TB_Measurement_Frequency_10HZ:
    case MVR2TB_Measurement_Frequency_20HZ:
    case MVR2TB_Measurement_Frequency_50HZ:
    case MVR2TB_Measurement_Frequency_100HZ:
        return (UART_Send_Custom_Command(static_cast<uint8_t>(Frequency)));

    default:
        return (static_cast<uint8_t>(HAL_ERROR));
    }
}

uint8_t Class_MVR2TB::UART_Send_Read_IO_Threshold()
{
    return (UART_Send_Custom_Command(MVR2TB_Command_READ_IO_THRESHOLD));
}

uint8_t Class_MVR2TB::UART_Send_Set_IO_Threshold(const uint16_t &Threshold_MM)
{
    const uint8_t Content[4] = {
        static_cast<uint8_t>(Threshold_MM & 0xffU),
        static_cast<uint8_t>((Threshold_MM >> 8U) & 0xffU),
        0x00U,
        0x00U,
    };

    const uint8_t Result = UART_Send_Custom_Command(MVR2TB_Command_SET_IO_THRESHOLD,
                                                     Content,
                                                     static_cast<uint8_t>(sizeof(Content)));
    if (Result == static_cast<uint8_t>(HAL_OK))
    {
        Pending_IO_Threshold_MM = Threshold_MM;
        Pending_IO_Threshold_Valid = true;
    }
    return (Result);
}

/**
 * @brief UART DMA 空闲回调入口
 */
void Class_MVR2TB::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if ((Buffer == nullptr) || (Length == 0U))
    {
        return;
    }

    for (uint16_t Index = 0; Index < Length; ++Index)
    {
        Process_Rx_Byte(Buffer[Index]);
    }
}

/**
 * @brief 处理一个串口字节
 */
void Class_MVR2TB::Process_Rx_Byte(const uint8_t &Byte)
{
    if (Binary_Length != 0U)
    {
        Process_Binary_Byte(Byte);
        return;
    }

    if (Byte == MVR2TB_RESPONSE_FRAME_HEADER)
    {
        if (ASCII_Length != 0U)
        {
            ++Parser_Statistics.Malformed_Frame_Count;
            ASCII_Length = 0U;
            ASCII_Buffer[0] = '\0';
        }

        Binary_Buffer[0] = Byte;
        Binary_Length = 1U;
        Binary_Expected_Length = 0U;
        return;
    }

    Process_ASCII_Byte(Byte);
}

/**
 * @brief 处理二进制响应流
 */
void Class_MVR2TB::Process_Binary_Byte(const uint8_t &Byte)
{
    if (Binary_Length >= MVR2TB_RX_BINARY_BUFFER_SIZE)
    {
        ++Parser_Statistics.Binary_Buffer_Overflow_Count;
        Binary_Length = 0U;
        Binary_Expected_Length = 0U;
        Process_Rx_Byte(Byte);
        return;
    }

    Binary_Buffer[Binary_Length] = Byte;
    ++Binary_Length;

    if (Binary_Length == 2U)
    {
        if (Binary_Buffer[1] == MVR2TB_Command_MEASUREMENT_DATA)
        {
            Binary_Expected_Length = MVR2TB_HEX_MEASUREMENT_FRAME_SIZE;
        }
    }
    else if ((Binary_Length == 3U) && (Binary_Expected_Length == 0U))
    {
        const uint8_t Content_Length = Binary_Buffer[2];
        if (Content_Length > MVR2TB_RESPONSE_CONTENT_MAX_SIZE)
        {
            ++Parser_Statistics.Malformed_Frame_Count;
            Resynchronize_Binary_Buffer();
            return;
        }
        Binary_Expected_Length = static_cast<uint8_t>(Content_Length + 4U);
    }

    if ((Binary_Expected_Length != 0U) && (Binary_Length == Binary_Expected_Length))
    {
        const uint8_t Received_Checksum = Binary_Buffer[Binary_Length - 1U];
        const uint8_t Expected_Checksum = Calculate_XOR(Binary_Buffer,
                                                         static_cast<uint16_t>(Binary_Length - 1U));

        bool Checksum_Valid = (Expected_Checksum == Received_Checksum);
        if ((!Checksum_Valid) &&
            (Binary_Buffer[1] == MVR2TB_Command_MEASUREMENT_DATA) &&
            (Binary_Length == MVR2TB_HEX_MEASUREMENT_FRAME_SIZE))
        {
            // 手册第 7 页样例 89 81 ... 01 99 的 0x99 实际只异或到 Byte8，
            // 未包含 Byte9(ILLB)。为兼容该公开样例和可能采用相同固件的模组，
            // HEX 测量帧同时接受“异或 Byte0~Byte8”的校验方式。
            const uint8_t Legacy_Checksum = Calculate_XOR(Binary_Buffer,
                                                            static_cast<uint16_t>(Binary_Length - 2U));
            if (Legacy_Checksum == Received_Checksum)
            {
                Checksum_Valid = true;
                ++Parser_Statistics.Legacy_Measurement_Checksum_Count;
            }
        }

        if (!Checksum_Valid)
        {
            ++Parser_Statistics.Checksum_Error_Count;
            Resynchronize_Binary_Buffer();
            return;
        }

        if (!Parse_Binary_Frame(Binary_Buffer, Binary_Length))
        {
            ++Parser_Statistics.Malformed_Frame_Count;
        }

        Binary_Length = 0U;
        Binary_Expected_Length = 0U;
    }
}

/**
 * @brief 校验失败后在当前缓冲区中寻找下一个 0x89 重新同步
 */
void Class_MVR2TB::Resynchronize_Binary_Buffer()
{
    uint8_t Retry_Buffer[MVR2TB_RX_BINARY_BUFFER_SIZE] = {};
    uint8_t Retry_Length = 0U;

    for (uint8_t Index = 1U; Index < Binary_Length; ++Index)
    {
        if (Binary_Buffer[Index] == MVR2TB_RESPONSE_FRAME_HEADER)
        {
            Retry_Length = static_cast<uint8_t>(Binary_Length - Index);
            memcpy(Retry_Buffer, &Binary_Buffer[Index], Retry_Length);
            break;
        }
    }

    Binary_Length = 0U;
    Binary_Expected_Length = 0U;

    for (uint8_t Index = 0U; Index < Retry_Length; ++Index)
    {
        Process_Rx_Byte(Retry_Buffer[Index]);
    }
}

/**
 * @brief 处理 ASCII 测量行
 */
void Class_MVR2TB::Process_ASCII_Byte(const uint8_t &Byte)
{
    if ((Byte == '\r') || (Byte == '\n'))
    {
        if (ASCII_Length != 0U)
        {
            ASCII_Buffer[ASCII_Length] = '\0';
            if (!Parse_ASCII_Measurement(ASCII_Buffer, ASCII_Length))
            {
                ++Parser_Statistics.Malformed_Frame_Count;
            }
            ASCII_Length = 0U;
            ASCII_Buffer[0] = '\0';
        }
        return;
    }

    if (MVR2TB_Is_Printable_ASCII(Byte) || (Byte == '\t'))
    {
        if (ASCII_Length >= MVR2TB_RX_ASCII_BUFFER_SIZE)
        {
            ++Parser_Statistics.ASCII_Buffer_Overflow_Count;
            ASCII_Length = 0U;
            ASCII_Buffer[0] = '\0';
            return;
        }

        ASCII_Buffer[ASCII_Length] = static_cast<char>(Byte);
        ++ASCII_Length;
        ASCII_Buffer[ASCII_Length] = '\0';
        return;
    }

    if (ASCII_Length != 0U)
    {
        ++Parser_Statistics.Malformed_Frame_Count;
        ASCII_Length = 0U;
        ASCII_Buffer[0] = '\0';
    }
}

/**
 * @brief 解析一帧已通过 XOR 校验的二进制数据
 */
bool Class_MVR2TB::Parse_Binary_Frame(const uint8_t *Frame, const uint8_t &Length)
{
    if ((Frame == nullptr) || (Length < 4U) || (Frame[0] != MVR2TB_RESPONSE_FRAME_HEADER))
    {
        return (false);
    }

    if (Frame[1] == MVR2TB_Command_MEASUREMENT_DATA)
    {
        return (Parse_HEX_Measurement(Frame, Length));
    }

    return (Parse_Command_Response(Frame, Length));
}

/**
 * @brief 解析固定 11 字节 HEX 测量帧
 */
bool Class_MVR2TB::Parse_HEX_Measurement(const uint8_t *Frame, const uint8_t &Length)
{
    if ((Frame == nullptr) ||
        (Length != MVR2TB_HEX_MEASUREMENT_FRAME_SIZE) ||
        (Frame[0] != MVR2TB_RESPONSE_FRAME_HEADER) ||
        (Frame[1] != MVR2TB_Command_MEASUREMENT_DATA))
    {
        return (false);
    }

    const uint16_t Distance_MM = MVR2TB_Read_LE_U16(&Frame[2]);
    const uint16_t Signal_Amplitude = MVR2TB_Read_LE_U16(&Frame[4]);
    const int8_t Temperature_Raw = static_cast<int8_t>(Frame[6]);
    const uint16_t Ambient_Light = MVR2TB_Read_LE_U16(&Frame[7]);
    const uint16_t Illumination_DAC = Frame[9];

    Update_Measurement(Distance_MM,
                       Signal_Amplitude,
                       static_cast<float>(Temperature_Raw),
                       Ambient_Light,
                       Illumination_DAC,
                       MVR2TB_Output_Format_HEX);
    return (true);
}

/**
 * @brief 解析通用命令响应
 */
bool Class_MVR2TB::Parse_Command_Response(const uint8_t *Frame, const uint8_t &Length)
{
    if ((Frame == nullptr) || (Length < 4U))
    {
        return (false);
    }

    const uint8_t Command = Frame[1];
    const uint8_t Content_Length = Frame[2];
    if ((Content_Length > MVR2TB_RESPONSE_CONTENT_MAX_SIZE) ||
        (Length != static_cast<uint8_t>(Content_Length + 4U)))
    {
        return (false);
    }

    Response_Data.Valid = true;
    Response_Data.Result = (Command == MVR2TB_Command_UNKNOWN_RESPONSE) ?
                               MVR2TB_Response_Result_UNKNOWN_COMMAND :
                               MVR2TB_Response_Result_ACKNOWLEDGED;
    Response_Data.Command = Command;
    Response_Data.Matched_Tx_Command = Match_Response_Command(Command);
    Response_Data.Content_Length = Content_Length;
    memset(Response_Data.Content, 0, sizeof(Response_Data.Content));
    if (Content_Length != 0U)
    {
        memcpy(Response_Data.Content, &Frame[3], Content_Length);
    }
    Response_Data.Checksum = Frame[Length - 1U];
    ++Response_Data.Sequence;

    ++Parser_Statistics.Valid_Response_Count;
    Mark_Valid_Packet((Command == MVR2TB_Command_UNKNOWN_RESPONSE) ?
                          MVR2TB_Rx_Packet_Type_UNKNOWN_COMMAND_RESPONSE :
                          MVR2TB_Rx_Packet_Type_COMMAND_RESPONSE);

    if (Command == MVR2TB_Command_UNKNOWN_RESPONSE)
    {
        if (Response_Data.Matched_Tx_Command == MVR2TB_Command_SET_IO_THRESHOLD)
        {
            Pending_IO_Threshold_Valid = false;
        }
        Pending_Command = MVR2TB_INVALID_COMMAND;
        return (true);
    }

    uint8_t Effective_Command = Command;
    if (Response_Data.Matched_Tx_Command != MVR2TB_INVALID_COMMAND)
    {
        Effective_Command = Response_Data.Matched_Tx_Command;
        Last_Acknowledged_Command = Effective_Command;
        Pending_Command = MVR2TB_INVALID_COMMAND;
    }
    else
    {
        Last_Acknowledged_Command = Command;
    }

    Handle_Acknowledgement(Effective_Command, &Frame[3], Content_Length);
    return (true);
}

/**
 * @brief 解析逗号分隔、回车结尾的 ASCII 测量数据
 */
bool Class_MVR2TB::Parse_ASCII_Measurement(char *Line, const uint16_t &Length)
{
    if ((Line == nullptr) || (Length == 0U))
    {
        return (false);
    }

    Line[Length] = '\0';

    char *Tokens[8] = {};
    uint8_t Token_Count = 0U;
    char *Token_Start = Line;

    for (uint16_t Index = 0U; Index <= Length; ++Index)
    {
        if ((Line[Index] == ',') || (Line[Index] == '\0'))
        {
            Line[Index] = '\0';
            char *Token = MVR2TB_Trim_ASCII_Token(Token_Start);
            if ((Token != nullptr) && (*Token != '\0'))
            {
                if (Token_Count >= static_cast<uint8_t>(sizeof(Tokens) / sizeof(Tokens[0])))
                {
                    return (false);
                }
                Tokens[Token_Count] = Token;
                ++Token_Count;
            }
            Token_Start = &Line[Index + 1U];
        }
    }

    if (Token_Count < 5U)
    {
        return (false);
    }

    uint8_t Field_Index = 0U;
    const char *Module_ID = nullptr;
    if (MVR2TB_Is_Alphabetic(Tokens[0][0]))
    {
        Module_ID = Tokens[0];
        Field_Index = 1U;
    }

    if (Token_Count < static_cast<uint8_t>(Field_Index + 5U))
    {
        return (false);
    }

    uint16_t Distance_MM = 0U;
    uint16_t Signal_Amplitude = 0U;
    float Temperature_Celsius = 0.0f;
    uint16_t Ambient_Light = 0U;
    uint16_t Illumination_DAC = 0U;

    if (!MVR2TB_Parse_Unsigned_U16(Tokens[Field_Index], &Distance_MM) ||
        !MVR2TB_Parse_Unsigned_U16(Tokens[Field_Index + 1U], &Signal_Amplitude) ||
        !MVR2TB_Parse_ASCII_Temperature(Tokens[Field_Index + 2U], &Temperature_Celsius) ||
        !MVR2TB_Parse_Unsigned_U16(Tokens[Field_Index + 3U], &Ambient_Light) ||
        !MVR2TB_Parse_Unsigned_U16(Tokens[Field_Index + 4U], &Illumination_DAC))
    {
        return (false);
    }

    Update_Measurement(Distance_MM,
                       Signal_Amplitude,
                       Temperature_Celsius,
                       Ambient_Light,
                       Illumination_DAC,
                       MVR2TB_Output_Format_ASCII,
                       Module_ID);
    return (true);
}

/**
 * @brief 更新经过换算的测量数据
 */
void Class_MVR2TB::Update_Measurement(const uint16_t &Distance_MM,
                                      const uint16_t &Signal_Amplitude,
                                      const float &Temperature_Celsius,
                                      const uint16_t &Ambient_Light,
                                      const uint16_t &Illumination_DAC,
                                      const Enum_MVR2TB_Output_Format &Source_Format,
                                      const char *Module_ID)
{
    Measurement_Data.Valid = true;
    Measurement_Data.Source_Format = Source_Format;
    Measurement_Data.Distance_MM = Distance_MM;
    Measurement_Data.Distance_Meter = static_cast<float>(Distance_MM) * 0.001f;
    Measurement_Data.Signal_Amplitude = Signal_Amplitude;
    Measurement_Data.Temperature_Celsius = Temperature_Celsius;
    Measurement_Data.Temperature_Kelvin = Temperature_Celsius + MVR2TB_KELVIN_OFFSET;
    Measurement_Data.Ambient_Light = Ambient_Light;
    Measurement_Data.Illumination_DAC = Illumination_DAC;

    if (Illumination_DAC == 0U)
    {
        Measurement_Data.Illumination_Status = MVR2TB_Illumination_Status_DAC_LOW;
    }
    else if (Illumination_DAC == 1U)
    {
        Measurement_Data.Illumination_Status = MVR2TB_Illumination_Status_DAC_HIGH;
    }
    else
    {
        Measurement_Data.Illumination_Status = MVR2TB_Illumination_Status_UNKNOWN;
    }

    if ((Module_ID != nullptr) && (*Module_ID != '\0'))
    {
        MVR2TB_Copy_String(Measurement_Data.Module_ID,
                           MVR2TB_MODULE_ID_BUFFER_SIZE,
                           Module_ID);
    }

    ++Measurement_Data.Sequence;
    Measurement_Updated = true;
    Output_Format = Source_Format;
    Standby = false;

    const bool Single_Measurement_Pending =
        (Pending_Command == MVR2TB_Command_SINGLE_MEASUREMENT);

    if ((Measurement_Mode == MVR2TB_Measurement_Mode_CONTINUOUS) ||
        ((Measurement_Mode == MVR2TB_Measurement_Mode_UNKNOWN) &&
         (!Single_Measurement_Pending)))
    {
        // 模组出厂默认可能直接连续上报，此时尚未执行模式查询/设置，
        // 可根据非单次请求触发的测量流推断连续测量正在运行。
        Continuous_Measurement_Running = true;
    }

    if (Single_Measurement_Pending)
    {
        Last_Acknowledged_Command = MVR2TB_Command_SINGLE_MEASUREMENT;
        Pending_Command = MVR2TB_INVALID_COMMAND;
    }

    if (Source_Format == MVR2TB_Output_Format_HEX)
    {
        ++Parser_Statistics.Valid_HEX_Measurement_Count;
        Mark_Valid_Packet(MVR2TB_Rx_Packet_Type_MEASUREMENT_HEX);
    }
    else
    {
        ++Parser_Statistics.Valid_ASCII_Measurement_Count;
        Mark_Valid_Packet(MVR2TB_Rx_Packet_Type_MEASUREMENT_ASCII);
    }
}

/**
 * @brief 解析版本号内容
 */
void Class_MVR2TB::Parse_Version(const uint8_t *Content,
                                 const uint8_t &Content_Length,
                                 Struct_MVR2TB_Version_Data *Version_Data)
{
    if (Version_Data == nullptr)
    {
        return;
    }

    memset(Version_Data, 0, sizeof(*Version_Data));
    Version_Data->Content_Length = Content_Length;

    const uint8_t Copy_Length = (Content_Length < 4U) ? Content_Length : 4U;
    for (uint8_t Index = 0U; Index < Copy_Length; ++Index)
    {
        const uint8_t Character = Content[Index];
        Version_Data->Raw[Index] = MVR2TB_Is_Printable_ASCII(Character) ?
                                       static_cast<char>(Character) :
                                       '?';
    }
    Version_Data->Raw[Copy_Length] = '\0';

    if (Content_Length >= 4U)
    {
        Version_Data->Formatted[0] = Version_Data->Raw[0];
        Version_Data->Formatted[1] = Version_Data->Raw[1];
        Version_Data->Formatted[2] = '.';
        Version_Data->Formatted[3] = Version_Data->Raw[2];
        Version_Data->Formatted[4] = '.';
        Version_Data->Formatted[5] = Version_Data->Raw[3];
        Version_Data->Formatted[6] = '\0';
        Version_Data->Valid = true;
    }
}

/**
 * @brief 将响应 CMD 与最近待确认命令匹配
 */
uint8_t Class_MVR2TB::Match_Response_Command(const uint8_t &Response_Command) const
{
    if (Pending_Command == MVR2TB_INVALID_COMMAND)
    {
        return (MVR2TB_INVALID_COMMAND);
    }

    if (Response_Command == MVR2TB_Command_UNKNOWN_RESPONSE)
    {
        return (Pending_Command);
    }

    if (Response_Command == Pending_Command)
    {
        return (Pending_Command);
    }

    // 0x20、0x21 测量模式选择命令的响应 CMD 均固定为 0x50。
    if ((Response_Command == MVR2TB_Command_START_CONTINUOUS_MEASUREMENT) &&
        ((Pending_Command == MVR2TB_Command_SELECT_SINGLE_MODE) ||
         (Pending_Command == MVR2TB_Command_SELECT_CONTINUOUS_MODE) ||
         (Pending_Command == MVR2TB_Command_START_CONTINUOUS_MEASUREMENT)))
    {
        return (Pending_Command);
    }

    return (MVR2TB_INVALID_COMMAND);
}

/**
 * @brief 根据有效命令响应更新驱动状态
 */
void Class_MVR2TB::Handle_Acknowledgement(const uint8_t &Effective_Command,
                                          const uint8_t *Content,
                                          const uint8_t &Content_Length)
{
    switch (Effective_Command)
    {
    case MVR2TB_Command_SELECT_SINGLE_MODE:
        Measurement_Mode = MVR2TB_Measurement_Mode_SINGLE;
        Continuous_Measurement_Running = false;
        break;

    case MVR2TB_Command_SELECT_CONTINUOUS_MODE:
        Measurement_Mode = MVR2TB_Measurement_Mode_CONTINUOUS;
        Continuous_Measurement_Running = false;
        break;

    case MVR2TB_Command_START_CONTINUOUS_MEASUREMENT:
        Continuous_Measurement_Running = true;
        Standby = false;
        break;

    case MVR2TB_Command_STOP_CONTINUOUS_MEASUREMENT:
        Continuous_Measurement_Running = false;
        break;

    case MVR2TB_Command_OUTPUT_HEX:
        Output_Format = MVR2TB_Output_Format_HEX;
        break;

    case MVR2TB_Command_OUTPUT_ASCII:
        Output_Format = MVR2TB_Output_Format_ASCII;
        break;

    case MVR2TB_Command_ENTER_STANDBY:
        Standby = true;
        Continuous_Measurement_Running = false;
        break;

    case MVR2TB_Command_READ_FIRMWARE_VERSION:
        Parse_Version(Content, Content_Length, &Firmware_Version);
        break;

    case MVR2TB_Command_READ_HARDWARE_VERSION:
        Parse_Version(Content, Content_Length, &Hardware_Version);
        break;

    case MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_DISABLE:
        Power_On_Auto_Measurement_Known = true;
        Power_On_Auto_Measurement = false;
        break;

    case MVR2TB_Command_POWER_ON_AUTO_MEASUREMENT_ENABLE:
        Power_On_Auto_Measurement_Known = true;
        Power_On_Auto_Measurement = true;
        break;

    case MVR2TB_Command_SET_FREQUENCY_10HZ:
        Measurement_Frequency = MVR2TB_Measurement_Frequency_10HZ;
        break;

    case MVR2TB_Command_SET_FREQUENCY_20HZ:
        Measurement_Frequency = MVR2TB_Measurement_Frequency_20HZ;
        break;

    case MVR2TB_Command_SET_FREQUENCY_50HZ:
        Measurement_Frequency = MVR2TB_Measurement_Frequency_50HZ;
        break;

    case MVR2TB_Command_SET_FREQUENCY_100HZ:
        Measurement_Frequency = MVR2TB_Measurement_Frequency_100HZ;
        break;

    case MVR2TB_Command_READ_IO_THRESHOLD:
        if ((Content != nullptr) && (Content_Length >= 2U))
        {
            IO_Threshold_MM = MVR2TB_Read_LE_U16(Content);
            IO_Threshold_Valid = true;
        }
        break;

    case MVR2TB_Command_SET_IO_THRESHOLD:
        if (Pending_IO_Threshold_Valid)
        {
            IO_Threshold_MM = Pending_IO_Threshold_MM;
            IO_Threshold_Valid = true;
            Pending_IO_Threshold_Valid = false;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 标记收到有效数据包
 */
void Class_MVR2TB::Mark_Valid_Packet(const Enum_MVR2TB_Rx_Packet_Type &Packet_Type)
{
    ++Flag;
    Sensor_Status = MVR2TB_Status_ENABLE;
    Offline_Countdown_100ms = Offline_Timeout_100ms;
    Last_Rx_Packet_Type = Packet_Type;

    if (UART_Manage_Object != nullptr)
    {
        Last_Rx_Time_Stamp = UART_Manage_Object->Rx_Time_Stamp;
    }
}

/**
 * @brief 100 ms 周期在线检测
 */
void Class_MVR2TB::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag != Pre_Flag)
    {
        Sensor_Status = MVR2TB_Status_ENABLE;
        Offline_Countdown_100ms = Offline_Timeout_100ms;
    }
    else if (Offline_Countdown_100ms != 0U)
    {
        --Offline_Countdown_100ms;
    }

    if (Offline_Countdown_100ms == 0U)
    {
        Sensor_Status = MVR2TB_Status_DISABLE;
    }

    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
