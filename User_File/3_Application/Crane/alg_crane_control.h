/**
 * @file alg_crane_control.h
 * @brief 吊机四自由度控制、USART1 上位机协议与安全互锁
 */

#ifndef ALG_CRANE_CONTROL_H
#define ALG_CRANE_CONTROL_H

#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Motor/Motor_RS/dvc_motor_RS.h"

#include <stdint.h>

/**
 * @note 以下参数是保守的软件限幅，联机前应按真实机械行程、减速比和负载重新标定。
 *       上位机协议中的角度统一使用 0.01 degree（厘度），电机驱动内部仍使用 rad。
 */
static constexpr int32_t CRANE_EXTENSION_MIN_CDEG = -360000;   // -3600 deg
static constexpr int32_t CRANE_EXTENSION_MAX_CDEG = 360000;    //  3600 deg
static constexpr int32_t CRANE_WINCH_MIN_CDEG = -3600000;      // -36000 deg
static constexpr int32_t CRANE_WINCH_MAX_CDEG = 3600000;       //  36000 deg
static constexpr int32_t CRANE_YAW_MIN_CDEG = -70000;          // -700 deg
static constexpr int32_t CRANE_YAW_MAX_CDEG = 70000;           //  700 deg

enum Enum_Crane_Pitch_Direction : int8_t
{
    Crane_Pitch_Direction_DOWN = -1,
    Crane_Pitch_Direction_STOP = 0,
    Crane_Pitch_Direction_UP = 1,
};

enum Enum_Crane_Startup_State : uint8_t
{
    Crane_Startup_State_DISABLED = 0,
    Crane_Startup_State_EXTENSION_START,
    Crane_Startup_State_EXTENSION_REPORT,
    Crane_Startup_State_WINCH_START,
    Crane_Startup_State_WINCH_REPORT,
    Crane_Startup_State_YAW_START,
    Crane_Startup_State_ACTIVE,
};

class Class_Crane_Control
{
public:
    void Init();
    void Loop();
    void TIM_1ms_PeriodElapsedCallback();

    void CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);
    void UART1_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

private:
    static constexpr uint8_t RS_MASTER_ID = 0xFD;
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
    static constexpr uint32_t PITCH_DEADMAN_TIMEOUT_MS = 300;
    static constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
    static constexpr uint32_t STARTUP_STEP_DELAY_MS = 10;
    static constexpr uint32_t MOTOR_ONLINE_TIMEOUT_MS = 500;
    static constexpr uint16_t UART_DMA_SIZE = 256;
    static constexpr uint16_t UART_RING_SIZE = 512;
    static constexpr uint16_t UART_LINE_SIZE = 96;

    Class_Motor_RS Extension_Motor;
    Class_Motor_RS Winch_Motor;
    Class_Motor_DM_Normal Yaw_Motor;

    volatile bool Safety_Requested = false;
    volatile bool Outputs_Enabled = false;
    volatile bool Telemetry_Requested = false;
    volatile Enum_Crane_Startup_State Startup_State = Crane_Startup_State_DISABLED;
    volatile Enum_Crane_Pitch_Direction Pitch_Direction = Crane_Pitch_Direction_STOP;

    uint32_t Startup_Deadline_ms = 0;
    uint32_t Last_Valid_Command_ms = 0;
    uint32_t Last_Pitch_Command_ms = 0;
    uint32_t Last_Telemetry_ms = 0;

    volatile bool Extension_Feedback_Received = false;
    volatile bool Winch_Feedback_Received = false;
    volatile bool Yaw_Feedback_Received = false;
    volatile uint32_t Extension_Last_Feedback_ms = 0;
    volatile uint32_t Winch_Last_Feedback_ms = 0;
    volatile uint32_t Yaw_Last_Feedback_ms = 0;

    uint8_t UART_DMA_Buffer[UART_DMA_SIZE] = {0};
    uint16_t UART_DMA_Tail = 0;
    uint8_t UART_Ring[UART_RING_SIZE] = {0};
    volatile uint16_t UART_Ring_Head = 0;
    volatile uint16_t UART_Ring_Tail = 0;
    volatile uint32_t UART_Ring_Overflow_Count = 0;
    uint32_t UART_Tx_Error_Count = 0;
    char UART_Line[UART_LINE_SIZE] = {0};
    uint16_t UART_Line_Length = 0;

    void Process_UART();
    void Parse_Command(char *Line);
    void Request_Safety(bool Enable, uint32_t Now_ms);
    void Disable_Outputs(bool Transmit_Stop);
    void Handle_Startup(uint32_t Now_ms);
    void Apply_Pitch_Output(Enum_Crane_Pitch_Direction Direction);
    void Send_Telemetry(uint32_t Now_ms);

    static bool Time_Reached(uint32_t Now_ms, uint32_t Deadline_ms);
    static bool Feedback_Is_Online(bool Received, uint32_t Last_ms, uint32_t Now_ms);
    static int32_t Clamp_Centidegree(int32_t Value, int32_t Min, int32_t Max);
    static float Centidegree_To_Radian(int32_t Centidegree);
    static int32_t Radian_To_Centidegree(float Radian);
};

extern Class_Crane_Control Crane_Control;

#endif
