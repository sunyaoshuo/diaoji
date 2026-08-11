#include "alg_mu_platform.h"

#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/SPI/drv_spi.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "fdcan.h"
#include "spi.h"

#include <cmath>

namespace
{
constexpr float MU_PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = MU_PI / 180.0f;
constexpr float MIN_LENGTH_SQ_MM = 7403.5f;
constexpr float MAX_LENGTH_SQ_MM = 14613.183225f;
constexpr float MOTOR_ZERO_OFFSET_RAD = 0.5f;
constexpr float MOTOR_SPEED_LIMIT_RADPS = 25.0f;

void MU_CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    MU_Platform.CAN1_RxCpltCallback(Header, Buffer);
}

void MU_SPI2_Callback(uint8_t *, uint8_t *, uint16_t, uint16_t)
{
    MU_Platform.SPI2_RxCpltCallback();
}
}

Class_MU_Platform MU_Platform;

void Class_MU_Platform::Init()
{
    Init_Kinematics();

    Motor_1.Init(&hfdcan1, 0x00, 0x03, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    Motor_2.Init(&hfdcan1, 0x01, 0x04, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    Motor_3.Init(&hfdcan1, 0x02, 0x05, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    Motor_1.Set_Control_Omega(MOTOR_SPEED_LIMIT_RADPS);
    Motor_2.Set_Control_Omega(MOTOR_SPEED_LIMIT_RADPS);
    Motor_3.Set_Control_Omega(MOTOR_SPEED_LIMIT_RADPS);

    CAN_Init(&hfdcan1, MU_CAN1_Callback);
    SPI_Init(&hspi2, MU_SPI2_Callback);
    BSP_BMI088.Init();

    // Enter motor mode, save the current mechanical position as zero, then
    // start platform control. This writes each motor's Flash on power-up.
    Motor_1.CAN_Send_Enter();
    Motor_2.CAN_Send_Enter();
    Motor_3.CAN_Send_Enter();
    Namespace_SYS_Timestamp::Delay_Millisecond(10);
    Motor_1.CAN_Send_Save_Zero();
    Motor_2.CAN_Send_Save_Zero();
    Motor_3.CAN_Send_Save_Zero();
    Namespace_SYS_Timestamp::Delay_Millisecond(10);

    Last_Control_ms = HAL_GetTick();
    Control_Enabled = true;
}

void Class_MU_Platform::Loop()
{
    const uint32_t Now_ms = HAL_GetTick();
    if (Control_Enabled && ((Now_ms - Last_Control_ms) >= CONTROL_PERIOD_MS))
    {
        Last_Control_ms = Now_ms;
        Update_Control(Now_ms);
    }
}

void Class_MU_Platform::TIM_1ms_PeriodElapsedCallback()
{
    static uint8_t Divider_128ms = 0;
    if (++Divider_128ms >= 128)
    {
        Divider_128ms = 0;
        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }
}

void Class_MU_Platform::TIM_10us_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

void Class_MU_Platform::TIM_125us_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

void Class_MU_Platform::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin) ||
        (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin))
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

void Class_MU_Platform::SPI2_RxCpltCallback()
{
    if (((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port) &&
         (SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin)) ||
        ((SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port) &&
         (SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)))
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

void Class_MU_Platform::CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *)
{
    if (Header.IdType != FDCAN_STANDARD_ID)
    {
        return;
    }

    if (Header.Identifier == 0x00u)
    {
        Motor_1.CAN_RxCpltCallback();
    }
    else if (Header.Identifier == 0x01u)
    {
        Motor_2.CAN_RxCpltCallback();
    }
    else if (Header.Identifier == 0x02u)
    {
        Motor_3.CAN_RxCpltCallback();
    }
}

void Class_MU_Platform::Init_Kinematics()
{
    for (uint8_t Index = 0; Index < 3u; ++Index)
    {
        const float Leg_Angle = static_cast<float>(Index) * 120.0f * DEG_TO_RAD;
        Kinematics.Cos_Leg[Index] = std::cos(Leg_Angle);
        Kinematics.Sin_Leg[Index] = std::sin(Leg_Angle);
        Kinematics.P_Local[Index][0][0] = Kinematics.RP_mm * Kinematics.Cos_Leg[Index];
        Kinematics.P_Local[Index][1][0] = Kinematics.RP_mm * Kinematics.Sin_Leg[Index];
        Kinematics.P_Local[Index][2][0] = 0.0f;
    }
}

bool Class_MU_Platform::Solve_Single_Leg(float R_mm, float Z_mm, float &Theta_rad) const
{
    const float X = R_mm - Kinematics.RB_mm;
    const float Distance_Sq = X * X + Z_mm * Z_mm;
    if ((Distance_Sq < MIN_LENGTH_SQ_MM) || (Distance_Sq > MAX_LENGTH_SQ_MM))
    {
        return false;
    }

    float Cos_Elbow = (Distance_Sq - Kinematics.L1_mm * Kinematics.L1_mm -
                       Kinematics.L2_mm * Kinematics.L2_mm) /
                      (2.0f * Kinematics.L1_mm * Kinematics.L2_mm);
    if (Cos_Elbow < -1.0f) Cos_Elbow = -1.0f;
    if (Cos_Elbow > 1.0f) Cos_Elbow = 1.0f;
    const float Elbow = std::acos(Cos_Elbow);
    const float Phi_Total = std::atan2(Z_mm, X);
    const float Phi_Link = std::atan2(Kinematics.L2_mm * std::sin(Elbow),
                                     Kinematics.L1_mm + Kinematics.L2_mm * std::cos(Elbow));
    Theta_rad = Phi_Total - Phi_Link;
    return true;
}

bool Class_MU_Platform::Solve_IK(float Z_mm, float Pitch_rad, float Roll_rad,
                                 Struct_MU_Joint_Angles &Output) const
{
    Class_Matrix_f32<3, 1> Translation;
    Translation[0][0] = 0.0f;
    Translation[1][0] = 0.0f;
    Translation[2][0] = Z_mm;
    Class_Matrix_f32<3, 3> Rotation =
        Namespace_ALG_Matrix::From_Euler_Angle(0.0f, Pitch_rad, Roll_rad);

    float Theta[3] = {0.0f, 0.0f, 0.0f};
    for (uint8_t Index = 0; Index < 3u; ++Index)
    {
        const Class_Matrix_f32<3, 1> World = Rotation * Kinematics.P_Local[Index] + Translation;
        const float R_Projection = World[0][0] * Kinematics.Cos_Leg[Index] +
                                   World[1][0] * Kinematics.Sin_Leg[Index];
        if (!Solve_Single_Leg(R_Projection, World[2][0], Theta[Index]))
        {
            return false;
        }
    }

    Output.Theta_1_rad = Theta[0];
    Output.Theta_2_rad = Theta[1];
    Output.Theta_3_rad = Theta[2];
    return true;
}

void Class_MU_Platform::Update_Control(uint32_t)
{
    const float Pitch_rad = Target.Pitch_deg * DEG_TO_RAD + BSP_BMI088.Get_Euler_Angle()[1][0];
    const float Roll_rad = Target.Roll_deg * DEG_TO_RAD + BSP_BMI088.Get_Euler_Angle()[2][0];

    Struct_MU_Joint_Angles New_Angles;
    IK_Valid = Solve_IK(Target.Z_mm, Pitch_rad, Roll_rad, New_Angles);
    if (!IK_Valid)
    {
        return;
    }

    Joint_Angles = New_Angles;
    Motor_1.Set_Control_Angle(Joint_Angles.Theta_1_rad + MOTOR_ZERO_OFFSET_RAD);
    Motor_2.Set_Control_Angle(Joint_Angles.Theta_2_rad + MOTOR_ZERO_OFFSET_RAD);
    Motor_3.Set_Control_Angle(Joint_Angles.Theta_3_rad + MOTOR_ZERO_OFFSET_RAD);
    Motor_1.TIM_Send_PeriodElapsedCallback();
    Motor_2.TIM_Send_PeriodElapsedCallback();
    Motor_3.TIM_Send_PeriodElapsedCallback();
}
