#ifndef ALG_MU_PLATFORM_H
#define ALG_MU_PLATFORM_H

#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"

#include <cstdint>

struct Struct_MU_Target_Pose
{
    float Z_mm = 90.0f;
    float Pitch_deg = 0.0f;
    float Roll_deg = 0.0f;
};

struct Struct_MU_Joint_Angles
{
    float Theta_1_rad = 0.0f;
    float Theta_2_rad = 0.0f;
    float Theta_3_rad = 0.0f;
};

class Class_MU_Platform
{
public:
    void Init();
    void Loop();

    void TIM_1ms_PeriodElapsedCallback();
    void TIM_10us_PeriodElapsedCallback();
    void TIM_125us_PeriodElapsedCallback();
    void EXTI_Flag_Callback(uint16_t GPIO_Pin);
    void SPI2_RxCpltCallback();
    void CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

private:
    struct Struct_Kinematics_Params
    {
        float RB_mm = 46.0f;
        float RP_mm = 93.128f;
        float L1_mm = 52.0f;
        float L2_mm = 72.0f;
        Class_Matrix_f32<3, 1> P_Local[3];
        float Cos_Leg[3] = {0.0f, 0.0f, 0.0f};
        float Sin_Leg[3] = {0.0f, 0.0f, 0.0f};
    };

    static constexpr uint32_t CONTROL_PERIOD_MS = 10;

    Class_Motor_DM_Normal Motor_1;
    Class_Motor_DM_Normal Motor_2;
    Class_Motor_DM_Normal Motor_3;
    Struct_Kinematics_Params Kinematics;
    Struct_MU_Target_Pose Target;
    Struct_MU_Joint_Angles Joint_Angles;

    volatile bool Control_Enabled = false;
    volatile bool IK_Valid = false;
    uint32_t Last_Control_ms = 0;

    void Init_Kinematics();
    bool Solve_Single_Leg(float R_mm, float Z_mm, float &Theta_rad) const;
    bool Solve_IK(float Z_mm, float Pitch_rad, float Roll_rad,
                  Struct_MU_Joint_Angles &Output) const;
    void Update_Control(uint32_t Now_ms);
};

extern Class_MU_Platform MU_Platform;

#endif
