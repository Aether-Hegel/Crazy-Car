#include "Motor.h"
#include "PWM.h"

//================================DIR引脚控制电机正反转================================
// 电机正转：DIR引脚为高电平
// 电机反转：DIR引脚为低电平
//====================================================================================

void Motor_Init(void)
{
    // GPIO口初始化 推挽输出 初始化为低电平
    gpio_init(MOTOR_L1_DIR, GPO, 0, GPO_PUSH_PULL); // L1_DIR
    gpio_init(MOTOR_L2_DIR, GPO, 0, GPO_PUSH_PULL); // L2_DIR
    gpio_init(MOTOR_R1_DIR, GPO, 0, GPO_PUSH_PULL); // R1_DIR
    gpio_init(MOTOR_R2_DIR, GPO, 0, GPO_PUSH_PULL); // R2_DIR
}

//===============================电机控制麦轮左移==========================================
// 左前电机正转，左后电机反转
// 右前电机正转，右后电机反转
//========================================================================================
void Car_Left_Move(void)
{
    gpio_set_level(MOTOR_L1_DIR, 1); // L1_DIR = 1
    gpio_set_level(MOTOR_L2_DIR, 0); // L2_DIR = 0
    gpio_set_level(MOTOR_R1_DIR, 1); // R1_DIR = 1
    gpio_set_level(MOTOR_R2_DIR, 0); // R2_DIR = 0
}

//===============================电机控制麦轮右移==========================================
// 左前电机反转，左后电机正转
// 右前电机反转，右后电机正转
//========================================================================================
void Car_Right_Move(void)
{
    gpio_set_level(MOTOR_L1_DIR, 0); // L1_DIR = 0
    gpio_set_level(MOTOR_L2_DIR, 1); // L2_DIR = 1
    gpio_set_level(MOTOR_R1_DIR, 0); // R1_DIR = 0
    gpio_set_level(MOTOR_R2_DIR, 1); // R2_DIR = 1
}

//===============================电机控制前进==========================================
// 左前电机正转，左后电机正转
// 右前电机正转，右后电机正转
//=====================================================================================
void Car_Forward_Move(void)
{
    gpio_set_level(MOTOR_L1_DIR, 1); // L1_DIR = 1
    gpio_set_level(MOTOR_L2_DIR, 1); // L2_DIR = 1
    gpio_set_level(MOTOR_R1_DIR, 1); // R1_DIR = 1
    gpio_set_level(MOTOR_R2_DIR, 1); // R2_DIR = 1
}

//===============================电机控制后退==========================================
// 左前电机反转，左后电机反转
// 右前电机反转，右后电机反转
//=====================================================================================
void Car_Backward_Move(void)
{
    gpio_set_level(MOTOR_L1_DIR, 0); // L1_DIR = 0
    gpio_set_level(MOTOR_L2_DIR, 0); // L2_DIR = 0
    gpio_set_level(MOTOR_R1_DIR, 0); // R1_DIR = 0
    gpio_set_level(MOTOR_R2_DIR, 0); // R2_DIR = 0
}

void Car_Move(void)
{
    uint8 command = 0; //控制小车移动方向(最后将这个参数作为函数的输入参数)
    switch (command)
    {
        case 0:
            Car_Forward_Move(); // 前进
            break;
        case 1:
            Car_Backward_Move(); // 后退
            break;
        case 2:
            Car_Left_Move(); // 左移
            break;
        case 3:
            Car_Right_Move(); // 右移
            break;
        default:
            // 停止或其他操作
            break;
    }

    PWM_CH1_Set_Duty(PWM_CH1_L1, 5000); // 设置左前电机占空比
    PWM_CH2_Set_Duty(PWM_CH2_R1, 5000); // 设置右前电机占空比
    PWM_CH3_Set_Duty(PWM_CH3_L2, 5000); // 设置左后电机占空比
    PWM_CH4_Set_Duty(PWM_CH4_R2, 5000); // 设置右后电机占空比

}