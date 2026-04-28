#include "PWM.h"

/*
占空比在 PWM_DUTY_MAX（zf_driver_pwm.c） 设置最大值范围，现已定为10000，占空比：0~10000 由16位寄存器计数设置占空比
*/

void MyPWM_Init(void)
{
    pwm_init(PWM_CH1_L1, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH2_R1, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH3_L2, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH4_R2, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
}

