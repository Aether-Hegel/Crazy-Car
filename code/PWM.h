#ifndef __PWM_H__
#define __PWM_H__

#include "zf_common_headfile.h"

// PWM 通道引脚定义
#define PWM_CH1_L1                 (PWM1_MODULE3_CHA_D0)
#define PWM_CH2_R1                 (PWM1_MODULE3_CHB_D1)
#define PWM_CH3_L2                 (PWM2_MODULE3_CHA_D2)
#define PWM_CH4_R2                 (PWM2_MODULE3_CHB_D3) 

void MyPWM_Init(void);

void PWM_CH1_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH2_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH3_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH4_Set_Duty(pwm_channel_enum pin, const uint32 duty);


#endif // __PWM_H__
