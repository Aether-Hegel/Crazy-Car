#ifndef __PWM_H__
#define __PWM_H__

#include "zf_common_headfile.h"

// PWM 通道引脚定义
#define PWM_CH1_L1                 (PWM2_MODULE0_CHA_C6)
#define PWM_CH2_R1                 (PWM2_MODULE1_CHA_C8)
#define PWM_CH3_L2                 (PWM2_MODULE2_CHB_C11)
#define PWM_CH4_R2                 (PWM2_MODULE3_CHA_D2) 

void MyPWM_Init(void);

void PWM_CH1_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH2_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH3_Set_Duty(pwm_channel_enum pin, const uint32 duty);
void PWM_CH4_Set_Duty(pwm_channel_enum pin, const uint32 duty);


#endif // __PWM_H__
