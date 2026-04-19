#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

// Motor Pin 控制电机正反转引脚

#define MOTOR_L1_DIR  C9
#define MOTOR_L2_DIR  C10
#define MOTOR_R1_DIR  C7
#define MOTOR_R2_DIR  D2

void Motor_Init(void);
void Car_Left_Move(void);
void Car_Right_Move(void);
void Car_Forward_Move(void);
void Car_Backward_Move(void);
void Car_Move(void);

#endif // __MOTOR_H__
