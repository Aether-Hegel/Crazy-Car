#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "zf_common_headfile.h"
#include "UART.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float target_speed;  // 目标速度
    float Actual_speed;  // 实际速度
    float error_prev2;   // 前前次误差 (e(k-2))
    float error_last;    // 上一次误差 (e(k-1))
    float error_current; // 当前误差 (e(k))
    float output;        // 当前输出值 (u(k))
    float Pwm_Max_Out;   // 限幅输出值
    float integral_sum;  // 积分累积值
    float integral_limit;// 积分限幅值

} Speed_PID;

void Set_Speed_PID(Speed_PID *pid, float Kp, float Ki, float Kd, float target, float PWM_MAX_OUt);
void Speed_PID_Init();
void Speed_PID_Calculate(Speed_PID *pid, float current_speed);
void Speed_Car();

#endif // SPEED_PID_H
