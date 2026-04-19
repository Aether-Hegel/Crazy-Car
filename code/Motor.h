#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 *                              麦克纳姆轮电机驱动头文件
 *
 * 硬件说明:
 *   本模块驱动4个带方向控制的有刷直流电机, 安装麦克纳姆轮实现全向移动。
 *
 * 电机编号与安装位置 (俯视图):
 *
 *          车头方向 (Forward)
 *            ↑
 *     ┌──────────────┐
 *     │  L1      R1  │     L1 = 左前 (Left-Front)
 *     │              │     R1 = 右前 (Right-Front)
 *     │              │     L2 = 左后 (Left-Rear)
 *     │  L2      R2  │     R2 = 右后 (Right-Rear)
 *     └──────────────┘
 *
 * 麦克纳姆轮滚子安装方向 (俯视图, "/" 和 "\" 表示滚子轴线方向):
 *
 *     ┌──────────────┐
 *     │  \L1    R1/  │
 *     │              │
 *     │  /L2    R2\  │
 *     └──────────────┘
 *
 *   这是 "O型" 安装方式 (滚子轴线形成O形)。
 *
 * 方向控制:
 *   DIR引脚 = 1 → 电机正转 (轮子向前滚动)
 *   DIR引脚 = 0 → 电机反转 (轮子向后滚动)
 *   PWM占空比范围: 0 ~ 10000 (对应 0% ~ 100%)
 *
 ********************************************************************************************************************/

/* ========================= 电机方向控制引脚定义 ========================= */

#define MOTOR_L1_DIR  C9      /* 左前电机方向引脚 */
#define MOTOR_L2_DIR  C10     /* 左后电机方向引脚 */
#define MOTOR_R1_DIR  C7      /* 右前电机方向引脚 */
#define MOTOR_R2_DIR  D2      /* 右后电机方向引脚 */

/* ========================= PWM 占空比限制 ========================= */

#define MOTOR_PWM_MAX   10000   /* PWM最大占空比, 对应100% */
#define MOTOR_PWM_MIN   0       /* PWM最小占空比, 对应0%   */

/* ========================= 函数声明 ========================= */

/**
 * @brief  电机GPIO方向引脚初始化
 */
void Motor_Init(void);

/**
 * @brief  麦克纳姆轮全向运动控制 (核心函数)
 *
 * @param  vx       X轴速度分量 (左右方向), 范围 [-10000, +10000]
 *                  正值 = 向右平移, 负值 = 向左平移, 单位: PWM占空比当量
 * @param  vy       Y轴速度分量 (前后方向), 范围 [-10000, +10000]
 *                  正值 = 向前运动, 负值 = 向后运动, 单位: PWM占空比当量
 * @param  omega    旋转角速度分量, 范围 [-10000, +10000]
 *                  正值 = 逆时针旋转 (俯视), 负值 = 顺时针旋转, 单位: PWM占空比当量
 */
void Mecanum_Move(int32 vx, int32 vy, int32 omega);

/**
 * @brief  设置单个电机的转速和方向
 *
 * @param  dir_pin   电机方向引脚 (MOTOR_L1_DIR / MOTOR_L2_DIR / MOTOR_R1_DIR / MOTOR_R2_DIR)
 * @param  pwm_pin   电机PWM通道 (PWM_CH1_L1 / PWM_CH2_R1 / PWM_CH3_L2 / PWM_CH4_R2)
 * @param  speed     电机速度, 范围 [-10000, +10000]
 *                   正值 = 正转, 负值 = 反转, 绝对值 = PWM占空比
 */
void Motor_Set(gpio_pin_enum dir_pin, pwm_channel_enum pwm_pin, int32 speed);

/**
 * @brief  紧急停车, 所有电机立即停止
 */
void Motor_Stop(void);

/* ========================= 便捷运动函数 ========================= */

/**
 * @brief  前进
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_Forward(int32 speed);

/**
 * @brief  后退
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_Backward(int32 speed);

/**
 * @brief  纯左平移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_Left(int32 speed);

/**
 * @brief  纯右平移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_Right(int32 speed);

/**
 * @brief  左前方 45° 斜移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_FrontLeft(int32 speed);

/**
 * @brief  右前方 45° 斜移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_FrontRight(int32 speed);

/**
 * @brief  左后方 45° 斜移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_RearLeft(int32 speed);

/**
 * @brief  右后方 45° 斜移
 * @param  speed  速度, 范围 [0, 10000]
 */
void Car_RearRight(int32 speed);

/**
 * @brief  原地逆时针旋转 (俯视)
 * @param  speed  旋转速度, 范围 [0, 10000]
 */
void Car_Rotate_CCW(int32 speed);

/**
 * @brief  原地顺时针旋转 (俯视)
 * @param  speed  旋转速度, 范围 [0, 10000]
 */
void Car_Rotate_CW(int32 speed);

#endif // __MOTOR_H__
