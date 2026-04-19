/**
 * @file Fuzzy_Pid.h
 * @brief 模糊PID定位置控制 - 头文件
 * 
 * ============================================================================
 *                            模糊PID定位置控制说明
 * ============================================================================
 * 
 * 一、硬件配置
 *    - 电机: 逐飞RC380有刷电机
 *    - 编码器: 1024线正交解码迷你编码器
 *    - 分辨率: 1024线 × 4倍频 = 4096脉冲/转
 * 
 * 二、控制原理
 *    采用串级PID控制结构：
 *    ┌─────────────────────────────────────────────────────────┐
 *    │                    串级PID结构                           │
 *    │                                                         │
 *    │   目标位置 ──→[位置环PID]──→ 目标速度 ──→[速度环PID]──→ PWM输出 │
 *    │       │                                  │            │
 *    │       ↓                                  ↓            │
 *    │   编码器反馈                        编码器速度反馈      │
 *    │   (位置信息)                        (速度信息)         │
 *    └─────────────────────────────────────────────────────────┘
 * 
 * 三、模糊控制原理
 *    模糊控制器根据误差e和误差变化率ec自动调整PID参数：
 *    ┌─────────────────────────────────────────────────────────┐
 *    │                     模糊控制器                           │
 *    │                                                         │
 *    │   e(误差) ──→[模糊化]──→ 查询规则表 ──→[去模糊化]──→ ΔKp,ΔKi,ΔKd │
 *    │      │                                                          │
 *    │   ec(误差变化率) ──→[模糊化]──→ 模糊推理 ─────────────────→      │
 *    └─────────────────────────────────────────────────────────┘
 * 
 * 四、模糊规则表说明
 *    - 行列分别对应误差e和误差变化率ec的模糊语言变量
 *    - 值范围: -3 ~ +3 (负大NB, 负中NM, 负小NS, 零ZO, 正小PS, 正中PM, 正大PB)
 *    - 根据系统状态自动查表获取PID参数修正量
 * 
 * ============================================================================
 * 
 * @author 逐飞科技
 * @date 2025
 */

#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include "zf_common_headfile.h"

/*=============================== 电机通道定义 ================================*/
#define FUZZY_POS_PID_L1       0       /**< 左前电机通道 */
#define FUZZY_POS_PID_L2       1       /**< 左后电机通道 */
#define FUZZY_POS_PID_R1       2       /**< 右前电机通道 */
#define FUZZY_POS_PID_R2       3       /**< 右后电机通道 */

/*=============================== 编码器参数定义 ================================*/
#define ENCODER_RESOLUTION     1024    /**< 编码器线数(1024线) */
#define GEAR_RATIO             1       /**< 齿轮减速比(无减速) */
#define PULSE_PER_REV          (ENCODER_RESOLUTION * 4 * GEAR_RATIO)  /**< 每转脉冲数(4096) */

/*=============================== 模糊规则表维度 ================================*/
#define FUZZY_SET_SIZE         7       /**< 模糊规则表维度(7x7) */

/*=============================== PID基础参数 ================================*/
/**@brief PID参数基础值
 *   - Kp: 比例系数，影响响应速度
 *   - Ki: 积分系数，消除稳态误差
 *   - Kd: 微分系数，抑制超调
 */
#define FUZZY_KP_BASE          15.0f   /**< 模糊PID比例系数基础值 */
#define FUZZY_KI_BASE          0.5f    /**< 模糊PID积分系数基础值 */
#define FUZZY_KD_BASE          8.0f    /**< 模糊PID微分系数基础值 */

/*=============================== PID输出限幅 ================================*/
#define POSITION_PID_MAX_OUT   1000.0f  /**< 位置环输出限幅*/
#define SPEED_PID_MAX_OUT      8000.0f /**< 速度环输出限幅(PWM上限) */

/*=============================== 数据结构定义 ================================*/

/**
 * @brief 位置环PID结构体
 * @details 用于外环位置控制，输出作为速度环的目标值，单位:RPM
 */
typedef struct
{
    float Kp;              /**< 比例系数 */
    float Ki;              /**< 积分系数 */
    float Kd;              /**< 微分系数 */
    float target;          /**< 目标位置(脉冲数) */
    float actual;          /**< 实际位置(脉冲数) */
    float error_prev2;     /**< 前前次误差 e(k-2) */
    float error_last;      /**< 上一次误差 e(k-1) */
    float error_current;   /**< 当前误差 e(k) */
    float output;          /**< PID输出值*/
    float Pwm_Max_Out;     /**< 输出限幅值 */
} Fuzzy_Position_PID;

/**
 * @brief 速度环PID结构体
 * @details 用于内环速度控制，输出直接驱动电机PWM，单位:RPM
 */
typedef struct
{
    float Kp;              /**< 比例系数 */
    float Ki;              /**< 积分系数 */
    float Kd;              /**< 微分系数 */
    float target;          /**< 目标速度(RPM) */
    float actual;          /**< 实际速度(RPM) */
    float error_prev2;     /**< 前前次误差 e(k-2) */
    float error_last;      /**< 上一次误差 e(k-1) */
    float error_current;   /**< 当前误差 e(k) */
    float output;          /**< PID输出值(电机PWM) */
    float Pwm_Max_Out;     /**< PWM输出限幅值 */
} Fuzzy_Speed_PID;

/**
 * @brief 模糊规则输出结构体
 * @details 存储PID参数的修正量(正/负方向)
 */
typedef struct
{
    float Kp_n;            /**< Kp负向修正量 */
    float Ki_n;            /**< Ki负向修正量 */
    float Kd_n;            /**< Kd负向修正量 */
    float Kp_p;            /**< Kp正向修正量 */
    float Ki_p;            /**< Ki正向修正量 */
    float Kd_p;            /**< Kd正向修正量 */
} Fuzzy_Rule_Output;

/**
 * @brief 串级PID输出结构体
 * @details 返回外环和内环的输出值
 */
typedef struct
{
    float position_out;   /**< 位置环输出 */
    float speed_out;      /**< 速度环输出(PWM值) */
} Cascade_PID_Output;

/**
 * @brief 模糊串级PID控制器结构体
 * @details 包含位置环、速度环和电机相关信息
 */
typedef struct
{
    Fuzzy_Position_PID position_pid;
    Fuzzy_Speed_PID speed_pid;
    int32 target_position;
    int32 actual_position;
    int32 speed_feedback;
    uint8 motor_id;
    uint8 dir_pin;
} Fuzzy_Cascade_PID;

/*=============================== 函数接口声明 ================================*/

/**
 * @brief 模糊PID初始化函数
 * @details 初始化所有PID参数为默认值，应在系统启动时调用一次
 * 
 * @note 调用示例: Fuzzy_PID_Init();
 */
void Fuzzy_PID_Init(void);

/**
 * @brief 设置目标位置函数
 * @details 设置指定电机的目标位置(脉冲数)
 * 
 * @param id        电机通道ID
 *                   - FUZZY_POS_PID_L1: 左电机
 *                   - FUZZY_POS_PID_R1: 右电机
 * @param target_pos 目标位置(单位: 脉冲数)
 *                    - 正值: 正转
 *                    - 负值: 反转
 * 
 * @note 使用示例: 
 *       // 让左电机转动4096个脉冲(1圈)
 *       Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);
 *       
 *       // 让右电机反转2048个脉冲(半圈)
 *       Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, -2048);
 */
void Fuzzy_PID_Set_Target(uint8 id, int32 target_pos);

/**
 * @brief 更新当前位置函数
 * @details 更新指定电机的当前位置(用于外部位置传感器)
 * 
 * @param id          电机通道ID
 * @param current_pos 当前实际位置(脉冲数)
 * 
 * @note 如果使用编码器自动更新，此函数可选使用
 */
void Fuzzy_PID_Update_Position(uint8 id, int32 current_pos);

/**
 * @brief 模糊PID计算函数(核心函数)
 * @details 执行模糊推理和串级PID计算，更新所有电机状态
 * 
 * @param id 电机通道ID
 *           - FUZZY_POS_PID_L1: 只更新左电机
 *           - FUZZY_POS_PID_R1: 只更新右电机
 *           - 0xFF: 更新所有电机
 * 
 * @note 此函数需要在定时中断中周期调用，推荐调用周期10ms
 * @note 使用示例:
 *       void TIM3_IRQHandler(void)
 *       {
 *           if(timer_check_interrupt(TIM_3))
 *           {
 *               Fuzzy_PID_Calculate(0xFF);  // 更新所有电机
 *           }
 *       }
 */
void Fuzzy_PID_Calculate(uint8 id);

/**
 * @brief 获取PID输出函数
 * @details 获取指定电机的PID输出值
 * 
 * @param id 电机通道ID
 * 
 * @return Cascade_PID_Output 结构体
 *         - position_out: 位置环输出
 *         - speed_out: 速度环输出(用于驱动电机)
 * 
 * @note 使用示例:
 *       Cascade_PID_Output out = Fuzzy_PID_Get_Output(FUZZY_POS_PID_L1);
 *       int16 pwm_value = (int16)out.speed_out;
 *       Motor_Set_PWM(pwm_value);  // 驱动电机
 */
Cascade_PID_Output Fuzzy_PID_Get_Output(uint8 id);

/**
 * @brief 获取电机位置函数
 * @details 获取编码器累计的脉冲数
 * 
 * @param id 电机通道ID
 * 
 * @return int32 当前位置(脉冲数)
 * 
 * @note 使用示例:
 *       int32 pos = Get_Motor_Position(FUZZY_POS_PID_L1);
 *       printf("左电机当前位置: %d 脉冲\n", pos);
 */
int32 Get_Motor_Position(uint8 id);

/**
 * @brief 清零电机位置函数
 * @details 清零指定电机的位置计数器
 * 
 * @param id 电机通道ID
 *           - 0xFF: 清零所有电机
 * 
 * @note 使用示例:
 *       // 电机回零后清零位置
 *       Clear_Motor_Position(FUZZY_POS_PID_L1);
 *       Clear_Motor_Position(FUZZY_POS_PID_R1);
 */
void Clear_Motor_Position(uint8 id);

/**
 * @brief 模糊推理函数
 * @details 根据误差和误差变化率进行模糊推理
 * 
 * @param error          当前误差
 * @param error_change   误差变化率(ec = e(k) - e(k-1))
 * 
 * @return float 模糊规则输出值(-3 ~ +3)
 * 
 * @note 内部函数，通常不需要外部调用
 */
float Fuzzy_Inference(float error, float error_change);

/**
 * @brief 模糊PID参数调整函数
 * @details 根据误差和误差变化率自动调整PID参数
 * 
 * @param error          当前误差
 * @param error_change   误差变化率
 * @param Kp            输出: 调整后的比例系数
 * @param Ki            输出: 调整后的积分系数
 * @param Kd            输出: 调整后的微分系数
 * 
 * @note 内部函数，通常不需要外部调用
 */
void Fuzzy_PID_Adjust(float error, float error_change, float *Kp, float *Ki, float *Kd);

#endif
