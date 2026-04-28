/**
 * @file  PID.h
 * @brief 级联PID控制器接口 — 外环模糊位置PID + 内环速度PID
 *
 * 控制结构:
 *
 *   目标位置 (脉冲数)
 *        │
 *        ▼
 *   [外环: 模糊位置PID]  ← Ecoder_Total_Lx (累计位置)
 *        │  输出: 目标速度 (脉冲/周期)
 *        ▼
 *   [内环: 速度PID]      ← Encoder_Current_Lx (本周期脉冲数)
 *        │  输出: PWM [-8000, +8000]
 *        ▼
 *   [Motor_Set] → 方向GPIO + PWM → 电机
 *
 * 标准调用顺序 (定时中断中):
 *   void pit0_isr(void)
 *   {
 *       Encoder_Speed_PID_Update();      // ① 更新编码器
 *       Cascade_PID_Calculate(0xFF);     // ② 级联PID计算 (外环→内环)
 *       Cascade_PID_Drive_All_Motor();   // ③ 驱动电机
 *   }
 */

#ifndef PID_H
#define PID_H

#include "zf_common_headfile.h"
#include "Fuzzy_Pid.h"
#include "Speed_PID.h"

/* =====================================================================
 *  电机通道 ID (与 Fuzzy_Pid.h 中的定义一致)
 * ===================================================================== */
#define CASCADE_CH_L1   FUZZY_POS_PID_L1   /**< 左前电机 */
#define CASCADE_CH_L2   FUZZY_POS_PID_L2   /**< 左后电机 */
#define CASCADE_CH_R1   FUZZY_POS_PID_R1   /**< 右前电机 */
#define CASCADE_CH_R2   FUZZY_POS_PID_R2   /**< 右后电机 */

/* =====================================================================
 *  公开函数接口
 * ===================================================================== */

/**
 * @brief 初始化级联PID控制器 (外环 + 内环)
 *
 * 内部依次调用 Fuzzy_PID_Init() 和各路内环 Speed_PID 参数写入。
 * 必须在 Motor_Init()、Encoder_Init()、MyPWM_Init() 之后调用一次。
 */
void Cascade_PID_Init(void);

/**
 * @brief 设置指定电机的目标位置 (绝对脉冲数，相对于零点)
 *
 * @param id         电机通道 (CASCADE_CH_L1/L2/R1/R2)
 * @param target_pos 目标脉冲数，正值=正转，负值=反转，4096=1圈
 */
void Cascade_PID_Set_Target(uint8 id, int32 target_pos);

/**
 * @brief 清零指定电机的位置计数器及PID内部状态
 *
 * @param id 电机通道 或 0xFF (清零全部4路)
 *
 * 注意: 清零后需重新调用 Cascade_PID_Set_Target() 设置新目标。
 */
void Cascade_PID_Clear_Position(uint8 id);

/**
 * @brief 级联PID主计算函数，在定时中断中周期调用
 *
 * 对每个选中通道依次执行:
 *   ① 外环: 模糊位置PID计算，输出目标速度
 *   ② 比例缩放: 外环输出 × CASCADE_SPEED_SCALE → 内环目标速度
 *   ③ 内环: 速度PID计算，输出PWM
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2) 或 0xFF (全部4路)
 *
 * 注意: 必须在 Encoder_Speed_PID_Update() 之后调用。
 */
void Cascade_PID_Calculate(uint8 id);

/**
 * @brief 将内环PID输出写入单路电机 (方向GPIO + PWM)
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2)
 *
 * 注意: 必须在本周期 Cascade_PID_Calculate() 之后调用。
 */
void Cascade_PID_Drive_Motor(uint8 id);

/**
 * @brief 驱动全部4路电机 (4次 Cascade_PID_Drive_Motor 的简化封装)
 */
void Cascade_PID_Drive_All_Motor(void);

/**
 * @brief 获取指定通道内环速度PID的最新输出 (PWM值)
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2)
 * @return   float PWM输出值，范围 [-8000, +8000]
 */
float Cascade_PID_Get_Inner_Output(uint8 id);

#endif /* PID_H */
