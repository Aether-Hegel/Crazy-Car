/**
 * @file  PID.c
 * @brief 级联PID控制器实现 — 外环模糊位置PID + 内环速度PID
 *
 * ─────────────────────────────────────────────────────────────────
 *  控制结构 (每路电机独立):
 *
 *   目标位置 [脉冲数]
 *        │
 *        ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 外环: 模糊位置PID  (Fuzzy_Pid.c / Fuzzy_Pid.h)          │
 *   │   输入: target_position, actual_position (编码器累计)   │
 *   │   算法: 模糊规则整定 Kp/Kd + 条件积分 + 前馈 + 死区锁存 │
 *   │   输出: 目标速度原始值 ∈ [-FUZZY_PWM_MAX_OUT, +...]     │
 *   └─────────────────────────────────────────────────────────┘
 *        │  × CASCADE_SPEED_SCALE  (外环输出 → 内环速度目标)
 *        ▼
 *   目标速度 [脉冲/控制周期]
 *        │
 *        ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 内环: 速度PID  (Speed_PID.c / Speed_PID.h)              │
 *   │   输入: target_speed, actual_speed (Encoder_Current)    │
 *   │   算法: 增量式 PID + 误差滤波 + 输出限幅               │
 *   │   输出: PWM ∈ [-8000, +8000]                           │
 *   └─────────────────────────────────────────────────────────┘
 *        │
 *        ▼
 *   Motor_Set() → 方向GPIO + PWM占空比 → RC380电机
 * ─────────────────────────────────────────────────────────────────
 *
 *  标准调用顺序 (定时中断，推荐周期 5~20 ms):
 *
 *    void pit0_isr(void)
 *    {
 *        Encoder_Speed_PID_Update();      // ① 先更新编码器
 *        Cascade_PID_Calculate(0xFF);     // ② 再执行级联PID (外环→内环)
 *        Cascade_PID_Drive_All_Motor();   // ③ 最后驱动电机
 *    }
 *
 * ─────────────────────────────────────────────────────────────────
 *  关键参数调整指引:
 *
 *  CASCADE_SPEED_SCALE — 外环输出到内环目标速度的缩放系数
 *    外环最大输出 = FUZZY_PWM_MAX_OUT = 1000
 *    内环目标速度单位 = Encoder_Current (脉冲/周期)
 *    scale = 0.5 → 外环满幅时内环目标速度 ±500 脉冲/周期
 *    若实际电机在满PWM时 Encoder_Current 峰值约 ±N:
 *      推荐 scale = N / FUZZY_PWM_MAX_OUT
 *
 *  内环速度PID参数 (cas_spid_Lx 初始值):
 *    Kp 过小 → 速度响应慢，跟踪外环命令迟滞
 *    Kp 过大 → 速度振荡，影响位置精度
 *    Ki 过大 → 低速积分饱和，导致超调
 *    Kd 适当 → 抑制速度突变，配合编码器噪声调整
 *
 *  外环模糊位置PID参数请参见 Fuzzy_Pid.h 中的调参指引。
 * ─────────────────────────────────────────────────────────────────
 */

#include "PID.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"

/* =====================================================================
 *  级联比例系数
 *
 *  将外环位置PID输出 (量纲: 等效PWM强度, ±FUZZY_PWM_MAX_OUT=±1000)
 *  转换为内环速度PID的目标速度 (量纲: 脉冲/控制周期, 与 Encoder_Current 一致)
 *
 *  调整方法:
 *    以实测方式，将单轮以固定PWM驱动，观察 Encoder_Current 峰值 N，
 *    设 CASCADE_SPEED_SCALE = (float)N / FUZZY_PWM_MAX_OUT
 *
 *  默认值 0.5: 外环输出 ±1000 → 内环目标速度 ±500 脉冲/周期
 * ===================================================================== */
#define CASCADE_SPEED_SCALE   0.5f

/* =====================================================================
 *  外部变量声明
 *
 *  Encoder_Current_Lx/Rx 定义于 Encoder.c，由 Encoder_Speed_PID_Update()
 *  每周期写入，代表本周期编码器脉冲增量，即当前实际速度。
 * ===================================================================== */
extern int16 Encoder_Current_L1;
extern int16 Encoder_Current_L2;
extern int16 Encoder_Current_R1;
extern int16 Encoder_Current_R2;

/* =====================================================================
 *  内环速度PID实例 (级联专用，与 Speed_PID.c 中的全局实例相互独立)
 *
 *  output_dead_zone 设为 0:
 *    外环位置PID已内置死区锁存机制 (FUZZY_DB_ENTER/EXIT)，当位置到位时
 *    外环输出为 0，内环目标速度自然趋零。若内环再加大死区，会导致
 *    小速度指令无法执行，影响位置精度。
 *
 *  integral_limit:
 *    限制增量式PID的输出累积上限，防止积分饱和。
 * ===================================================================== */
static Speed_PID cas_spid_L1 = {
    .Kp = 1.2f, .Ki = 0.05f, .Kd = 2.5f,
    .target_speed    = 0.0f,
    .Pwm_Max_Out     = 8000.0f,
    .integral_sum    = 0.0f,
    .integral_limit  = 5000.0f,
    .output_dead_zone = 50.0f
};

static Speed_PID cas_spid_L2 = {
    .Kp = 1.2f, .Ki = 0.05f, .Kd = 2.5f,
    .target_speed    = 0.0f,
    .Pwm_Max_Out     = 8000.0f,
    .integral_sum    = 0.0f,
    .integral_limit  = 5000.0f,
    .output_dead_zone = 50.0f
};

static Speed_PID cas_spid_R1 = {
    .Kp = 1.2f, .Ki = 0.05f, .Kd = 2.5f,
    .target_speed    = 0.0f,
    .Pwm_Max_Out     = 8000.0f,
    .integral_sum    = 0.0f,
    .integral_limit  = 5000.0f,
    .output_dead_zone = 50.0f
};

static Speed_PID cas_spid_R2 = {
    .Kp = 1.2f, .Ki = 0.05f, .Kd = 2.5f,
    .target_speed    = 0.0f,
    .Pwm_Max_Out     = 8000.0f,
    .integral_sum    = 0.0f,
    .integral_limit  = 5000.0f,
    .output_dead_zone = 50.0f
};

/* =====================================================================
 *  内部辅助函数
 * ===================================================================== */

/**
 * @brief 根据通道ID获取对应的内环速度PID实例指针
 * @param id 电机通道 (0=L1, 1=L2, 2=R1, 3=R2)
 * @return   Speed_PID 指针，无效 id 返回 NULL
 */
static Speed_PID *get_inner_pid(uint8 id)
{
    if      (id == CASCADE_CH_L1) return &cas_spid_L1;
    else if (id == CASCADE_CH_L2) return &cas_spid_L2;
    else if (id == CASCADE_CH_R1) return &cas_spid_R1;
    else if (id == CASCADE_CH_R2) return &cas_spid_R2;
    return NULL;
}

/**
 * @brief 根据通道ID获取当前实际速度 (本周期编码器脉冲增量)
 * @param id 电机通道
 * @return   int16 编码器脉冲/周期，正值=正转，负值=反转
 */
static int16 get_actual_speed(uint8 id)
{
    if      (id == CASCADE_CH_L1) return Encoder_Current_L1;
    else if (id == CASCADE_CH_L2) return Encoder_Current_L2;
    else if (id == CASCADE_CH_R1) return Encoder_Current_R1;
    else if (id == CASCADE_CH_R2) return Encoder_Current_R2;
    return 0;
}

/**
 * @brief 重置单个内环速度PID的误差历史和输出
 *
 * 在设置新的位置目标或系统复位时调用，防止历史积分/误差
 * 在下次运动开始时造成PWM突跳。
 *
 * @param pid 目标速度PID实例指针
 */
static void reset_inner_pid(Speed_PID *pid)
{
    pid->target_speed   = 0.0f;
    pid->Actual_speed   = 0.0f;
    pid->error_current  = 0.0f;
    pid->error_last     = 0.0f;
    pid->error_prev2    = 0.0f;
    pid->output         = 0.0f;
    pid->integral_sum   = 0.0f;
}

/**
 * @brief 单通道级联计算核心
 *
 * 执行流程:
 *   ① 调用外环模糊位置PID (Fuzzy_PID_Calculate)，内部自动从编码器
 *      读取 actual_position，并经模糊推理输出 output
 *   ② 外环输出 × CASCADE_SPEED_SCALE → 内环目标速度
 *   ③ 读取当前实际速度 (Encoder_Current_Lx)
 *   ④ 内环速度PID计算 (Speed_PID_Calculate)，输出最终PWM
 *
 * @param id          电机通道
 * @param inner       对应的内环速度PID实例
 * @param actual_spd  当前实际速度 (脉冲/控制周期)
 */
static void cascade_step(uint8 id, Speed_PID *inner, int16 actual_spd)
{
    /* ① 外环: 模糊位置PID计算 */
    Fuzzy_PID_Calculate(id);

    /* ② 外环输出转换为内环目标速度
     *    Fuzzy_PID_Get_Output().speed_out ∈ [-FUZZY_PWM_MAX_OUT, +FUZZY_PWM_MAX_OUT]
     *    × CASCADE_SPEED_SCALE → 脉冲/周期目标速度 */
    float outer_output    = Fuzzy_PID_Get_Output(id).speed_out;
    inner->target_speed   = outer_output * CASCADE_SPEED_SCALE;

    /* ③ 内环: 速度PID计算 */
    Speed_PID_Calculate(inner, (float)actual_spd);
}

/* =====================================================================
 *  公开接口实现
 * ===================================================================== */

/**
 * @brief 初始化级联PID控制器 (外环 + 内环)
 *
 * ① 调用 Fuzzy_PID_Init() 初始化全部4路外环模糊位置PID
 * ② 重置全部4路内环速度PID的误差状态
 *
 * 调用要求: 必须在 Motor_Init()、Encoder_Init()、MyPWM_Init() 之后调用。
 */
void Cascade_PID_Init(void)
{
    /* 外环: 模糊位置PID初始化 */
    Fuzzy_PID_Init();

    /* 内环: 速度PID状态清零 (Kp/Ki/Kd 已在声明时写入，无需重设) */
    reset_inner_pid(&cas_spid_L1);
    reset_inner_pid(&cas_spid_L2);
    reset_inner_pid(&cas_spid_R1);
    reset_inner_pid(&cas_spid_R2);
}

/**
 * @brief 设置指定电机的目标位置 (绝对脉冲数)
 *
 * 内部调用 Fuzzy_PID_Set_Target() 写入外环位置目标，
 * 同时重置内环误差历史，防止目标突变时 PWM 冲击。
 *
 * @param id         电机通道 (CASCADE_CH_L1/L2/R1/R2)
 * @param target_pos 目标脉冲数，相对于上次 Cascade_PID_Clear_Position 的零点
 */
void Cascade_PID_Set_Target(uint8 id, int32 target_pos)
{
    /* 外环目标设置 (含微分冲击抑制) */
    Fuzzy_PID_Set_Target(id, target_pos);

    /* 内环状态重置: 避免旧误差历史在新目标段产生初始脉冲 */
    Speed_PID *inner = get_inner_pid(id);
    if (inner != NULL) {
        reset_inner_pid(inner);
    }
}

/**
 * @brief 清零指定电机的位置计数器及全部PID内部状态
 *
 * 外环: 调用 Clear_Motor_Position()，清零编码器累计值和外环误差历史
 * 内环: 调用 reset_inner_pid()，清零速度误差历史和输出
 *
 * @param id 电机通道 或 0xFF (清零全部4路)
 *
 * 注意: 清零后 target_position 不变，需重新调用 Cascade_PID_Set_Target()。
 */
void Cascade_PID_Clear_Position(uint8 id)
{
    /* 外环位置清零 */
    Clear_Motor_Position(id);

    /* 内环状态清零 */
    if (id == CASCADE_CH_L1 || id == 0xFF) reset_inner_pid(&cas_spid_L1);
    if (id == CASCADE_CH_L2 || id == 0xFF) reset_inner_pid(&cas_spid_L2);
    if (id == CASCADE_CH_R1 || id == 0xFF) reset_inner_pid(&cas_spid_R1);
    if (id == CASCADE_CH_R2 || id == 0xFF) reset_inner_pid(&cas_spid_R2);
}

/**
 * @brief 级联PID主计算函数，必须在定时中断中周期调用
 *
 * 对每个选中通道依次执行 cascade_step():
 *   外环模糊位置PID → 目标速度 → 内环速度PID → 存入 cas_spid_Lx.output
 *
 * 计算结果通过 Cascade_PID_Drive_Motor() 或 Cascade_PID_Get_Inner_Output()
 * 获取和使用。
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2) 或 0xFF (全部4路)
 *
 * 注意: 必须在 Encoder_Speed_PID_Update() 之后调用，否则读到上周期旧值。
 */
void Cascade_PID_Calculate(uint8 id)
{
    if (id == CASCADE_CH_L1 || id == 0xFF)
        cascade_step(CASCADE_CH_L1, &cas_spid_L1, Encoder_Current_L1);

    if (id == CASCADE_CH_L2 || id == 0xFF)
        cascade_step(CASCADE_CH_L2, &cas_spid_L2, Encoder_Current_L2);

    if (id == CASCADE_CH_R1 || id == 0xFF)
        cascade_step(CASCADE_CH_R1, &cas_spid_R1, Encoder_Current_R1);

    if (id == CASCADE_CH_R2 || id == 0xFF)
        cascade_step(CASCADE_CH_R2, &cas_spid_R2, Encoder_Current_R2);
}

/**
 * @brief 获取指定通道内环速度PID的最新输出 (不触发重新计算)
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2)
 * @return   float PWM输出值，范围 [-8000, +8000]
 *           正值=正转，负值=反转。传入无效 id 返回 0.0f。
 */
float Cascade_PID_Get_Inner_Output(uint8 id)
{
    Speed_PID *inner = get_inner_pid(id);
    if (inner == NULL) return 0.0f;
    return inner->output;
}

/**
 * @brief 将内环PID输出写入单路电机 (方向GPIO + PWM)
 *
 * 读取 cas_spid_Lx.output，调用 Motor_Set():
 *   output > 0 → 方向引脚置高(正转) + PWM = output
 *   output < 0 → 方向引脚置低(反转) + PWM = |output|
 *   output = 0 → PWM = 0，电机停止
 *
 * @param id 电机通道 (CASCADE_CH_L1/L2/R1/R2)
 *
 * 注意: 必须在本周期 Cascade_PID_Calculate() 之后调用。
 */
void Cascade_PID_Drive_Motor(uint8 id)
{
    int32 pwm = (int32)Cascade_PID_Get_Inner_Output(id);

    if      (id == CASCADE_CH_L1) Motor_Set(MOTOR_L1_DIR, PWM_CH1_L1, pwm);
    else if (id == CASCADE_CH_L2) Motor_Set(MOTOR_L2_DIR, PWM_CH3_L2, pwm);
    else if (id == CASCADE_CH_R1) Motor_Set(MOTOR_R1_DIR, PWM_CH2_R1, pwm);
    else if (id == CASCADE_CH_R2) Motor_Set(MOTOR_R2_DIR, PWM_CH4_R2, pwm);
}

/**
 * @brief 驱动全部4路电机 (4次 Cascade_PID_Drive_Motor 的简化封装)
 *
 * 示例 (标准中断结构):
 *   void pit0_isr(void)
 *   {
 *       Encoder_Speed_PID_Update();
 *       Cascade_PID_Calculate(0xFF);
 *       Cascade_PID_Drive_All_Motor();
 *   }
 */
void Cascade_PID_Drive_All_Motor(void)
{
    Cascade_PID_Drive_Motor(CASCADE_CH_L1);
    Cascade_PID_Drive_Motor(CASCADE_CH_L2);
    Cascade_PID_Drive_Motor(CASCADE_CH_R1);
    Cascade_PID_Drive_Motor(CASCADE_CH_R2);
}
