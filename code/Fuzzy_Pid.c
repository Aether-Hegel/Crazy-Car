/**
 * ============================================================================
 *                      模糊PID定位置控制 - 源文件
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 本模块实现了基于模糊控制的串级PID定位置控制系统，专为逐飞科技RC380有刷电机
 * 和1024线正交解码编码器设计。
 * 
 * 控制框图:
 * ----------
 * 
 *     ┌──────────────────────────────────────────────────────────────────┐
 *     │                       模糊串级PID控制器                            │
 *     │                                                                  │
 *     │   目标位置                                                    │
 *     │       │                                                        │
 *     │       ▼                                                        │
 *     │   ┌─────────────────────────────────────────────────────────┐   │
 *     │   │              外环: 位置环(模糊PID)                       │   │
 *     │   │                                                         │   │
 *     │   │   e_pos = target_pos - actual_pos                      │   │
 *     │   │   fuzzy_adjust(e_pos, ec_pos) → ΔKp, ΔKi, ΔKd          │   │
 *     │   │   output = fuzzy_pid(e_pos, Kp+ΔKp, Ki+ΔKi, Kd+ΔKd)    │   │
 *     │   │                                                         │   │
 *     │   └─────────────────────────────────────────────────────────┘   │
 *     │       │                                                        │
 *     │       ▼ (位置环输出作为速度环目标)                               │
 *     │   ┌─────────────────────────────────────────────────────────┐   │
 *     │   │              内环: 速度环(普通PID)                       │   │
 *     │   │                                                         │   │
 *     │   │   e_speed = target_speed - actual_speed                │   │
 *     │   │   output = pid(e_speed, Kp, Ki, Kd)                    │   │
 *     │   │                                                         │   │
 *     │   └─────────────────────────────────────────────────────────┘   │
 *     │       │                                                        │
 *     │       ▼                                                        │
 *     │   ┌────────┐                                                   │
 *     │   │  PWM   │ ──→ 电机驱动                                      │
 *     │   └────────┘                                                   │
 *     │                                                                  │
 *     │   编码器反馈:                                                    │
 *     │   - 位置反馈: encoder_total (累计脉冲数)                        │
 *     │   - 速度反馈: delta_encoder (每周期脉冲差)                      │
 *     │                                                                  │
 *     └──────────────────────────────────────────────────────────────────┘
 * 
 * 模糊控制规则表:
 * ----------------
 *    误差e \ 误差变化率ec   NB    NM    NS    ZO    PS    PM    PB
 *    ─────────────────────────────────────────────────────────────────
 *         NB                  PB    PB    PM    PM    PS    ZO    ZO
 *         NM                  PB    PB    PM    PS    PS    ZO    NS
 *         NS                  PM    PM    PM    PS    ZO    NS    NS
 *         ZO                  PM    PS    PS    ZO    NS    NS    NM
 *         PS                  PS    PS    ZO    NS    NS    NM    NM
 *         PM                  ZO    ZO    NS    NM    NM    NB    NB
 *         PB                  ZO    ZO    NS    NM    NM    NB    NB
 * 
 *    NB=负大, NM=负中, NS=负小, ZO=零, PS=正小, PM=正中, PB=正大
 * 
 * 使用说明:
 * ----------
 * 1. 系统初始化阶段调用 Fuzzy_PID_Init()
 * 2. 设置目标位置调用 Fuzzy_PID_Set_Target()
 * 3. 定时中断中调用 Fuzzy_PID_Calculate()
 * 4. 从输出结构体获取PWM值驱动电机
 * 
 * 调参建议:
 * ----------
 * - 位置环: Kp影响响应速度, Ki消除稳态误差, Kd抑制超调
 * - 速度环: 建议先调位置环再调速度环
 * - 模糊规则可根据实际响应调整Rule_Kp/Rule_Ki/Rule_Kd表
 * 
 * ============================================================================
 */

#include "Fuzzy_Pid.h"
#include "Encoder.h"
#include "Motor.h"

/*=============================== 模糊规则表定义 ================================*/

/**
 * @brief 模糊推理规则表
 * @details 7x7矩阵，行索引为误差e的模糊化结果，列索引为误差变化率ec的模糊化结果
 *         返回值为模糊语言变量对应的量化值(-3 ~ +3)
 * 
 * 语言变量对应关系:
 *   -3: NB (Negative Big)     负大
 *   -2: NM (Negative Medium)  负中
 *   -1: NS (Negative Small)   负小
 *    0: ZO (Zero)             零
 *   +1: PS (Positive Small)   正小
 *   +2: PM (Positive Medium) 正中
 *   +3: PB (Positive Big)     正大
 */
static const int8 Fuzzy_Table[FUZZY_SET_SIZE][FUZZY_SET_SIZE] = {
    /*         ec: NB    NM    NS    ZO    PS    PM    PB    */
    /* e=NB */ {  3,   3,   2,   1,   0,  -1,  -2},
    /* e=NM */ {  3,   3,   2,   1,   0,  -1,  -2},
    /* e=NS */ {  2,   2,   2,   1,   0,  -1,  -1},
    /* e=ZO */ {  2,   2,   1,   0,  -1,  -2,  -2},
    /* e=PS */ {  1,   1,   0,  -1,  -1,  -2,  -2},
    /* e=PM */ {  1,   1,   0,  -1,  -2,  -2,  -3},
    /* e=PB */ {  2,   1,   0,  -1,  -2,  -3,  -3}
};

/**
 * @brief Kp参数模糊规则表
 * @details 存储不同误差等级下Kp的修正量
 *         _n结尾表示负向修正，_p结尾表示正向修正
 */
static const Fuzzy_Rule_Output Rule_Kp[FUZZY_SET_SIZE] = {
    /* NB */ {.Kp_n = -2.0f, .Ki_n = -1.5f, .Kd_n = -1.0f, .Kp_p = 0.5f,  .Ki_p = 0.3f,  .Kd_p = 0.2f},
    /* NM */ {.Kp_n = -1.5f, .Ki_n = -1.0f, .Kd_n = -0.5f, .Kp_p = 0.4f,  .Ki_p = 0.2f,  .Kd_p = 0.15f},
    /* NS */ {.Kp_n = -1.0f, .Ki_n = -0.5f, .Kd_n = -0.3f, .Kp_p = 0.3f,  .Ki_p = 0.15f, .Kd_p = 0.1f},
    /* ZO */ {.Kp_n = -0.5f, .Ki_n = -0.2f, .Kd_n = -0.1f, .Kp_p = 0.2f,  .Ki_p = 0.1f,  .Kd_p = 0.05f},
    /* PS */ {.Kp_n = -0.3f, .Ki_n = -0.1f, .Kd_n = -0.05f,.Kp_p = 0.15f, .Ki_p = 0.08f, .Kd_p = 0.03f},
    /* PM */ {.Kp_n = -0.15f,.Ki_n = -0.05f,.Kd_n = -0.02f,.Kp_p = 0.08f, .Ki_p = 0.05f, .Kd_p = 0.02f},
    /* PB */ {.Kp_n = -0.08f,.Ki_n = -0.03f,.Kd_n = -0.01f,.Kp_p = 0.05f, .Ki_p = 0.03f, .Kd_p = 0.01f}
};

/**
 * @brief Ki参数模糊规则表
 * @details 存储不同误差等级下Ki的修正量
 *         Ki用于消除稳态误差，修正量较小
 */
static const Fuzzy_Rule_Output Rule_Ki[FUZZY_SET_SIZE] = {
    /* NB */ {.Kp_n = -0.1f,  .Ki_n = -0.08f, .Kd_n = -0.05f,.Kp_p = 0.02f, .Ki_p = 0.01f, .Kd_p = 0.005f},
    /* NM */ {.Kp_n = -0.08f, .Ki_n = -0.06f, .Kd_n = -0.03f,.Kp_p = 0.015f,.Ki_p = 0.008f,.Kd_p = 0.004f},
    /* NS */ {.Kp_n = -0.06f, .Ki_n = -0.04f, .Kd_n = -0.02f,.Kp_p = 0.01f, .Ki_p = 0.005f,.Kd_p = 0.002f},
    /* ZO */ {.Kp_n = -0.04f, .Ki_n = -0.03f, .Kd_n = -0.01f,.Kp_p = 0.008f,.Ki_p = 0.004f,.Kd_p = 0.001f},
    /* PS */ {.Kp_n = -0.02f, .Ki_n = -0.02f, .Kd_n = -0.008f,.Kp_p = 0.005f,.Ki_p = 0.002f,.Kd_p = 0.001f},
    /* PM */ {.Kp_n = -0.01f, .Ki_n = -0.01f, .Kd_n = -0.005f,.Kp_p = 0.003f,.Ki_p = 0.001f,.Kd_p = 0.0005f},
    /* PB */ {.Kp_n = -0.005f,.Ki_n = -0.005f,.Kd_n = -0.003f,.Kp_p = 0.002f,.Ki_p = 0.0008f,.Kd_p = 0.0003f}
};

/**
 * @brief Kd参数模糊规则表
 * @details 存储不同误差等级下Kd的修正量
 *         Kd用于抑制超调，修正量适中
 */
static const Fuzzy_Rule_Output Rule_Kd[FUZZY_SET_SIZE] = {
    /* NB */ {.Kp_n = -1.5f, .Ki_n = -1.0f, .Kd_n = -0.8f, .Kp_p = 0.4f,  .Ki_p = 0.2f,  .Kd_p = 0.15f},
    /* NM */ {.Kp_n = -1.0f, .Ki_n = -0.8f, .Kd_n = -0.5f, .Kp_p = 0.3f,  .Ki_p = 0.15f, .Kd_p = 0.1f},
    /* NS */ {.Kp_n = -0.8f, .Ki_n = -0.5f, .Kd_n = -0.3f, .Kp_p = 0.2f,  .Ki_p = 0.1f,  .Kd_p = 0.08f},
    /* ZO */ {.Kp_n = -0.5f, .Ki_n = -0.3f, .Kd_n = -0.2f, .Kp_p = 0.15f, .Ki_p = 0.08f, .Kd_p = 0.05f},
    /* PS */ {.Kp_n = -0.3f, .Ki_n = -0.2f, .Kd_n = -0.1f, .Kp_p = 0.1f,  .Ki_p = 0.05f, .Kd_p = 0.03f},
    /* PM */ {.Kp_n = -0.2f, .Ki_n = -0.1f, .Kd_n = -0.08f,.Kp_p = 0.08f, .Ki_p = 0.03f, .Kd_p = 0.02f},
    /* PB */ {.Kp_n = -0.1f, .Ki_n = -0.08f,.Kd_n = -0.05f,.Kp_p = 0.05f, .Ki_p = 0.02f, .Kd_p = 0.01f}
};

/*=============================== 编码器计数变量 ================================*/

static int32 encoder_total_L1 = 0;
static int32 encoder_total_L2 = 0;
static int32 encoder_total_R1 = 0;
static int32 encoder_total_R2 = 0;

static int16 encoder_last_L1 = 0;
static int16 encoder_last_L2 = 0;
static int16 encoder_last_R1 = 0;
static int16 encoder_last_R2 = 0;

/*=============================== 模糊PID控制器实例 ================================*/

static Fuzzy_Cascade_PID Fuzzy_Cascade_PID_L1;
static Fuzzy_Cascade_PID Fuzzy_Cascade_PID_L2;
static Fuzzy_Cascade_PID Fuzzy_Cascade_PID_R1;
static Fuzzy_Cascade_PID Fuzzy_Cascade_PID_R2;

/*=============================== 函数实现 ================================*/

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Init
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 模糊PID初始化函数，初始化所有PID参数为默认值
 * 应在系统启动时调用一次，通常在main函数的初始化阶段调用
 * 
 * 初始化的参数包括:
 *   - 位置环PID参数: Kp=15.0, Ki=0.5, Kd=8.0
 *   - 速度环PID参数: Kp=12.0, Ki=1.0, Kd=5.0
 *   - 输出限幅值
 *   - 积分限幅值
 *   - 位置计数清零
 * 
 * 调用示例:
 * ----------
 * @code
 * void System_Init(void)
 * {
 *     Motor_Init();        // 电机驱动初始化
 *     Encoder_Init();      // 编码器初始化
 *     Fuzzy_PID_Init();    // 模糊PID初始化(放在最后)
 * }
 * @endcode
 * 
 * 注意:
 * ----------
 * - 此函数必须在编码器和电机初始化之后调用
 * - 只在系统启动时调用一次，不需要重复调用
 * 
 * ============================================================================
 */
void Fuzzy_PID_Init(void)
{
    /*-------------------- 左电机位置环初始化 --------------------*/
    Fuzzy_Cascade_PID_L1.position_pid.Kp = FUZZY_KP_BASE;          /* 比例系数基础值 */
    Fuzzy_Cascade_PID_L1.position_pid.Ki = FUZZY_KI_BASE;          /* 积分系数基础值 */
    Fuzzy_Cascade_PID_L1.position_pid.Kd = FUZZY_KD_BASE;          /* 微分系数基础值 */
    Fuzzy_Cascade_PID_L1.position_pid.Pwm_Max_Out = POSITION_PID_MAX_OUT;  /* 位置环输出限幅 */
    Fuzzy_Cascade_PID_L1.position_pid.error_last = 0.0f;           /* 误差历史清零 */
    Fuzzy_Cascade_PID_L1.position_pid.error_prev2 = 0.0f;          /* 误差历史清零 */
    Fuzzy_Cascade_PID_L1.position_pid.output = 0.0f;               /* 输出清零 */
    Fuzzy_Cascade_PID_L1.position_pid.target = 0.0f;               /* 目标清零 */
    Fuzzy_Cascade_PID_L1.position_pid.actual = 0.0f;               /* 实际值清零 */
    Fuzzy_Cascade_PID_L1.position_pid.error_current = 0.0f;       /* 当前误差清零 */

    /*-------------------- 左电机速度环初始化 --------------------*/
    Fuzzy_Cascade_PID_L1.speed_pid.Kp = 12.0f;                     /* 速度环比例系数 */
    Fuzzy_Cascade_PID_L1.speed_pid.Ki = 1.0f;                     /* 速度环积分系数 */
    Fuzzy_Cascade_PID_L1.speed_pid.Kd = 5.0f;                      /* 速度环微分系数 */
    Fuzzy_Cascade_PID_L1.speed_pid.Pwm_Max_Out = SPEED_PID_MAX_OUT;        /* PWM输出限幅 */
    Fuzzy_Cascade_PID_L1.speed_pid.error_last = 0.0f;             /* 误差历史清零 */
    Fuzzy_Cascade_PID_L1.speed_pid.error_prev2 = 0.0f;             /* 误差历史清零 */
    Fuzzy_Cascade_PID_L1.speed_pid.output = 0.0f;                 /* 输出清零 */
    Fuzzy_Cascade_PID_L1.speed_pid.target = 0.0f;                 /* 目标清零 */
    Fuzzy_Cascade_PID_L1.speed_pid.actual = 0.0f;                 /* 实际值清零 */
    Fuzzy_Cascade_PID_L1.speed_pid.error_current = 0.0f;          /* 当前误差清零 */

    /*-------------------- 左前电机辅助信息 --------------------*/
    Fuzzy_Cascade_PID_L1.target_position = 0;
    Fuzzy_Cascade_PID_L1.actual_position = 0;
    Fuzzy_Cascade_PID_L1.motor_id = FUZZY_POS_PID_L1;
    Fuzzy_Cascade_PID_L1.dir_pin = MOTOR_L1_DIR;

    /*-------------------- 左后电机位置环初始化 --------------------*/
    Fuzzy_Cascade_PID_L2.position_pid.Kp = FUZZY_KP_BASE;
    Fuzzy_Cascade_PID_L2.position_pid.Ki = FUZZY_KI_BASE;
    Fuzzy_Cascade_PID_L2.position_pid.Kd = FUZZY_KD_BASE;
    Fuzzy_Cascade_PID_L2.position_pid.Pwm_Max_Out = POSITION_PID_MAX_OUT;
    Fuzzy_Cascade_PID_L2.position_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_L2.position_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_L2.position_pid.output = 0.0f;
    Fuzzy_Cascade_PID_L2.position_pid.target = 0.0f;
    Fuzzy_Cascade_PID_L2.position_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_L2.position_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_L2.speed_pid.Kp = 12.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.Ki = 1.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.Kd = 5.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.Pwm_Max_Out = SPEED_PID_MAX_OUT;
    Fuzzy_Cascade_PID_L2.speed_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.output = 0.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.target = 0.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_L2.speed_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_L2.target_position = 0;
    Fuzzy_Cascade_PID_L2.actual_position = 0;
    Fuzzy_Cascade_PID_L2.motor_id = FUZZY_POS_PID_L2;
    Fuzzy_Cascade_PID_L2.dir_pin = MOTOR_L2_DIR;

    /*-------------------- 右前电机位置环初始化 --------------------*/
    Fuzzy_Cascade_PID_R1.position_pid.Kp = FUZZY_KP_BASE;
    Fuzzy_Cascade_PID_R1.position_pid.Ki = FUZZY_KI_BASE;
    Fuzzy_Cascade_PID_R1.position_pid.Kd = FUZZY_KD_BASE;
    Fuzzy_Cascade_PID_R1.position_pid.Pwm_Max_Out = POSITION_PID_MAX_OUT;
    Fuzzy_Cascade_PID_R1.position_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_R1.position_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_R1.position_pid.output = 0.0f;
    Fuzzy_Cascade_PID_R1.position_pid.target = 0.0f;
    Fuzzy_Cascade_PID_R1.position_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_R1.position_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_R1.speed_pid.Kp = 12.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.Ki = 1.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.Kd = 5.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.Pwm_Max_Out = SPEED_PID_MAX_OUT;
    Fuzzy_Cascade_PID_R1.speed_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.output = 0.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.target = 0.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_R1.speed_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_R1.target_position = 0;
    Fuzzy_Cascade_PID_R1.actual_position = 0;
    Fuzzy_Cascade_PID_R1.motor_id = FUZZY_POS_PID_R1;
    Fuzzy_Cascade_PID_R1.dir_pin = MOTOR_R1_DIR;

    /*-------------------- 右后电机位置环初始化 --------------------*/
    Fuzzy_Cascade_PID_R2.position_pid.Kp = FUZZY_KP_BASE;
    Fuzzy_Cascade_PID_R2.position_pid.Ki = FUZZY_KI_BASE;
    Fuzzy_Cascade_PID_R2.position_pid.Kd = FUZZY_KD_BASE;
    Fuzzy_Cascade_PID_R2.position_pid.Pwm_Max_Out = POSITION_PID_MAX_OUT;
    Fuzzy_Cascade_PID_R2.position_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_R2.position_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_R2.position_pid.output = 0.0f;
    Fuzzy_Cascade_PID_R2.position_pid.target = 0.0f;
    Fuzzy_Cascade_PID_R2.position_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_R2.position_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_R2.speed_pid.Kp = 12.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.Ki = 1.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.Kd = 5.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.Pwm_Max_Out = SPEED_PID_MAX_OUT;
    Fuzzy_Cascade_PID_R2.speed_pid.error_last = 0.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.error_prev2 = 0.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.output = 0.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.target = 0.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.actual = 0.0f;
    Fuzzy_Cascade_PID_R2.speed_pid.error_current = 0.0f;

    Fuzzy_Cascade_PID_R2.target_position = 0;
    Fuzzy_Cascade_PID_R2.actual_position = 0;
    Fuzzy_Cascade_PID_R2.motor_id = FUZZY_POS_PID_R2;
    Fuzzy_Cascade_PID_R2.dir_pin = MOTOR_R2_DIR;
}

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Set_Target
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 设置指定电机的目标位置
 * 
 * 参数说明:
 * ----------
 * @param id          电机通道ID
 *                     - FUZZY_POS_PID_L1 (0): 左电机
 *                     - FUZZY_POS_PID_R1 (1): 右电机
 * 
 * @param target_pos  目标位置(单位: 脉冲数)
 *                     - 正值: 电机正转
 *                     - 负值: 电机反转
 * 
 * 使用示例:
 * ----------
 * @code
 * // 让左电机转动1圈 (4096脉冲)
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);
 * 
 * // 让右电机转动2圈 (8192脉冲)
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, 8192);
 * 
 * // 让左电机反转半圈 (-2048脉冲)
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, -2048);
 * 
 * // 让两电机同时转动相同距离
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, 4096);
 * @endcode
 * 
 * 注意事项:
 * ----------
 * - 目标位置是相对值，相对于上次清零后的位置
 * - 4096脉冲 = RC380电机转1圈 (1024线 × 4倍频)
 * - 可以同时设置多个电机的目标位置
 * 
 * ============================================================================
 */
void Fuzzy_PID_Set_Target(uint8 id, int32 target_pos)
{
    if (id == FUZZY_POS_PID_L1)
    {
        Fuzzy_Cascade_PID_L1.target_position = target_pos;
    }
    else if (id == FUZZY_POS_PID_L2)
    {
        Fuzzy_Cascade_PID_L2.target_position = target_pos;
    }
    else if (id == FUZZY_POS_PID_R1)
    {
        Fuzzy_Cascade_PID_R1.target_position = target_pos;
    }
    else if (id == FUZZY_POS_PID_R2)
    {
        Fuzzy_Cascade_PID_R2.target_position = target_pos;
    }
}

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Update_Position
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 更新指定电机的当前位置
 * 当使用外部位置传感器时可以调用此函数更新位置
 * 
 * 参数说明:
 * ----------
 * @param id           电机通道ID
 * @param current_pos  当前实际位置(脉冲数)
 * 
 * 注意:
 * ----------
 * - 如果使用内置编码器读取，此函数可选使用
 * - 此函数仅更新位置信息，不触发PID计算
 * 
 * ============================================================================
 */
void Fuzzy_PID_Update_Position(uint8 id, int32 current_pos)
{
    if (id == FUZZY_POS_PID_L1)
    {
        Fuzzy_Cascade_PID_L1.actual_position = current_pos;
    }
    else if (id == FUZZY_POS_PID_L2)
    {
        Fuzzy_Cascade_PID_L2.actual_position = current_pos;
    }
    else if (id == FUZZY_POS_PID_R1)
    {
        Fuzzy_Cascade_PID_R1.actual_position = current_pos;
    }
    else if (id == FUZZY_POS_PID_R2)
    {
        Fuzzy_Cascade_PID_R2.actual_position = current_pos;
    }
}

/**
 * ============================================================================
 * 函数: Fuzzy_Inference
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 模糊推理函数，将精确误差值模糊化后查表返回模糊输出
 * 
 * 算法流程:
 * ----------
 *   1. 输入限幅: 将误差和误差变化率限制在合理范围内
 *   2. 模糊化: 将精确值转换为模糊语言变量(量化)
 *              - 误差e: 范围[-1000, +1000] → 量化到[-3, +3]
 *              - 误差变化率ec: 范围[-500, +500] → 量化到[-3, +3]
 *   3. 查表: 根据e和ec的量化索引查模糊规则表
 *   4. 返回: 模糊语言变量值(-3 ~ +3)
 * 
 * 参数说明:
 * ----------
 * @param error          当前误差 (目标位置 - 实际位置)
 * @param error_change   误差变化率 (ec = e(k) - e(k-1))
 * 
 * @return float 模糊推理输出值 (-3 ~ +3)
 * 
 * 注意:
 * ----------
 * - 这是内部函数，通常不需要外部调用
 * - 模糊化公式: fuzzy_value = precise_value / (range / 6)
 * - 误差范围 ±1000, ec范围 ±500 是典型值，可根据实际系统调整
 * 
 * ============================================================================
 */
float Fuzzy_Inference(float error, float error_change)
{
    float error_fuzzy, ec_fuzzy;  /* 模糊化后的误差和误差变化率 */
    int8 error_idx, ec_idx;        /* 查表的索引 */

    /*-------------------- 输入限幅 --------------------*/
    /* 将输入值限制在合理范围内，防止量化时越界 */
    if (error > 1000.0f) error = 1000.0f;
    else if (error < -1000.0f) error = -1000.0f;
    if (error_change > 500.0f) error_change = 500.0f;
    else if (error_change < -500.0f) error_change = -500.0f;

    /*-------------------- 模糊化 --------------------*/
    /* 将精确值量化到模糊语言变量范围 [-3, +3] */
    /* 误差e: 1000 / 6 ≈ 166.67, ec: 500 / 6 ≈ 83.33 */
    error_fuzzy = error / 166.67f;
    ec_fuzzy = error_change / 83.33f;

    /* 限幅到[-3, +3]范围 */
    error_fuzzy = (error_fuzzy < -3.0f) ? -3.0f : (error_fuzzy > 3.0f ? 3.0f : error_fuzzy);
    ec_fuzzy = (ec_fuzzy < -3.0f) ? -3.0f : (ec_fuzzy > 3.0f ? 3.0f : ec_fuzzy);

    /*-------------------- 计算查表索引 --------------------*/
    /* 模糊语言变量到数组索引的转换: [-3,+3] → [0,6] */
    error_idx = (int8)(error_fuzzy + 3.0f);
    ec_idx = (int8)(ec_fuzzy + 3.0f);

    /* 索引边界检查 */
    error_idx = (error_idx < 0) ? 0 : (error_idx > 6 ? 6 : error_idx);
    ec_idx = (ec_idx < 0) ? 0 : (ec_idx > 6 ? 6 : ec_idx);

    /*-------------------- 查表返回 --------------------*/
    return (float)Fuzzy_Table[error_idx][ec_idx];
}

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Adjust
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 模糊PID参数调整函数，根据误差和误差变化率自动调整PID参数
 * 
 * 算法流程:
 * ----------
 *   1. 误差和误差变化率限幅
 *   2. 模糊化误差和误差变化率
 *   3. 调用Fuzzy_Inference获取模糊规则输出
 *   4. 根据规则索引查Rule_Kp/Rule_Ki/Rule_Kd表获取修正量
 *   5. 计算最终PID参数: K = K_base + correction × rule_level
 *   6. 参数下限保护
 * 
 * 参数说明:
 * ----------
 * @param error          当前误差
 * @param error_change   误差变化率
 * @param Kp            输出: 调整后的比例系数
 * @param Ki            输出: 调整后的积分系数
 * @param Kd            输出: 调整后的微分系数
 * 
 * 调参原理:
 * ----------
 * - 当误差大且误差变化率大时: 增大Kp加速响应，减小Ki防止超调
 * - 当误差小且误差变化率小时: 增大Ki消除稳态误差
 * - 当误差趋于目标时: 适当增大Kd抑制可能出现的超调
 * 
 * 注意:
 * ----------
 * - 这是内部函数，由Cascade_PID_Calculate自动调用
 * - PID参数下限保护，防止参数过小导致响应迟钝
 * 
 * ============================================================================
 */
void Fuzzy_PID_Adjust(float error, float error_change, float *Kp, float *Ki, float *Kd)
{
    int8 rule_idx, error_idx, ec_idx;  /* 规则索引和模糊化索引 */
    float rule_value;                   /* 模糊推理输出 */
    float Kp_rule, Ki_rule, Kd_rule;   /* 查表得到的修正量 */
    float error_fuzzy, ec_fuzzy;       /* 模糊化后的值 */

    /*-------------------- 输入限幅 --------------------*/
    /* 位置环的误差和误差变化率范围更大 */
    if (error > 5000.0f) error = 5000.0f;
    else if (error < -5000.0f) error = -5000.0f;
    if (error_change > 2000.0f) error_change = 2000.0f;
    else if (error_change < -2000.0f) error_change = -2000.0f;

    /*-------------------- 模糊化 --------------------*/
    /* 误差: 5000 / 6 ≈ 833.33, 误差变化率: 2000 / 6 ≈ 333.33 */
    error_fuzzy = error / 833.33f;
    ec_fuzzy = error_change / 333.33f;

    /* 限幅到[-3, +3]范围 */
    error_fuzzy = (error_fuzzy < -3.0f) ? -3.0f : (error_fuzzy > 3.0f ? 3.0f : error_fuzzy);
    ec_fuzzy = (ec_fuzzy < -3.0f) ? -3.0f : (ec_fuzzy > 3.0f ? 3.0f : ec_fuzzy);

    /*-------------------- 计算索引 --------------------*/
    error_idx = (int8)(error_fuzzy + 3.0f);
    ec_idx = (int8)(ec_fuzzy + 3.0f);
    error_idx = (error_idx < 0) ? 0 : (error_idx > 6 ? 6 : error_idx);
    ec_idx = (ec_idx < 0) ? 0 : (ec_idx > 6 ? 6 : ec_idx);

    /*-------------------- 模糊推理 --------------------*/
    rule_value = Fuzzy_Inference(error, error_change);
    rule_idx = (int8)rule_value;

    /*-------------------- 查表获取修正量 --------------------*/
    /* 根据规则输出值的正负选择不同的修正量 */
    if (rule_idx >= 0)
    {
        Kp_rule = Rule_Kp[error_idx].Kp_p;
        Ki_rule = Rule_Ki[error_idx].Ki_p;
        Kd_rule = Rule_Kd[error_idx].Kd_p;
    }
    else
    {
        rule_idx = -rule_idx;  /* 取绝对值 */
        Kp_rule = Rule_Kp[error_idx].Kp_n;
        Ki_rule = Rule_Ki[error_idx].Ki_n;
        Kd_rule = Rule_Kd[error_idx].Kd_n;
    }

    /*-------------------- 计算最终PID参数 --------------------*/
    /* K_final = K_base + K_correction × rule_level */
    *Kp = FUZZY_KP_BASE + Kp_rule * rule_idx;
    *Ki = FUZZY_KI_BASE + Ki_rule * rule_idx;
    *Kd = FUZZY_KD_BASE + Kd_rule * rule_idx;

    /*-------------------- 参数下限保护 --------------------*/
    /* 防止参数过小导致响应迟钝 */
    if (*Kp < 0.1f) *Kp = 0.1f;
    if (*Ki < 0.01f) *Ki = 0.01f;
    if (*Kd < 0.1f) *Kd = 0.1f;
}

/**
 * ============================================================================
 * 函数: Cascade_PID_Calculate
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 串级PID计算函数，执行位置环和速度环的PID运算
 * 
 * 计算流程:
 * ----------
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │                      位置环计算                                   │
 *   │  1. 计算位置误差 e_pos = target - actual                         │
 *   │  2. 模糊调整PID参数                                               │
 *   │  3. 增量式PID计算得到目标速度                                     │
 *   │  4. 输出限幅                                                      │
 *   └──────────────────────────────────────────────────────────────────┘
 *                              │
 *                              ▼
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │                      速度环计算                                   │
 *   │  1. 速度环目标 = 位置环输出                                       │
 *   │  2. 计算速度误差 e_speed = target - actual                       │
 *   │  3. 增量式PID计算得到PWM输出                                      │
 *   │  4. 输出限幅                                                      │
 *   └──────────────────────────────────────────────────────────────────┘
 *                              │
 *                              ▼
 *                        PWM输出值
 * 
 * PID公式(增量式):
 * ----------
 *   Δu(k) = Kp×[e(k)-e(k-1)] + Ki×Σe(k) + Kd×[e(k)-2e(k-1)+e(k-2)]
 *   u(k) = u(k-1) + Δu(k)
 * 
 * 参数说明:
 * ----------
 * @param pid  指向Fuzzy_Cascade_PID结构体的指针
 * 
 * 注意:
 * ----------
 * - 这是内部函数，由Fuzzy_PID_Calculate调用
 * - 位置环输出限幅: ±500 (作为速度环的目标速度上限)
 * - 速度环输出限幅: ±8000 (作为PWM的限幅)
 * 
 * ============================================================================
 */
static void Cascade_PID_Calculate(Fuzzy_Cascade_PID *pid)
{
    float pos_error, pos_error_change;   /* 位置误差和误差变化率 */
    float speed_error;                    /* 速度误差 */
    float pos_increment, speed_increment; /* PID增量 */

    /*==================== 位置环计算 ====================*/

    /* 更新误差历史: e(k-2) <- e(k-1) <- e(k) */
    pid->position_pid.error_prev2 = pid->position_pid.error_last;
    pid->position_pid.error_last = pid->position_pid.error_current;

    /* 计算位置误差 */
    pos_error = (float)(pid->target_position - pid->actual_position);
    pid->position_pid.error_current = pos_error;

    /* 计算误差变化率 */
    pos_error_change = pos_error - pid->position_pid.error_last;

    /* 模糊调整PID参数 */
    Fuzzy_PID_Adjust(pos_error, pos_error_change, 
                     &pid->position_pid.Kp, 
                     &pid->position_pid.Ki, 
                     &pid->position_pid.Kd);

    /* 增量式PID计算 */
    /* Δu = Kp×[e(k)-e(k-1)] + Ki×e(k) + Kd×[e(k)-2e(k-1)+e(k-2)] */
    pos_increment = pid->position_pid.Kp * (pos_error - pid->position_pid.error_last) +
                    pid->position_pid.Ki * pos_error +
                    pid->position_pid.Kd * (pos_error - 2.0f * pid->position_pid.error_last + pid->position_pid.error_prev2);

    /* 累积输出 */
    pid->position_pid.output += pos_increment;

    /* 输出限幅(位置环输出作为速度环目标，限幅±500) */
    if (pid->position_pid.output > pid->position_pid.Pwm_Max_Out)
        pid->position_pid.output = pid->position_pid.Pwm_Max_Out;
    else if (pid->position_pid.output < -pid->position_pid.Pwm_Max_Out)
        pid->position_pid.output = -pid->position_pid.Pwm_Max_Out;

    /* 位置环输出作为速度环目标 */
    pid->speed_pid.target = pid->position_pid.output;

    /*==================== 速度环计算 ====================*/

    /* 更新误差历史 */
    pid->speed_pid.error_prev2 = pid->speed_pid.error_last;
    pid->speed_pid.error_last = pid->speed_pid.error_current;

    /* 计算速度误差 */
    speed_error = pid->speed_pid.target - pid->speed_pid.actual;
    pid->speed_pid.error_current = speed_error;

    /* 增量式PID计算 */
    speed_increment = pid->speed_pid.Kp * (speed_error - pid->speed_pid.error_last) +
                      pid->speed_pid.Ki * speed_error +
                      pid->speed_pid.Kd * (speed_error - 2.0f * pid->speed_pid.error_last + pid->speed_pid.error_prev2);

    /* 累积输出 */
    pid->speed_pid.output += speed_increment;

    /* 输出限幅(速度环输出是PWM值，限幅±8000) */
    if (pid->speed_pid.output > pid->speed_pid.Pwm_Max_Out)
        pid->speed_pid.output = pid->speed_pid.Pwm_Max_Out;
    else if (pid->speed_pid.output < -pid->speed_pid.Pwm_Max_Out)
        pid->speed_pid.output = -pid->speed_pid.Pwm_Max_Out;
}

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Calculate
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 模糊PID主计算函数，在定时中断中周期调用
 * 读取编码器数据，执行串级PID计算
 * 
 * 参数说明:
 * ----------
 * @param id  电机通道选择
 *            - FUZZY_POS_PID_L1 (0): 只更新左电机
 *            - FUZZY_POS_PID_R1 (1): 只更新右电机
 *            - 0xFF: 更新所有电机
 * 
 * 使用示例:
 * ----------
 * @code
 * // 在定时器中断中调用(推荐周期10ms)
 * void TIM3_IRQHandler(void)
 * {
 *     if(timer_check_interrupt(TIM_3))
 *     {
 *         // 清除中断标志
 *         timer_clear_interrupt_flag(TIM_3);
 *         
 *         // 更新所有电机
 *         Fuzzy_PID_Calculate(0xFF);
 *     }
 * }
 * @endcode
 * 
 * 注意事项:
 * ----------
 * - 此函数必须在定时中断中周期调用
 * - 推荐调用周期: 5ms ~ 20ms
 * - 周期太短会增加CPU负担，周期太长会影响控制精度
 * 
 * ============================================================================
 */
void Fuzzy_PID_Calculate(uint8 id)
{
    int16 current_count_L1, current_count_L2, current_count_R1, current_count_R2;
    int16 delta_L1, delta_L2, delta_R1, delta_R2;

    /*==================== 读取并更新编码器数据 ====================*/

    current_count_L1 = encoder_get_count(QTIMER1_ENCODER1);
    delta_L1 = current_count_L1 - encoder_last_L1;
    encoder_total_L1 += delta_L1;
    encoder_last_L1 = current_count_L1;
    encoder_clear_count(QTIMER1_ENCODER1);

    current_count_L2 = encoder_get_count(QTIMER1_ENCODER2);
    delta_L2 = current_count_L2 - encoder_last_L2;
    encoder_total_L2 += delta_L2;
    encoder_last_L2 = current_count_L2;
    encoder_clear_count(QTIMER1_ENCODER2);

    current_count_R1 = encoder_get_count(QTIMER2_ENCODER1);
    delta_R1 = current_count_R1 - encoder_last_R1;
    encoder_total_R1 += delta_R1;
    encoder_last_R1 = current_count_R1;
    encoder_clear_count(QTIMER2_ENCODER1);

    current_count_R2 = encoder_get_count(QTIMER2_ENCODER2);
    delta_R2 = current_count_R2 - encoder_last_R2;
    encoder_total_R2 += delta_R2;
    encoder_last_R2 = current_count_R2;
    encoder_clear_count(QTIMER2_ENCODER2);

    /*==================== PID计算 ====================*/

    if (id == FUZZY_POS_PID_L1 || id == 0xFF)
    {
        Fuzzy_Cascade_PID_L1.speed_pid.actual = (float)delta_L1;
        Fuzzy_Cascade_PID_L1.actual_position = encoder_total_L1;
        Cascade_PID_Calculate(&Fuzzy_Cascade_PID_L1);
    }

    if (id == FUZZY_POS_PID_L2 || id == 0xFF)
    {
        Fuzzy_Cascade_PID_L2.speed_pid.actual = (float)delta_L2;
        Fuzzy_Cascade_PID_L2.actual_position = encoder_total_L2;
        Cascade_PID_Calculate(&Fuzzy_Cascade_PID_L2);
    }

    if (id == FUZZY_POS_PID_R1 || id == 0xFF)
    {
        Fuzzy_Cascade_PID_R1.speed_pid.actual = (float)delta_R1;
        Fuzzy_Cascade_PID_R1.actual_position = encoder_total_R1;
        Cascade_PID_Calculate(&Fuzzy_Cascade_PID_R1);
    }

    if (id == FUZZY_POS_PID_R2 || id == 0xFF)
    {
        Fuzzy_Cascade_PID_R2.speed_pid.actual = (float)delta_R2;
        Fuzzy_Cascade_PID_R2.actual_position = encoder_total_R2;
        Cascade_PID_Calculate(&Fuzzy_Cascade_PID_R2);
    }
}

/**
 * ============================================================================
 * 函数: Fuzzy_PID_Get_Output
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 获取指定电机的PID输出值
 * 
 * 返回值说明:
 * ----------
 * @return Cascade_PID_Output 结构体
 *         - position_out: 位置环输出(目标速度)
 *         - speed_out: 速度环输出(电机PWM值)
 * 
 * 使用示例:
 * ----------
 * @code
 * // 获取左电机输出
 * Cascade_PID_Output out_l = Fuzzy_PID_Get_Output(FUZZY_POS_PID_L1);
 * 
 * // 获取右电机输出
 * Cascade_PID_Output out_r = Fuzzy_PID_Get_Output(FUZZY_POS_PID_R1);
 * 
 * // 设置电机PWM
 * Motor_Set_PWM(MOTOR_L1, (int16)out_l.speed_out);
 * Motor_Set_PWM(MOTOR_R1, (int16)out_r.speed_out);
 * 
 * // 或者使用速度环输出的符号控制方向
 * int16 left_pwm = (int16)out_l.speed_out;
 * gpio_set_level(MOTOR_L1_DIR, left_pwm >= 0 ? 1 : 0);  // 设置方向
 * Motor_Set_PWM(MOTOR_L1, abs(left_pwm));                 // 设置速度
 * @endcode
 * 
 * 注意:
 * ----------
 * - speed_out是最终的PWM输出值
 * - 正值表示正转，负值表示反转
 * - 绝对值越大转速越快
 * 
 * ============================================================================
 */
Cascade_PID_Output Fuzzy_PID_Get_Output(uint8 id)
{
    Cascade_PID_Output output = {0.0f, 0.0f};

    if (id == FUZZY_POS_PID_L1)
    {
        output.position_out = Fuzzy_Cascade_PID_L1.position_pid.output;
        output.speed_out = Fuzzy_Cascade_PID_L1.speed_pid.output;
    }
    else if (id == FUZZY_POS_PID_L2)
    {
        output.position_out = Fuzzy_Cascade_PID_L2.position_pid.output;
        output.speed_out = Fuzzy_Cascade_PID_L2.speed_pid.output;
    }
    else if (id == FUZZY_POS_PID_R1)
    {
        output.position_out = Fuzzy_Cascade_PID_R1.position_pid.output;
        output.speed_out = Fuzzy_Cascade_PID_R1.speed_pid.output;
    }
    else if (id == FUZZY_POS_PID_R2)
    {
        output.position_out = Fuzzy_Cascade_PID_R2.position_pid.output;
        output.speed_out = Fuzzy_Cascade_PID_R2.speed_pid.output;
    }

    return output;
}

/**
 * ============================================================================
 * 函数: Get_Motor_Position
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 获取编码器累计的脉冲数(即电机当前位置)
 * 
 * 参数说明:
 * ----------
 * @param id  电机通道ID
 *            - FUZZY_POS_PID_L1 (0): 左电机
 *            - FUZZY_POS_PID_R1 (1): 右电机
 * 
 * @return int32 当前位置(脉冲数)
 * 
 * 使用示例:
 * ----------
 * @code
 * // 获取并显示电机位置
 * int32 pos_l = Get_Motor_Position(FUZZY_POS_PID_L1);
 * int32 pos_r = Get_Motor_Position(FUZZY_POS_PID_R1);
 * 
 * printf("左电机位置: %d 脉冲 (%.2f 圈)\n", pos_l, pos_l / 4096.0f);
 * printf("右电机位置: %d 脉冲 (%.2f 圈)\n", pos_r, pos_r / 4096.0f);
 * @endcode
 * 
 * 注意:
 * ----------
 * - 4096脉冲 = RC380电机转1圈
 * - 位置是累计值，需要定期清零
 * 
 * ============================================================================
 */
int32 Get_Motor_Position(uint8 id)
{
    if (id == FUZZY_POS_PID_L1)
        return encoder_total_L1;
    else if (id == FUZZY_POS_PID_L2)
        return encoder_total_L2;
    else if (id == FUZZY_POS_PID_R1)
        return encoder_total_R1;
    else if (id == FUZZY_POS_PID_R2)
        return encoder_total_R2;
    return 0;
}

/**
 * ============================================================================
 * 函数: Clear_Motor_Position
 * ============================================================================
 * 
 * 功能说明:
 * ----------
 * 清零指定电机的位置计数器
 * 
 * 参数说明:
 * ----------
 * @param id  电机通道ID
 *            - FUZZY_POS_PID_L1 (0): 只清零左电机
 *            - FUZZY_POS_PID_R1 (1): 只清零右电机
 *            - 0xFF: 清零所有电机
 * 
 * 使用示例:
 * ----------
 * @code
 * // 方法1: 清零单个电机
 * Clear_Motor_Position(FUZZY_POS_PID_L1);
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);  // 从0开始转1圈
 * 
 * // 方法2: 清零所有电机(用于初始化或回零)
 * Clear_Motor_Position(0xFF);
 * 
 * // 方法3: 在某个位置清零，建立新的参考零点
 * Motor_Move_To(5000);  // 移动到某位置
 * Clear_Motor_Position(FUZZY_POS_PID_L1);  // 当前位置变为0
 * Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 0);  // 停在当前位置
 * @endcode
 * 
 * 注意事项:
 * ----------
 * - 清零后之前的位置信息会丢失
 * - 可以在电机到达原点时调用清零
 * 
 * ============================================================================
 */
void Clear_Motor_Position(uint8 id)
{
    if (id == FUZZY_POS_PID_L1 || id == 0xFF)
    {
        encoder_total_L1 = 0;
        encoder_last_L1 = 0;
    }
    if (id == FUZZY_POS_PID_L2 || id == 0xFF)
    {
        encoder_total_L2 = 0;
        encoder_last_L2 = 0;
    }
    if (id == FUZZY_POS_PID_R1 || id == 0xFF)
    {
        encoder_total_R1 = 0;
        encoder_last_R1 = 0;
    }
    if (id == FUZZY_POS_PID_R2 || id == 0xFF)
    {
        encoder_total_R2 = 0;
        encoder_last_R2 = 0;
    }
}
