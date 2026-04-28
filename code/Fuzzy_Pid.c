/**
 * @file  Fuzzy_Pid.c
 * @brief 模糊PD位置环控制器实现 — 逐飞RC380 + 1024线正交编码器
 *
 * ─────────────────────────────────────────────────────────────────
 *  控制算法: 位置式PD，无积分项
 *
 *    u(k) = Kp(e,ec) × e(k)  +  Kd(e,ec) × ec(k)
 *    ec(k) = e(k) - e(k-1)
 *
 *  其中 Kp、Kd 由模糊推理根据当前误差实时整定，不固定。
 * ─────────────────────────────────────────────────────────────────
 *  模糊推理完整流程:
 *
 *    ① 量化 (Fuzzification)
 *         e  / FUZZY_E_SCALE  ∈ [-3.0, +3.0]  → 索引 e_idx  ∈ [0,6]
 *         ec / FUZZY_EC_SCALE ∈ [-3.0, +3.0]  → 索引 ec_idx ∈ [0,6]
 *       7个模糊等级: NB=0 NM=1 NS=2 ZO=3 PS=4 PM=5 PB=6
 *
 *    ② 规则推理 (Rule Inference)
 *         level_Kp = Kp_Rule[e_idx][ec_idx]   ∈ [0, 6]
 *         level_Kd = Kd_Rule[e_idx][ec_idx]   ∈ [0, 6]
 *
 *    ③ 反模糊化 (Defuzzification) — 线性映射
 *         Kp = Kp_MIN + (Kp_MAX - Kp_MIN) × level_Kp / 6
 *         Kd = Kd_MIN + (Kd_MAX - Kd_MIN) × level_Kd / 6
 *
 *    ④ PD输出
 *         output = Kp × e + Kd × ec
 *         output = clamp(output, -PWM_MAX, +PWM_MAX)
 * ─────────────────────────────────────────────────────────────────
 *  编码器数据来源:
 *    Ecoder_Total_L1/L2/R1/R2 定义于 Encoder.c，每个控制周期由
 *    Encoder_Speed_PID_Update() 累加，本模块直接读取。
 * ─────────────────────────────────────────────────────────────────
 *  标准调用顺序 (在定时中断中):
 *    Encoder_Speed_PID_Update();    // 1. 先更新编码器
 *    Fuzzy_PID_Calculate(0xFF);     // 2. 再计算PD
 *    Fuzzy_PID_Drive_All_Motor();   // 3. 最后驱动电机
 */

#include "Fuzzy_Pid.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"
#include <math.h>     /* [v2] floorf, fabsf */

/* =====================================================================
 *  模糊规则表
 *
 *  行索引 = e  量化等级: 0=NB  1=NM  2=NS  3=ZO  4=PS  5=PM  6=PB
 *  列索引 = ec 量化等级: 0=NB  1=NM  2=NS  3=ZO  4=PS  5=PM  6=PB
 *  表值范围 [0, 6]，经线性反模糊化映射到对应参数区间
 * ===================================================================== */

/*
 * Kp 规则表
 *
 * 设计原则:
 *   1. e 与 ec 同号 (系统正在远离目标):
 *      → 取最大 Kp (值=6)，以最大驱动力拉回
 *      → 例: e=NB(大负误差) 且 ec=NB(误差继续变负) → Kp_Rule[0][0] = 6
 *
 *   2. e 与 ec 反号 (系统正在趋近目标):
 *      → 降低 Kp (值=2~3)，减小超调风险
 *      → 例: e=PB(大正误差) 且 ec=NB(快速逼近) → Kp_Rule[6][0] = 2
 *
 *   3. e ≈ ZO (已在目标附近):
 *      → 最小 Kp (值=0~1)，配合 KP_MIN=0.05 让接近目标时 P 项基本归零
 *      → [v2修正] 旧版 ZO 行边沿仍取 2,导致 Kp≈2.0 在小误差段持续饱和
 *      → 新版 e=ZO 且 ec=ZO → Kp_Rule[3][3] = 0,真正的"到位即松油门"
 *
 * 参数映射: level=0 → Kp=0.05, level=6 → Kp=5.0  [v2: KP_MIN由0.5降至0.05]
 *
 *        ec:  NB  NM  NS  ZO  PS  PM  PB
 */
static const uint8 Kp_Rule[FUZZY_SET_SIZE][FUZZY_SET_SIZE] = {
    /* e=NB */ { 6,  6,  5,  5,  4,  3,  2 },
    /* e=NM */ { 5,  5,  5,  4,  3,  3,  2 },
    /* e=NS */ { 4,  4,  3,  3,  2,  2,  1 },
    /* e=ZO */ { 1,  1,  0,  0,  0,  1,  1 },   /* [v2修正] 原 {2,2,2,1,2,2,2} */
    /* e=PS */ { 1,  2,  2,  3,  3,  4,  4 },
    /* e=PM */ { 2,  3,  3,  4,  5,  5,  5 },
    /* e=PB */ { 2,  3,  4,  5,  5,  6,  6 },
};

/*
 * Kd 规则表
 *
 * 设计原则:
 *   1. |ec| 大 (系统运动速度快):
 *      → 取大 Kd (值=5~6)，提供足够阻尼防止超调
 *      → 例: 任意 e 且 ec=NB → Kd 列值 = 5~6
 *
 *   2. e≈ZO 且 |ec| 大 (在目标附近快速抖动):
 *      → 取最大 Kd (值=6)，快速平息振荡
 *      → 例: e=ZO 且 ec=NB → Kd_Rule[3][0] = 6
 *
 *   3. ec≈ZO (系统几乎静止):
 *      → 取最小 Kd (值=1)，此时 Kd×ec≈0，避免放大噪声
 *      → 例: 任意 e 且 ec=ZO → Kd 列值 = 1~2
 *
 * 参数映射: level=0 → Kd=0.05, level=6 → Kd=1.5
 *
 *        ec:  NB  NM  NS  ZO  PS  PM  PB
 */
static const uint8 Kd_Rule[FUZZY_SET_SIZE][FUZZY_SET_SIZE] = {
    /* e=NB */ { 5,  4,  3,  1,  3,  4,  5 },
    /* e=NM */ { 5,  4,  3,  1,  3,  4,  5 },
    /* e=NS */ { 5,  5,  4,  2,  4,  5,  5 },
    /* e=ZO */ { 6,  5,  4,  1,  4,  5,  6 },
    /* e=PS */ { 5,  5,  4,  2,  4,  5,  5 },
    /* e=PM */ { 5,  4,  3,  1,  3,  4,  5 },
    /* e=PB */ { 5,  4,  3,  1,  3,  4,  5 },
};

/* =====================================================================
 *  编码器外部变量 (定义于 Encoder.c，由 Encoder_Speed_PID_Update 更新)
 * ===================================================================== */
extern int32 Ecoder_Total_L1;
extern int32 Ecoder_Total_L2;
extern int32 Ecoder_Total_R1;
extern int32 Ecoder_Total_R2;

/* =====================================================================
 *  4路控制器实例 
 * ===================================================================== */
Fuzzy_PD_Controller pd_L1, pd_L2, pd_R1, pd_R2;

/* =====================================================================
 *  内部函数实现
 * ===================================================================== */

/**
 * @brief  将精确浮点值量化为 7 级模糊索引 [0, 6]
 *
 * 量化公式:
 *   fv  = clamp(value / scale, -3.0, +3.0)
 *   idx = (int)(fv + 3.0)
 *
 * 映射关系 (以 FUZZY_E_SCALE=833.3 为例，e范围±2500):
 *   value ≤ -2500  → idx=0 (NB)
 *   value ≈ -1667  → idx=1 (NM)
 *   value ≈  -833  → idx=2 (NS)
 *   value ≈     0  → idx=3 (ZO)
 *   value ≈  +833  → idx=4 (PS)
 *   value ≈ +1667  → idx=5 (PM)
 *   value ≥ +2500  → idx=6 (PB)
 *
 * @param value  待量化的精确值 (误差脉冲数 或 误差变化率)
 * @param scale  量化比例系数 (FUZZY_E_SCALE 或 FUZZY_EC_SCALE)
 * @return int8  模糊等级索引，范围 [0, 6]
 *
 * [v2修正] 取整方式由截断改为四舍五入:
 *   旧实现 (int8)(fv+3.0) 对负数有偏置,例如:
 *     fv = -0.4 → idx = (int)(2.6) = 2 (NS)   ← 期望应为 ZO
 *     fv = +0.4 → idx = (int)(3.4) = 3 (ZO)   ← 正确
 *   导致小误差段规则选择左右不对称,加剧极限环。
 *   新实现使用 floorf(fv + 3.5f) 实现真正的四舍五入对齐。
 */
static int8 fuzzify(float value, float scale)
{
    float fv = value / scale;
    int8  idx;

    /* 将模糊值限幅到 [-3.0, +3.0]，防止越界 */
    if (fv >  3.0f) fv =  3.0f;
    if (fv < -3.0f) fv = -3.0f;

    /* [v2] 四舍五入: floor(fv + 3.5) 等价于 round(fv) + 3 */
    idx = (int8)floorf(fv + 3.5f);
    if (idx < 0) idx = 0;
    if (idx > 6) idx = 6;
    return idx;
}

/**
 * @brief  模糊推理核心: 根据位置误差 e 和误差变化率 ec 实时整定 Kp、Kd
 *
 * 完整步骤:
 *   ① 调用 fuzzify() 将 e 和 ec 各量化为 [0,6] 的整数索引
 *   ② 以 (e_idx, ec_idx) 为坐标分别查 Kp_Rule、Kd_Rule，得到模糊等级 [0,6]
 *   ③ 对模糊等级做线性反模糊化，映射到物理参数范围:
 *        Kp = Kp_MIN + (Kp_MAX - Kp_MIN) × level / 6
 *           = 0.5   + 4.5 × level / 6     ∈ [0.5,  5.0]
 *        Kd = Kd_MIN + (Kd_MAX - Kd_MIN) × level / 6
 *           = 0.05  + 1.45 × level / 6    ∈ [0.05, 1.5]
 *
 * 示例 (以10ms中断、误差e=500脉冲、上周期误差600脉冲为例):
 *   e  = 500,  e_idx  = fuzzify(500,  833.3) = (int)(0.6+3) = 3 → ZO
 *   ec = 500-600 = -100, ec_idx = fuzzify(-100, 333.3) = (int)(-0.3+3) = 2 → NS
 *   level_Kp = Kp_Rule[3][2] = 2  → Kp = 0.5 + 4.5×2/6 = 2.0
 *   level_Kd = Kd_Rule[3][2] = 4  → Kd = 0.05 + 1.45×4/6 ≈ 1.02
 *
 * @param e       位置误差 (脉冲数)，= target_position - actual_position
 * @param ec      误差变化率 (脉冲/周期)，= e(k) - e(k-1)
 * @param kp_out  [输出] 整定后的比例增益 Kp ∈ [FUZZY_KP_MIN, FUZZY_KP_MAX]
 * @param kd_out  [输出] 整定后的微分增益 Kd ∈ [FUZZY_KD_MIN, FUZZY_KD_MAX]
 */
static void fuzzy_adjust_gains(float e, float ec, float *kp_out, float *kd_out)
{
    int8  e_idx  = fuzzify(e,  FUZZY_E_SCALE);
    int8  ec_idx = fuzzify(ec, FUZZY_EC_SCALE);
    float kp_lv  = (float)Kp_Rule[e_idx][ec_idx];   /* 规则表输出 0~6 */
    float kd_lv  = (float)Kd_Rule[e_idx][ec_idx];   /* 规则表输出 0~6 */

    /* 线性反模糊化: level=0 → 参数最小值, level=6 → 参数最大值 */
    *kp_out = FUZZY_KP_MIN + (FUZZY_KP_MAX - FUZZY_KP_MIN) * kp_lv / 6.0f;
    *kd_out = FUZZY_KD_MIN + (FUZZY_KD_MAX - FUZZY_KD_MIN) * kd_lv / 6.0f;
}

/**
 * @brief  单步位置式PD计算 (无积分)
 *
 * 公式:
 *   e(k)   = target_position - actual_position  (当前位置误差)
 *   ec(k)  = e(k) - e(k-1)                      (误差变化量，近似微分)
 *   u(k)   = Kp × e(k) + Kd × ec(k)             (PD输出)
 *   u(k)   = clamp(u(k), -pwm_max, +pwm_max)     (输出限幅)
 *
 * 关于"位置式PD"与"增量式PD"的区别:
 *   位置式: 输出 u 直接代表所需 PWM 绝对值
 *     → 当 e=0 时 u 自然归零，无需额外清零逻辑
 *     → 更适合位置控制场景
 *   增量式: 每步输出 Δu，需要 u(k)=u(k-1)+Δu 累积
 *     → 适合速度控制，但位置控制易产生积累漂移
 *
 * @param c  指向目标控制器实例的指针
 *
 * @note 函数末尾将 error_last 更新为本步 e，供下次调用计算 ec 使用。
 */
static void pd_step(Fuzzy_PD_Controller *c)
{
    /* ① 计算当前误差 */
    float e  = (float)(c->target_position - c->actual_position);
    float abs_e = fabsf(e);

    /*
     * [v6] 死区状态锁存机制
     *
     *   状态1: db_active=1 (锁定态)
     *     输出强制 0,直到 |e| > DB_EXIT 才解锁
     *
     *   状态2: db_active=0 (正常态)
     *     连续 DB_HOLD_CYCLES 周期 |e| < DB_ENTER 后转为锁定
     *
     *   关键改进对比 v5:
     *     - 滞回机制: ENTER=8, EXIT=30,  8~30脉冲扰动被忽略
     *     - 持续判定: 5周期(50ms)真到位才锁,避免瞬时穿越误锁
     *     - 不再重置 ec_filt: 出区时模糊推理感知系统状态正确,
     *                         避免 v5 那种"冷启动满前馈"形成极限环
     */
    if (c->db_active) {
        if (abs_e > (float)FUZZY_DB_EXIT) {
            /* 偏差超过出区阈值,解锁 */
            c->db_active = 0;
            c->db_steady_count = 0;
            /* [v7] 解锁瞬间不重置 I — 累积的 I 反映了静态偏置的方向,
             *      可作为出区瞬间的"前馈预估",有助于快速重新跟踪。
             *      若长时间锁定后扰动方向反转,I 会被新误差自然修正。 */
        } else {
            /* 锁定状态:强制零输出,平滑过渡(slew limit 仍生效) */
            float target_out = 0.0f;
            float delta = target_out - c->output;
            if (delta >  FUZZY_PWM_SLEW_MAX) delta =  FUZZY_PWM_SLEW_MAX;
            if (delta < -FUZZY_PWM_SLEW_MAX) delta = -FUZZY_PWM_SLEW_MAX;
            c->output += delta;
            c->error_last = e;
            /* [v7] 锁定状态下积分缓慢衰减,防止扰动期间累积过大,
             *      解锁瞬间 I 主导输出造成冲击。每周期衰减 5%,
             *      持续锁定 60 周期后 I 衰减到原值 5% 以下。 */
            c->integral *= 0.95f;
            return;
        }
    } else {
        /* 正常态:检查是否进入锁定 */
        if (abs_e < (float)FUZZY_DB_ENTER) {
            c->db_steady_count++;
            if (c->db_steady_count >= FUZZY_DB_HOLD_CYCLES) {
                c->db_active = 1;
                /* 不立即清零,通过下一周期 slew limit 平滑降到 0 */
            }
        } else {
            c->db_steady_count = 0;
        }
    }

    /* ② 计算误差变化率 (离散微分) */
    float ec_raw = e - c->error_last;

    /*
     * [v2新增, v5提为宏, v6保留] ec 一阶低通滤波:
     *   死区内不再重置滤波器(对比v5),保证出区时滤波器状态连续,
     *   避免冷启动效应。
     */
    c->ec_filt = FUZZY_EC_FILTER_ALPHA * c->ec_filt
               + (1.0f - FUZZY_EC_FILTER_ALPHA) * ec_raw;
    float ec   = c->ec_filt;

    /* ③ 模糊推理更新 Kp、Kd */
    fuzzy_adjust_gains(e, ec, &c->Kp, &c->Kd);

    /*
     * [v7新增] 积分项 — 条件累加 + Anti-windup
     *
     *   ① 仅 |e| < I_RANGE 时累加,大误差段交给 P 项主导
     *   ② 输出限幅期间不累加(后置在 slew_limit 之前判断)
     *   ③ 积分输出值本身限幅,防止单一方向极长时间持续误差时 I 失控
     */
    int integral_active = (abs_e < FUZZY_I_RANGE);
    if (integral_active) {
        c->integral += e;
        /* 积分项 PWM 贡献 = Ki × integral, 限幅在 ±I_OUT_MAX */
        float i_max_accum = FUZZY_I_OUT_MAX / FUZZY_KI;  /* 反推累加器上限 */
        if (c->integral >  i_max_accum) c->integral =  i_max_accum;
        if (c->integral < -i_max_accum) c->integral = -i_max_accum;
    }
    float u_i = FUZZY_KI * c->integral;

    /* ④ 完整 PID 计算(位置式) — 注意此处计算的是"目标输出 u_target",
     *    实际写入 c->output 的值还要经过 slew rate 限幅 */
    float u_target = c->Kp * e + c->Kd * ec + u_i;

    /*
     * [v3新增, v4改为渐进式] 摩擦前馈:
     *   远端 (|e| ≥ E_FF_FULL): 前馈满量程,顶开静摩擦
     *   近端 (|e| < E_FF_FULL): 前馈按 |e|/E_FF_FULL 线性衰减
     */
    {
        float ff_ratio = abs_e / FUZZY_E_FF_FULL;
        if (ff_ratio > 1.0f) ff_ratio = 1.0f;
        float ff = (float)FUZZY_FRICTION_FF * ff_ratio;
        if (e > 0.0f) u_target += ff;
        else          u_target -= ff;
    }

    /* ⑤ 输出限幅,防止超出电机驱动器允许范围 */
    int saturated = 0;
    if (u_target >  c->pwm_max) { u_target =  c->pwm_max; saturated = 1; }
    if (u_target < -c->pwm_max) { u_target = -c->pwm_max; saturated = 1; }

    /*
     * [v7新增] Anti-windup:
     *   若本周期输出已饱和限幅,且积分项与误差同号(继续累加会让饱和
     *   更严重),则撤回本周期的积分累加。
     *   这是反积分饱和的标准做法之一(back-calculation 的简化版)。
     */
    if (saturated && integral_active) {
        if ((c->integral > 0 && e > 0) || (c->integral < 0 && e < 0)) {
            c->integral -= e;   /* 撤回本周期累加 */
        }
    }

    /*
     * [v6新增] PWM 变化率限制 (Slew Rate Limit):
     *   |u(k) - u(k-1)| ≤ FUZZY_PWM_SLEW_MAX
     *   消除 PWM 突变,保护驱动器和机械传动。
     *   注意: 限幅作用在最终输出层,不影响内部 PD 与前馈的逻辑。
     */
    {
        float delta = u_target - c->output;
        if (delta >  FUZZY_PWM_SLEW_MAX) delta =  FUZZY_PWM_SLEW_MAX;
        if (delta < -FUZZY_PWM_SLEW_MAX) delta = -FUZZY_PWM_SLEW_MAX;
        c->output += delta;
    }

    /* ⑥ 保存本步误差供下周期计算 ec */
    c->error_last = e;
}

/**
 * @brief  初始化单个 Fuzzy_PD_Controller 实例
 *
 * 写入内容:
 *   - Kp = FUZZY_KP_BASE (2.0),  Kd = FUZZY_KD_BASE (0.3)
 *   - error_last = 0,  output = 0
 *   - pwm_max = FUZZY_PWM_MAX_OUT (8000)
 *   - target_position = 0,  actual_position = 0
 *   - motor_id 和 dir_pin 绑定到对应硬件通道
 *
 * @param c         控制器实例指针
 * @param motor_id  电机通道 ID (FUZZY_POS_PID_L1/L2/R1/R2)
 * @param dir_pin   方向控制 GPIO 引脚 (MOTOR_L1_DIR 等)
 */
static void pd_ctrl_init(Fuzzy_PD_Controller *c, uint8 motor_id, uint8 dir_pin)
{
    c->Kp              = FUZZY_KP_BASE;
    c->Kd              = FUZZY_KD_BASE;
    c->error_last      = 0.0f;
    c->ec_filt         = 0.0f;          /* [v2] 滤波器状态清零 */
    c->integral        = 0.0f;          /* [v7] 积分累加器清零 */
    c->output          = 0.0f;
    c->pwm_max         = FUZZY_PWM_MAX_OUT;
    c->target_position = 0;
    c->actual_position = 0;
    c->motor_id        = motor_id;
    c->dir_pin         = dir_pin;
    c->db_active       = 0;             /* [v6] 死区状态机初始为非锁定 */
    c->db_steady_count = 0;             /* [v6] */
}

/* =====================================================================
 *  公开接口实现
 * ===================================================================== */

/**
 * @brief  初始化全部 4 路模糊PD控制器
 *
 * 功能:
 *   对 pd_L1/pd_L2/pd_R1/pd_R2 四个实例调用 pd_ctrl_init()，
 *   将所有增益、误差历史、输出值归零，并绑定电机硬件引脚。
 *
 * 调用时机:
 *   系统上电初始化阶段，在 Motor_Init() 和 Encoder_Init() 之后
 *   调用一次，程序运行期间不需要重复调用。
 *
 * 示例:
 *   void System_Init(void)
 *   {
 *       Motor_Init();       // 先初始化电机方向引脚
 *       Encoder_Init();     // 再初始化编码器硬件
 *       MyPWM_Init();       // PWM 模块初始化
 *       Fuzzy_PID_Init();   // 最后初始化控制器
 *   }
 *
 * 注意:
 *   若在运动过程中误调此函数，所有目标位置清零，电机将立刻
 *   以 output=0 停止，但编码器硬件计数不受影响。
 */
void Fuzzy_PID_Init(void)
{
    pd_ctrl_init(&pd_L1, FUZZY_POS_PID_L1, MOTOR_L1_DIR);
    pd_ctrl_init(&pd_L2, FUZZY_POS_PID_L2, MOTOR_L2_DIR);
    pd_ctrl_init(&pd_R1, FUZZY_POS_PID_R1, MOTOR_R1_DIR);
    pd_ctrl_init(&pd_R2, FUZZY_POS_PID_R2, MOTOR_R2_DIR);
}

/**
 * @brief  设置指定电机的目标位置，并消除微分冲击
 *
 * 功能:
 *   将控制器的 target_position 更新为指定脉冲数，同时将
 *   error_last 预置为"当前误差"，使得第一个控制周期的
 *   ec = e(k) - error_last = 0，避免目标突变引发 Kd×ec 的 PWM 尖峰。
 *
 * 参数:
 *   @param id          电机通道
 *                        FUZZY_POS_PID_L1 (0) — 左前
 *                        FUZZY_POS_PID_L2 (1) — 左后
 *                        FUZZY_POS_PID_R1 (2) — 右前
 *                        FUZZY_POS_PID_R2 (3) — 右后
 *   @param target_pos  目标脉冲数 (相对于上次 Clear_Motor_Position 的零点)
 *                        正值 = 正转,  负值 = 反转
 *                        换算: 1圈 = PULSE_PER_REV = 4096 脉冲
 *
 * 无返回值。传入无效 id 时静默忽略。
 *
 * 示例:
 *   // 左前电机从当前零点正转 1 圈
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);
 *
 *   // 右前电机反转半圈
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, -2048);
 *
 *   // 四轮同步前进约 10 cm (假设每圈对应 200 mm，4096/200×10 ≈ 205 脉冲)
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1,  205);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L2,  205);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1,  205);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_R2,  205);
 *
 * 注意:
 *   ① 目标是绝对值（相对零点），不是增量；如需增量运动请先
 *      读取 Get_Motor_Position() 再加上偏移量。
 *   ② 调用后不立刻输出，需等待下一个 Fuzzy_PID_Calculate() 周期。
 */
void Fuzzy_PID_Set_Target(uint8 id, int32 target_pos)
{
    Fuzzy_PD_Controller *c = NULL;

    if      (id == FUZZY_POS_PID_L1) c = &pd_L1;
    else if (id == FUZZY_POS_PID_L2) c = &pd_L2;
    else if (id == FUZZY_POS_PID_R1) c = &pd_R1;
    else if (id == FUZZY_POS_PID_R2) c = &pd_R2;

    if (c == NULL) return;

    c->target_position = target_pos;

    /*
     * 微分冲击抑制 (Derivative Kick Prevention):
     *   若不做此预置，第一步 ec = e(k) - 0 = e(k)（可能高达数千脉冲），
     *   Kd × ec 会产生巨大的 PWM 突跳，损伤电机或造成失控。
     *   预置后第一步 ec = e(k) - e(k) = 0，等第二步才有正常的微分响应。
     */
    c->error_last = (float)(target_pos - c->actual_position);

    /*
     * [v7] 积分清零:
     *   设置新目标时,旧目标累积的 I 项已经无意义,清零防止
     *   "上一段任务残留的 I 把电机往错误方向推"。
     */
    c->integral = 0.0f;
}

/**
 * @brief  从外部更新指定电机的当前位置（可选接口）
 *
 * 功能:
 *   直接写入控制器的 actual_position，用于接入外部位置传感器
 *   （如绝对值编码器、光电开关等）替代内置编码器。
 *   通常情况下，Fuzzy_PID_Calculate() 会自动从 Ecoder_Total_Lx/Rx
 *   读取位置，无需调用此函数。
 *
 * 参数:
 *   @param id           电机通道 (FUZZY_POS_PID_L1/L2/R1/R2)
 *   @param current_pos  外部测量的当前位置 (脉冲数或等效单位)
 *
 * 注意:
 *   若与内置编码器混用（部分通道调用此函数，部分通道不调用），
 *   Fuzzy_PID_Calculate() 仍会覆盖 actual_position 为编码器值，
 *   本函数写入的值将被覆盖。需要完全替代编码器时，应在
 *   Fuzzy_PID_Calculate() 之前调用本函数，并注释掉Calculate内的
 *   encoder读取行。
 */
void Fuzzy_PID_Update_Position(uint8 id, int32 current_pos)
{
    if      (id == FUZZY_POS_PID_L1) pd_L1.actual_position = current_pos;
    else if (id == FUZZY_POS_PID_L2) pd_L2.actual_position = current_pos;
    else if (id == FUZZY_POS_PID_R1) pd_R1.actual_position = current_pos;
    else if (id == FUZZY_POS_PID_R2) pd_R2.actual_position = current_pos;
}

/**
 * @brief  模糊PD主计算函数，必须在定时中断中周期调用
 *
 * 功能:
 *   ① 从 Ecoder_Total_Lx/Rx 读取最新编码器累计值写入 actual_position
 *   ② 对选中通道调用 pd_step()，完成一次完整的模糊推理 + PD计算
 *   ③ 计算结果存入各控制器的 output 字段，供后续 Drive_Motor 使用
 *
 * 参数:
 *   @param id  指定计算通道
 *              FUZZY_POS_PID_L1 (0) — 仅计算左前
 *              FUZZY_POS_PID_L2 (1) — 仅计算左后
 *              FUZZY_POS_PID_R1 (2) — 仅计算右前
 *              FUZZY_POS_PID_R2 (3) — 仅计算右后
 *              0xFF               — 计算全部4路（推荐）
 *
 * 无返回值。计算结果通过 output 字段输出，用 Fuzzy_PID_Get_Output()
 * 或 Fuzzy_PID_Drive_Motor() 获取/使用。
 *
 * 内部 pd_step 执行流程 (每路相同):
 *   e  = target_position - actual_position  (脉冲数，可正可负)
 *   ec = e - error_last                     (本周期误差变化量)
 *   → fuzzy_adjust_gains(e, ec) → Kp, Kd   (查表整定)
 *   output = Kp×e + Kd×ec                  (位置式PD)
 *   output = clamp(output, -8000, +8000)    (限幅)
 *   error_last = e                          (保存供下周期使用)
 *
 * 示例 (10 ms 定时中断):
 *   void pit0_isr(void)
 *   {
 *       Encoder_Speed_PID_Update();   // 必须先更新编码器！
 *       Fuzzy_PID_Calculate(0xFF);    // 再计算PD
 *       Fuzzy_PID_Drive_All_Motor();  // 最后驱动电机
 *   }
 *
 * 注意:
 *   ① 必须在 Encoder_Speed_PID_Update() 之后调用，否则读到上周期旧值。
 *   ② 调用周期建议 5~20 ms；过短增加CPU负担，过长控制精度下降。
 *   ③ 不调用 Drive_Motor 时，output 仍正常更新，可用于调试读取。
 */
void Fuzzy_PID_Calculate(uint8 id)
{
    if (id == FUZZY_POS_PID_L1 || id == 0xFF) {
        pd_L1.actual_position = Ecoder_Total_L1;
        pd_step(&pd_L1);
    }
    if (id == FUZZY_POS_PID_L2 || id == 0xFF) {
        pd_L2.actual_position = Ecoder_Total_L2;
        pd_step(&pd_L2);
    }
    if (id == FUZZY_POS_PID_R1 || id == 0xFF) {
        pd_R1.actual_position = Ecoder_Total_R1;
        pd_step(&pd_R1);
    }
    if (id == FUZZY_POS_PID_R2 || id == 0xFF) {
        pd_R2.actual_position = Ecoder_Total_R2;
        pd_step(&pd_R2);
    }
}

/**
 * @brief  获取指定电机最新的 PD 输出值（不触发重新计算）
 *
 * 功能:
 *   返回上一次 Fuzzy_PID_Calculate() 执行后存入 output 的结果，
 *   封装为 Cascade_PID_Output 结构体。
 *
 * 参数:
 *   @param id  电机通道 (FUZZY_POS_PID_L1/L2/R1/R2)
 *
 * 返回值:
 *   Cascade_PID_Output 结构体:
 *     .speed_out     最终 PWM 输出值，范围 [-8000, +8000]
 *                    正值 → 正转，负值 → 反转，绝对值 = 占空比
 *     .position_out  本版本始终为 0.0f（保留字段，兼容旧接口）
 *
 * 示例:
 *   // 读取左前电机输出后手动驱动
 *   Cascade_PID_Output out = Fuzzy_PID_Get_Output(FUZZY_POS_PID_L1);
 *   int16 pwm = (int16)out.speed_out;
 *   Motor_Set(MOTOR_L1_DIR, PWM_CH1_L1, pwm);
 *
 *   // 调试打印
 *   printf("L1=%.0f  R1=%.0f\n",
 *          Fuzzy_PID_Get_Output(FUZZY_POS_PID_L1).speed_out,
 *          Fuzzy_PID_Get_Output(FUZZY_POS_PID_R1).speed_out);
 *
 * 注意:
 *   若在两次 Calculate() 之间多次调用，返回值不变（无副作用）。
 */
Cascade_PID_Output Fuzzy_PID_Get_Output(uint8 id)
{
    Cascade_PID_Output out = {0.0f, 0.0f};

    if      (id == FUZZY_POS_PID_L1) out.speed_out = pd_L1.output;
    else if (id == FUZZY_POS_PID_L2) out.speed_out = pd_L2.output;
    else if (id == FUZZY_POS_PID_R1) out.speed_out = pd_R1.output;
    else if (id == FUZZY_POS_PID_R2) out.speed_out = pd_R2.output;

    return out;
}

/**
 * @brief  将 PD 输出写入单个电机的方向引脚和 PWM 通道
 *
 * 功能:
 *   读取目标通道的 output 值，调用 Motor_Set() 完成:
 *     output > 0 → 方向引脚置高(正转) + PWM = output
 *     output < 0 → 方向引脚置低(反转) + PWM = |output|
 *     output = 0 → PWM = 0，电机停止
 *
 * 通道与硬件映射:
 *   L1: MOTOR_L1_DIR + PWM_CH1_L1 (C9 + C8)
 *   L2: MOTOR_L2_DIR + PWM_CH3_L2 (D2 + D3)
 *   R1: MOTOR_R1_DIR + PWM_CH2_R1 (C7 + C6)
 *   R2: MOTOR_R2_DIR + PWM_CH4_R2 (C10 + C11)
 *
 * 参数:
 *   @param id  电机通道 (FUZZY_POS_PID_L1/L2/R1/R2)
 *
 * 示例:
 *   // 在中断中依次驱动左侧两轮
 *   Fuzzy_PID_Calculate(FUZZY_POS_PID_L1);
 *   Fuzzy_PID_Calculate(FUZZY_POS_PID_L2);
 *   Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L1);
 *   Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L2);
 *
 * 注意:
 *   必须在本周期 Fuzzy_PID_Calculate() 之后调用，否则驱动的是
 *   上一周期的 output。
 */
void Fuzzy_PID_Drive_Motor(uint8 id)
{
    int32 pwm = (int32)Fuzzy_PID_Get_Output(id).speed_out;

    if      (id == FUZZY_POS_PID_L1) Motor_Set(MOTOR_L1_DIR, PWM_CH1_L1, pwm);
    else if (id == FUZZY_POS_PID_L2) Motor_Set(MOTOR_L2_DIR, PWM_CH3_L2, pwm);
    else if (id == FUZZY_POS_PID_R1) Motor_Set(MOTOR_R1_DIR, PWM_CH2_R1, pwm);
    else if (id == FUZZY_POS_PID_R2) Motor_Set(MOTOR_R2_DIR, PWM_CH4_R2, pwm);
}

/**
 * @brief  驱动全部 4 路电机（Fuzzy_PID_Drive_Motor 的批量版本）
 *
 * 功能:
 *   依次调用 4 次 Fuzzy_PID_Drive_Motor()，一次驱动所有电机。
 *   等效于:
 *     Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L1);
 *     Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L2);
 *     Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_R1);
 *     Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_R2);
 *
 * 示例 (标准中断结构):
 *   void pit0_isr(void)
 *   {
 *       Encoder_Speed_PID_Update();
 *       Fuzzy_PID_Calculate(0xFF);
 *       Fuzzy_PID_Drive_All_Motor();   // 一行替代4次Drive_Motor调用
 *   }
 */
void Fuzzy_PID_Drive_All_Motor(void)
{
    Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L1);
    Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_L2);
    Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_R1);
    Fuzzy_PID_Drive_Motor(FUZZY_POS_PID_R2);
}

/**
 * @brief  获取指定电机编码器累计脉冲数（即当前绝对位置）
 *
 * 功能:
 *   直接返回 Ecoder_Total_Lx/Rx 变量的当前值，该值由
 *   Encoder_Speed_PID_Update() 每周期累加，反映自上次
 *   Clear_Motor_Position() 后的净位移。
 *
 * 参数:
 *   @param id  电机通道 (FUZZY_POS_PID_L1/L2/R1/R2)
 *
 * 返回值:
 *   int32 累计脉冲数
 *     正值 = 正转方向累计位移
 *     负值 = 反转方向累计位移
 *     4096 = 转动 1 整圈
 *   传入无效 id 返回 0。
 *
 * 示例:
 *   // 读取并换算为圈数
 *   int32 pulses = Get_Motor_Position(FUZZY_POS_PID_L1);
 *   float revs   = pulses / 4096.0f;
 *   printf("L1 已转 %d 脉冲 (%.3f 圈)\n", pulses, revs);
 *
 *   // 到位检测 (±15脉冲死区，约±1.3°)
 *   if (abs(Get_Motor_Position(FUZZY_POS_PID_R1) - target) <= 15)
 *       printf("右前电机到位!\n");
 */
int32 Get_Motor_Position(uint8 id)
{
    if      (id == FUZZY_POS_PID_L1) return Ecoder_Total_L1;
    else if (id == FUZZY_POS_PID_L2) return Ecoder_Total_L2;
    else if (id == FUZZY_POS_PID_R1) return Ecoder_Total_R1;
    else if (id == FUZZY_POS_PID_R2) return Ecoder_Total_R2;
    return 0;
}

/**
 * @brief  清零指定电机的位置计数器及 PD 内部状态
 *
 * 功能:
 *   将 Ecoder_Total_Lx/Rx 清零（建立新参考零点），同时清除
 *   控制器的 error_last 和 output，防止残留状态在下次运动中
 *   产生误差或 PWM 突跳。
 *
 * 参数:
 *   @param id  电机通道
 *              FUZZY_POS_PID_L1/L2/R1/R2 — 清零单路
 *              0xFF                       — 清零全部4路
 *
 * 清零内容对比:
 *   ┌────────────────────────────┬──────┬───────────────────────────┐
 *   │ 变量                       │ 清零 │ 说明                       │
 *   ├────────────────────────────┼──────┼───────────────────────────┤
 *   │ Ecoder_Total_Lx            │  ✓  │ 编码器累计脉冲，建立新零点 │
 *   │ pd_Lx.error_last           │  ✓  │ 消除历史误差，防止首步冲击 │
 *   │ pd_Lx.output               │  ✓  │ 清零上次输出，电机安全停止 │
 *   │ pd_Lx.target_position      │  ✗  │ 目标不变，需手动重新设置   │
 *   │ 编码器硬件计数寄存器        │  ✗  │ 由 Encoder.c 管理          │
 *   └────────────────────────────┴──────┴───────────────────────────┘
 *
 * 示例:
 *   // 场景1: 在当前位置建立新零点，再运动1圈
 *   Motor_Stop();
 *   Clear_Motor_Position(FUZZY_POS_PID_L1);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);
 *
 *   // 场景2: 全部电机归零后同步运动
 *   Clear_Motor_Position(0xFF);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 2048);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L2, 2048);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, 2048);
 *   Fuzzy_PID_Set_Target(FUZZY_POS_PID_R2, 2048);
 *
 * 注意:
 *   清零后 target_position 不变，若不重新 Set_Target，控制器将
 *   以旧目标值（相对新零点）驱动电机，可能造成意外运动。
 *   建议清零后立即调用 Set_Target() 设置新目标。
 */
void Clear_Motor_Position(uint8 id)
{
    if (id == FUZZY_POS_PID_L1 || id == 0xFF) {
        Ecoder_Total_L1      = 0;
        pd_L1.error_last     = 0.0f;
        pd_L1.ec_filt        = 0.0f;    /* [v2] */
        pd_L1.integral       = 0.0f;    /* [v7] */
        pd_L1.output         = 0.0f;
        pd_L1.db_active      = 0;       /* [v6] */
        pd_L1.db_steady_count = 0;      /* [v6] */
    }
    if (id == FUZZY_POS_PID_L2 || id == 0xFF) {
        Ecoder_Total_L2      = 0;
        pd_L2.error_last     = 0.0f;
        pd_L2.ec_filt        = 0.0f;    /* [v2] */
        pd_L2.integral       = 0.0f;    /* [v7] */
        pd_L2.output         = 0.0f;
        pd_L2.db_active      = 0;       /* [v6] */
        pd_L2.db_steady_count = 0;      /* [v6] */
    }
    if (id == FUZZY_POS_PID_R1 || id == 0xFF) {
        Ecoder_Total_R1      = 0;
        pd_R1.error_last     = 0.0f;
        pd_R1.ec_filt        = 0.0f;    /* [v2] */
        pd_R1.integral       = 0.0f;    /* [v7] */
        pd_R1.output         = 0.0f;
        pd_R1.db_active      = 0;       /* [v6] */
        pd_R1.db_steady_count = 0;      /* [v6] */
    }
    if (id == FUZZY_POS_PID_R2 || id == 0xFF) {
        Ecoder_Total_R2      = 0;
        pd_R2.error_last     = 0.0f;
        pd_R2.ec_filt        = 0.0f;    /* [v2] */
        pd_R2.integral       = 0.0f;    /* [v7] */
        pd_R2.output         = 0.0f;
        pd_R2.db_active      = 0;       /* [v6] */
        pd_R2.db_steady_count = 0;      /* [v6] */
    }
}
