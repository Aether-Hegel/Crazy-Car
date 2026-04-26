#include "Speed_PID.h"
#include "UART.h"

extern uint8 fifo4_rx_buffer[UART4_RX_BUF_SIZE]; // FIFO输出读出缓冲区
extern uint32 fifo4_rx_index;

/**
 * @brief 速度PID参数
 * @details 用于PID调式时使用的临时变量
 */
float Kp_temp = 0.0f;
float Ki_temp = 0.0f;
float Kd_temp = 0.0f;
float target_temp = 0.0f;

Speed_PID Speed_PID_L1 = {.Kp = 2.0f, .Ki = 0.2f, .Kd = 2.0f, .target_speed = 0.0f, .Pwm_Max_Out = 8000, .integral_sum = 0.0f, .integral_limit = 10000.0f, .output_dead_zone = 300.0f};
Speed_PID Speed_PID_R1 = {.Kp = 1.8f, .Ki = 0.5f, .Kd = 0.6f, .target_speed = 0.0f, .Pwm_Max_Out = 8000, .integral_sum = 0.0f, .integral_limit = 10000.0f,  .output_dead_zone = 300.0f};
Speed_PID Speed_PID_L2 = {.Kp = 2.5f, .Ki = 0.4f, .Kd = 1.0f, .target_speed = 0.0f, .Pwm_Max_Out = 8000, .integral_sum = 0.0f, .integral_limit = 10000.0f,  .output_dead_zone = 300.0f};
Speed_PID Speed_PID_R2 = {.Kp = 1.8f, .Ki = 0.4f, .Kd = 1.0f, .target_speed = 0.0f, .Pwm_Max_Out = 8000, .integral_sum = 0.0f, .integral_limit = 10000.0f,  .output_dead_zone = 300.0f};

/*********************************************************************************************************************
 * 函数名称：PID_parameter_calculate
 * 函数功能：从串口接收的 ASCII 数据中解析 PID 参数，并将解析结果存储到全局变量中
 *
 * 参数说明：
 *   无（函数使用全局变量 fifo4_rx_buffer 和 fifo4_rx_index）
 *
 * 返回值：
 *   无（解析结果存储到全局变量 Kp_temp、Ki_temp、Kd_temp）
 *
 * 数据格式：
 *   输入格式：ASCII 字符串，格式为 "Kp,Ki,Kd"
 *   示例：
 *     - "25,30,15"      → Kp=25, Ki=30, Kd=15
 *     - "2.5,3.0,1.5"   → Kp=2.5, Ki=3.0, Kd=1.5
 *     - "25.5,30.8,15.25" → Kp=25.5, Ki=30.8, Kd=15.25
 *   分隔符：逗号（','）用于分隔三个参数
 *   小数点：点（'.'）用于表示小数部分
 *   结束符：回车（'\r'）或换行（'\n'）表示数据结束
 *
 * 算法逻辑：
 *
 *   1. 数据来源：
 *      - fifo4_rx_buffer[]：存储从串口接收到的 ASCII 字符
 *      - fifo4_rx_index：当前缓冲区中有效数据的长度
 *
 *   2. ASCII 到数值转换算法：
 *      对于每个参数（Kp、Ki、Kd），执行以下步骤：
 *
 *      a) 整数部分解析：
 *         从左到右遍历字符，遇到数字字符时进行累加
 *         公式：temp_value = temp_value * 10 + (字符 - '0')
 *         例如："25" → 0*10+2=2 → 2*10+5=25
 *
 *      b) 小数部分解析：
 *         遇到小数点后，后续数字作为小数部分处理
 *         公式：decimal_part = decimal_part * 10 + (字符 - '0')
 *              decimal_divisor = decimal_divisor * 10
 *         例如："5" → 0*10+5=5, decimal_divisor=10
 *
 *      c) 合并整数和小数部分：
 *         公式：最终值 = 整数部分 + 小数部分 / decimal_divisor
 *         例如：temp_value=25, decimal_part=5, decimal_divisor=10
 *              最终值 = 25 + 5/10 = 25.5
 *
 *   3. 三个参数的解析顺序：
 *
 *      a) 比例系数 Kp（Proportional Coefficient）：
 *         - 从缓冲区开头开始解析，直到遇到第一个逗号
 *         - Kp 用于控制误差的响应速度，值越大响应越快但容易超调
 *
 *      b) 积分系数 Ki（Integral Coefficient）：
 *         - 从第一个逗号之后开始解析，直到遇到第二个逗号
 *         - Ki 用于消除稳态误差，值越大消除误差越快但可能引起振荡
 *
 *      c) 微分系数 Kd（Derivative Coefficient）：
 *         - 从第二个逗号之后开始解析，直到遇到换行符或缓冲区结束
 *         - Kd 用于抑制超调，改善系统稳定性，值越大抗干扰能力越强
 *
 *   4. 变量说明：
 *      - i, j：循环索引变量，用于遍历缓冲区
 *      - temp_value：存储整数部分累加结果
 *      - decimal_part：存储小数部分累加结果
 *      - has_decimal：标志位，0=整数部分，1=小数部分
 *      - decimal_divisor：小数部分的除数，用于定位小数点位置（10, 100, 1000...）
 *
 *   5. 使用示例：
 *
 *      输入数据："2.5,3.0,1.5"
 *      fifo4_rx_buffer = {'2','.','5',',','3','.','0',',','1','.','5'}
 *      fifo4_rx_index = 11
 *
 *      解析过程：
 *        Kp:  '2' → temp_value=2
 *             '.' → has_decimal=1
 *             '5' → decimal_part=5, decimal_divisor=10
 *             Kp_temp = 2 + 5/10 = 2.5
 *
 *        Ki:  '3' → temp_value=3
 *             '.' → has_decimal=1
 *             '0' → decimal_part=0, decimal_divisor=10
 *             Ki_temp = 3 + 0/10 = 3.0
 *
 *        Kd:  '1' → temp_value=1
 *             '.' → has_decimal=1
 *             '5' → decimal_part=5, decimal_divisor=10
 *             Kd_temp = 1 + 5/10 = 1.5
 *
 *   6. 注意事项：
 *      - 调用此函数前，需确保 fifo4_rx_buffer 中已包含有效数据
 *      - 数据格式必须严格遵循 "Kp,Ki,Kd" 格式，否则解析可能出错
 *      - 函数不检查数据有效性，非法字符将被忽略
 *      - 解析结果存储在全局变量 Kp_temp、Ki_temp、Kd_temp 中
 *
 * 使用示例：
 *   在主循环中调用：
 *   ```c
 *   while(1) {
 *       uart_echo_data();        // 从 FIFO 读取数据到 fifo4_rx_buffer
 *       PID_parameter_calculate(); // 解析 PID 参数
 *       Speed_PID_Init();        // 应用解析后的参数
 *   }
 *   ```
 *
 * 修改记录：
 *   日期      作者      备注
 *   2025-xx-xx xxx       创建函数，实现 ASCII 到 PID 参数的转换
 *********************************************************************************************************************/
void PID_parameter_calculate()
{
    // 方法：将ASCII数字字符转换为十进制数值
    // 发送格式：Kp,Ki,Kd (例如：25,30,15)
    // fifo4_rx_buffer 内容：{'2','5',',','3','0',',','1','5'}

    uint32 i = 0;                 // 全局索引，用于记录当前解析位置
    uint32 j = 0;                 // 局部索引，用于 Kp 解析的循环
    float temp_value = 0;         // 整数部分累加器
    float decimal_part = 0;       // 小数部分累加器
    uint8 has_decimal = 0;        // 小数点标志：0=整数部分，1=小数部分
    float decimal_divisor = 1.0f; // 小数部分除数，用于计算小数位置

    // ==========================================================================================================
    // 步骤1：解析比例系数 Kp（Proportional Coefficient）
    // 功能：控制误差的响应速度，值越大响应越快但容易超调
    // ==========================================================================================================
    temp_value = 0;         // 重置整数部分累加器
    decimal_part = 0;       // 重置小数部分累加器
    has_decimal = 0;        // 重置小数点标志
    decimal_divisor = 1.0f; // 重置小数除数

    // 从缓冲区开头遍历，直到遇到逗号或缓冲区结束
    for (j = 0; j < fifo4_rx_index && fifo4_rx_buffer[j] != ','; j++)
    {
        if (fifo4_rx_buffer[j] == '.') // 检测到小数点
        {
            has_decimal = 1; // 标记进入小数部分
            continue;        // 跳过小数点字符
        }

        if (fifo4_rx_buffer[j] >= '0' && fifo4_rx_buffer[j] <= '9') // 检测到数字字符
        {
            if (has_decimal) // 如果已进入小数部分
            {
                // 小数部分累加：decimal_part = decimal_part * 10 + 当前数字
                decimal_part = decimal_part * 10 + (fifo4_rx_buffer[j] - '0');
                decimal_divisor *= 10.0f; // 除数乘以10，定位小数点位置
            }
            else // 仍在整数部分
            {
                // 整数部分累加：temp_value = temp_value * 10 + 当前数字
                temp_value = temp_value * 10 + (fifo4_rx_buffer[j] - '0');
            }
        }
    }
    // 合并整数和小数部分：Kp = 整数部分 + 小数部分 / 除数
    Kp_temp = temp_value + decimal_part / decimal_divisor;
    i = j + 1; // 跳过逗号，移动到 Ki 的起始位置

    // ==========================================================================================================
    // 步骤2：解析积分系数 Ki（Integral Coefficient）
    // 功能：消除稳态误差，值越大消除误差越快但可能引起振荡
    // ==========================================================================================================
    temp_value = 0;         // 重置整数部分累加器
    decimal_part = 0;       // 重置小数部分累加器
    has_decimal = 0;        // 重置小数点标志
    decimal_divisor = 1.0f; // 重置小数除数

    // 从 Ki 起始位置遍历，直到遇到逗号或缓冲区结束
    for (; i < fifo4_rx_index && fifo4_rx_buffer[i] != ','; i++)
    {
        if (fifo4_rx_buffer[i] == '.') // 检测到小数点
        {
            has_decimal = 1; // 标记进入小数部分
            continue;        // 跳过小数点字符
        }

        if (fifo4_rx_buffer[i] >= '0' && fifo4_rx_buffer[i] <= '9') // 检测到数字字符
        {
            if (has_decimal) // 如果已进入小数部分
            {
                // 小数部分累加
                decimal_part = decimal_part * 10 + (fifo4_rx_buffer[i] - '0');
                decimal_divisor *= 10.0f; // 除数乘以10
            }
            else // 仍在整数部分
            {
                // 整数部分累加
                temp_value = temp_value * 10 + (fifo4_rx_buffer[i] - '0');
            }
        }
    }
    // 合并整数和小数部分：Ki = 整数部分 + 小数部分 / 除数
    Ki_temp = temp_value + decimal_part / decimal_divisor;
    i++; // 跳过逗号，移动到 Kd 的起始位置

    // ==========================================================================================================
    // 步骤3：解析微分系数 Kd（Derivative Coefficient）
    // 功能：抑制超调，改善系统稳定性，值越大抗干扰能力越强
    // ==========================================================================================================
    temp_value = 0;         // 重置整数部分累加器
    decimal_part = 0;       // 重置小数部分累加器
    has_decimal = 0;        // 重置小数点标志
    decimal_divisor = 1.0f; // 重置小数除数

    // 从 Kd 起始位置遍历，直到遇到换行符或缓冲区结束
    for (; i < fifo4_rx_index && fifo4_rx_buffer[i] != ','; i++)
    {
        if (fifo4_rx_buffer[i] == '.') // 检测到小数点
        {
            has_decimal = 1; // 标记进入小数部分
            continue;        // 跳过小数点字符
        }

        if (fifo4_rx_buffer[i] >= '0' && fifo4_rx_buffer[i] <= '9') // 检测到数字字符
        {
            if (has_decimal) // 如果已进入小数部分
            {
                // 小数部分累加
                decimal_part = decimal_part * 10 + (fifo4_rx_buffer[i] - '0');
                decimal_divisor *= 10.0f; // 除数乘以10
            }
            else // 仍在整数部分
            {
                // 整数部分累加
                temp_value = temp_value * 10 + (fifo4_rx_buffer[i] - '0'); // 转换成十进制数值
            }
        }
    }
    // 合并整数和小数部分：Kd = 整数部分 + 小数部分 / 除数
    Kd_temp = temp_value + decimal_part / decimal_divisor;
    i++;

    // ==========================================================================================================
    // 步骤4：解析目标速度值
    // ==========================================================================================================
    temp_value = 0;         // 重置整数部分累加器
    decimal_part = 0;       // 重置小数部分累加器
    has_decimal = 0;        // 重置小数点标志
    decimal_divisor = 1.0f; // 重置小数除数

    // 从 target 起始位置遍历，直到遇到换行符或缓冲区结束
    for (; i < fifo4_rx_index; i++)
    {
        if (fifo4_rx_buffer[i] == '.') // 检测到小数点
        {
            has_decimal = 1; // 标记进入小数部分
            continue;        // 跳过小数点字符
        }

        if (fifo4_rx_buffer[i] >= '0' && fifo4_rx_buffer[i] <= '9') // 检测到数字字符
        {
            if (has_decimal) // 如果已进入小数部分
            {
                // 小数部分累加
                decimal_part = decimal_part * 10 + (fifo4_rx_buffer[i] - '0');
                decimal_divisor *= 10.0f; // 除数乘以10
            }
            else // 仍在整数部分
            {
                // 整数部分累加
                temp_value = temp_value * 10 + (fifo4_rx_buffer[i] - '0'); // 转换成十进制数值
            }
        }
        else if (fifo4_rx_buffer[i] == '\r' || fifo4_rx_buffer[i] == '\n')
        {
            break; // 遇到回车或换行符，结束解析
        }
    }
    // 合并整数和小数部分：Kd = 整数部分 + 小数部分 / 除数
    target_temp = temp_value + decimal_part / decimal_divisor;
}

void Set_Speed_PID(Speed_PID *pid, float Kp, float Ki, float Kd, float target, float PWM_MAX_OUt)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target_speed = target;
    pid->Pwm_Max_Out = PWM_MAX_OUt;
}


void Speed_PID_Init() // 初始化PID参数
{
    PID_parameter_calculate();
    // Set_Speed_PID(&Speed_PID_L1, Kp_temp, Ki_temp, Kd_temp, target_temp, 10000);
    // Set_Speed_PID(&Speed_PID_R1, Kp_temp, Ki_temp, Kd_temp, 100, 10000);
}

void Speed_PID_Calculate(Speed_PID *pid, float current_speed)
{
    // 更新实际速度
    pid->Actual_speed = current_speed;
    // 更新状态变量
    pid->error_prev2 = pid->error_last;   // 将e(k-1)移动到e(k-2)
    pid->error_last = pid->error_current; // 将当前滤波误差保存为下次循环的e(k-1)

    // 计算当前误差 e(k)
    float error_current = pid->target_speed - pid->Actual_speed;
    // 滤波误差（降低滤波强度以减少相位滞后，0.5/0.5 平衡噪声抑制与响应速度）
    float filtered_error = error_current * 0.5f + pid->error_last * 0.5f;
    // 保存滤波后的误差e(k)
    pid->error_current = filtered_error;

    // 增量式PID计算
    // u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
    float increment = pid->Kp * (filtered_error - pid->error_last) + pid->Ki * pid->error_current  + pid->Kd * (filtered_error - 2.0f * pid->error_last + pid->error_prev2);

    // 积分限幅：限制单步增量中的积分贡献，防止累积过大
    float i_contribution = pid->Ki * pid->error_current;
    if (i_contribution >  pid->integral_limit) i_contribution =  pid->integral_limit;
    if (i_contribution < -pid->integral_limit) i_contribution = -pid->integral_limit;
    increment = pid->Kp * (filtered_error - pid->error_last) + i_contribution + pid->Kd * (filtered_error - 2.0f * pid->error_last + pid->error_prev2);

    // 计算当前输出
    float output = pid->output + increment;

    // 输出限幅
    if (output > pid->Pwm_Max_Out)
        output = pid->Pwm_Max_Out;
    else if (output < -pid->Pwm_Max_Out)
        output = -pid->Pwm_Max_Out;

    // 输出死区：绝对值低于阈值时强制清零，同时清除误差状态防止残留
    if (output > -pid->output_dead_zone && output < pid->output_dead_zone)
    {
        output = 0.0f;
        pid->error_current = 0.0f;
        pid->error_last    = 0.0f;
        pid->error_prev2   = 0.0f;
    }

    pid->output = output;
}

