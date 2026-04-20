#include "Timer.h"
#include "Speed_PID.h"
#include "Encoder.h"

extern int16 Ecoder_count_L1;
extern int16 Ecoder_count_L2;
extern int16 Ecoder_count_R1;
extern int16 Ecoder_count_R2;

extern Speed_PID Speed_PID_L1;
extern Speed_PID Speed_PID_R1;

void Timer2_Init(void)
{
    pit_ms_init(PIT_CH0, 100);
    interrupt_enable(PIT_IRQn);
    interrupt_set_priority(PIT_IRQn, 2);
}

/**
 * @brief Timer2中断处理函数(100ms周期)
 * @details 在此函数中调用编码器计数值以实现周期性的PID控制和编码器数据更新
 * @return void
 */
void Timer2_IRQHandler(void)
{



    // Speed_PID_Calculate(&Speed_PID_R1, Ecoder_count_R1); // PID计算
}
