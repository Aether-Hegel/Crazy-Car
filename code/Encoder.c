#include "Encoder.h"
#include "zf_driver_encoder.h"

int32 Ecoder_Total_L1 = 0;
int32 Ecoder_Total_L2 = 0;
int32 Ecoder_Total_R1 = 0;
int32 Ecoder_Total_R2 = 0;

int16 Encoder_Current_L1 = 0;
int16 Encoder_Current_L2 = 0;
int16 Encoder_Current_R1 = 0;
int16 Encoder_Current_R2 = 0;

int16 Encoder_Last_L1 = 0;
int16 Encoder_Last_L2 = 0;
int16 Encoder_Last_R1 = 0;
int16 Encoder_Last_R2 = 0;

int16 Ecoder_count_L1 = 0;
int16 Ecoder_count_L2 = 0;
int16 Ecoder_count_R1 = 0;
int16 Ecoder_count_R2 = 0;

void Encoder_Init(void)
{
    encoder_quad_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
    encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
    encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);
}

/**
 * @brief 编码器更新函数
 * @details 在周期中断函数中定期调用此函数来更新编码器计数值，防止硬件计数器溢出
 * @return void
 */
void Encoder_Speed_PID_Update(void)
{
    Encoder_Current_L1 = encoder_get_count(QTIMER1_ENCODER2);
    Ecoder_Total_L1 += Encoder_Current_L1;
    encoder_clear_count(QTIMER1_ENCODER2);

    Encoder_Current_L2 = encoder_get_count(QTIMER2_ENCODER1);
    Ecoder_Total_L2 += Encoder_Current_L2;
    encoder_clear_count(QTIMER2_ENCODER1);

    Encoder_Current_R1 = encoder_get_count(QTIMER1_ENCODER1);
    Ecoder_Total_R1 += Encoder_Current_R1;
    encoder_clear_count(QTIMER1_ENCODER1);
    
    Encoder_Current_R2 = encoder_get_count(QTIMER2_ENCODER2);
    Ecoder_Total_R2 += Encoder_Current_R2;
    encoder_clear_count(QTIMER2_ENCODER2);
}
