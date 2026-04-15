#ifndef ENCODER_H
#define ENCODER_H

#include "zf_common_headfile.h"
#include "zf_driver_encoder.h"

// //-------------------------------------------------------------------------------------------------------------------
// // 编码器实例结构体定义（基于硬件Timer）
// //-------------------------------------------------------------------------------------------------------------------
// typedef struct
// {
//     encoder_index_enum encoder_id;       // 编码器ID（QTIMER1_ENCODER1等）
//     encoder_channel1_enum ch1_pin;       // A相引脚
//     encoder_channel2_enum ch2_pin;       // B相引脚
//     int32  count;                        // 软件计数值（用于累加）
//     int16  last_hw_count;                // 上一次硬件计数值
// } encoder_instance_t;

// //-------------------------------------------------------------------------------------------------------------------
// // 函数声明
// //-------------------------------------------------------------------------------------------------------------------
// void encoder_init_quad(encoder_instance_t *encoder, encoder_index_enum encoder_id, 
//                        encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin);
// void encoder_init_dir(encoder_instance_t *encoder, encoder_index_enum encoder_id, 
//                       encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin);
// int32 encoder_get_total_count(encoder_instance_t *encoder);
// void encoder_update_count(encoder_instance_t *encoder);
// void encoder_clear_total_count(encoder_instance_t *encoder);

void Encoder_Init(void);
void Encoder_Speed_PID_Update(void);

#endif // ENCODER_H