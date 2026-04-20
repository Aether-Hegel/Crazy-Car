#include "Encoder.h"
#include "zf_driver_encoder.h"

int16 Ecoder_count_L1;
int16 Ecoder_count_L2;
int16 Ecoder_count_R1;
int16 Ecoder_count_R2;

// //-------------------------------------------------------------------------------------------------------------------
// // 代码片段1：正交编码器初始化函数
// // 功能：使用硬件Timer的编码器模式初始化正交编码器
// // 参数：encoder - 编码器实例结构体指针
// //       encoder_id - 编码器ID（QTIMER1_ENCODER1等）
// //       ch1_pin - A相引脚（encoder_channel1_enum枚举）
// //       ch2_pin - B相引脚（encoder_channel2_enum枚举）
// // 返回：void
// // 说明：调用底层encoder_quad_init()函数，硬件自动处理A/B相信号和相位解码
// //-------------------------------------------------------------------------------------------------------------------
// void encoder_init_quad(encoder_instance_t *encoder, encoder_index_enum encoder_id,
//                        encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
// {
//     if(encoder == NULL)
//     {
//         return;
//     }

//     // 保存编码器配置信息
//     encoder->encoder_id = encoder_id;
//     encoder->ch1_pin = ch1_pin;
//     encoder->ch2_pin = ch2_pin;

//     // 调用底层驱动函数初始化正交编码器模式
//     // 硬件Timer将自动：
//     // - 监测A相和B相信号的上升沿和下降沿
//     // - 进行相位解码判断旋转方向
//     // - 根据方向自动增减计数值
//     encoder_quad_init(encoder_id, ch1_pin, ch2_pin);

//     // 初始化软件计数变量
//     encoder->count = 0;
//     encoder->last_hw_count = encoder_get_count(encoder_id);
// }

// //-------------------------------------------------------------------------------------------------------------------
// // 代码片段2：带方向编码器初始化函数
// // 功能：使用硬件Timer的编码器模式初始化带方向输出的编码器
// // 参数：encoder - 编码器实例结构体指针
// //       encoder_id - 编码器ID（QTIMER1_ENCODER1等）
// //       ch1_pin - 脉冲输入引脚（encoder_channel1_enum枚举）
// //       ch2_pin - 方向控制引脚（encoder_channel2_enum枚举）
// // 返回：void
// // 说明：ch1_pin作为脉冲输入计数，ch2_pin作为计数方向控制引脚
// //-------------------------------------------------------------------------------------------------------------------
// void encoder_init_dir(encoder_instance_t *encoder, encoder_index_enum encoder_id,
//                       encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
// {
//     if(encoder == NULL)
//     {
//         return;
//     }

//     // 保存编码器配置信息
//     encoder->encoder_id = encoder_id;
//     encoder->ch1_pin = ch1_pin;
//     encoder->ch2_pin = ch2_pin;

//     // 调用底层驱动函数初始化带方向编码器模式
//     // 硬件Timer将自动：
//     // - 监测ch1_pin的脉冲边沿
//     // - 根据ch2_pin的电平决定计数方向
//     encoder_dir_init(encoder_id, ch1_pin, ch2_pin);

//     // 初始化软件计数变量
//     encoder->count = 0;
//     encoder->last_hw_count = encoder_get_count(encoder_id);
// }

// //-------------------------------------------------------------------------------------------------------------------
// // 代码片段3：更新编码器计数值函数
// // 功能：读取硬件Timer计数并更新软件累计值（防止溢出）
// // 参数：encoder - 编码器实例结构体指针
// // 返回：void
// // 说明：硬件计数器为16位（int16），可能溢出。此函数检测溢出并累计到32位变量中
// //-------------------------------------------------------------------------------------------------------------------
// void encoder_update_count(encoder_instance_t *encoder)
// {
//     int16 current_hw_count;
//     int16 delta;

//     if(encoder == NULL)
//     {
//         return;
//     }

//     // 读取当前硬件计数值（硬件Timer自动处理相位解码和计数方向）
//     current_hw_count = encoder_get_count(encoder->encoder_id);

//     // 计算差值，处理溢出情况
//     delta = current_hw_count - encoder->last_hw_count;

//     // 累加到软件计数值（int32，不会溢出）
//     encoder->count += delta;

//     // 更新上一次硬件计数值
//     encoder->last_hw_count = current_hw_count;
// }

// //-------------------------------------------------------------------------------------------------------------------
// // 代码片段4：获取总计数值并清零硬件计数函数
// // 功能：获取编码器的软件累计值，并清零硬件计数器以避免溢出
// // 参数：encoder - 编码器实例结构体指针
// // 返回：int32 - 编码器的总计数值
// // 说明：定期调用此函数可清空硬件计数器，防止长期运行导致溢出
// //-------------------------------------------------------------------------------------------------------------------
// int32 encoder_get_total_count(encoder_instance_t *encoder)
// {
//     int16 current_hw_count;
//     int16 delta;
//     int32 total_count;

//     if(encoder == NULL)
//     {
//         return 0;
//     }

//     // 读取当前硬件计数值
//     current_hw_count = encoder_get_count(encoder->encoder_id);

//     // 计算差值
//     delta = current_hw_count - encoder->last_hw_count;

//     // 累加到软件计数值
//     encoder->count += delta;

//     // 保存总计数值
//     total_count = encoder->count;

//     // 清零硬件计数器
//     encoder_clear_count(encoder->encoder_id);

//     // 重置硬件计数值记录
//     encoder->last_hw_count = 0;

//     // 清零软件累计值（已保存到total_count）
//     encoder->count = 0;

//     return total_count;
// }

// //-------------------------------------------------------------------------------------------------------------------
// // 辅助函数：清零编码器总计数
// //-------------------------------------------------------------------------------------------------------------------
// void encoder_clear_total_count(encoder_instance_t *encoder)
// {
//     if(encoder == NULL)
//     {
//         return;
//     }

//     // 清零硬件计数
//     encoder_clear_count(encoder->encoder_id);

//     // 清零软件累计值
//     encoder->count = 0; 
//     encoder->last_hw_count = 0;
// }

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
    Ecoder_count_L1 = encoder_get_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER1);  
    Ecoder_count_L2 = encoder_get_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER1_ENCODER2);
    Ecoder_count_R1 = encoder_get_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER2_ENCODER1);
    Ecoder_count_R2 = encoder_get_count(QTIMER2_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER2);
}
