#ifndef __UART_H__
#define __UART_H__

#include "zf_common_typedef.h"
#include "stdio.h"
#include "string.h"

#define UART2_RX_BUF_SIZE 128

// 串口接收中断处理函数
void uart2_rx_interrupt_handler(void);

// 获取接收缓冲区数据
uint16 uart_rx_get_data(uint8 *buffer, uint16 len);

// 清空接收缓冲区
void uart_rx_clear_buffer(void);

// UART2初始化函数
void MyUART_2_Init(void);

// 将接收到的数据立即回传
void uart_echo_data(void);

#endif // __UART_H__
