#include "UART.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"
#include "zf_common_interrupt.h"

// 数据不对时，注意定义的数据类型uint8是不是太小了
uint8 uart4_rx_buffer[UART4_RX_BUF_SIZE]; // 串口4接收缓冲区
uint8 fifo4_rx_buffer[UART4_RX_BUF_SIZE]; // FIFO输出读出缓冲区

uint16 uart4_rx_index = 0;
uint32 fifo4_rx_index = 0;

fifo_struct uart4_fifo;

/**
 * @brief 初始化UART4
 * @note  该函数用于初始化UART4串口
 */
void MyUART_4_Init()
{
    fifo_init(&uart4_fifo, FIFO_DATA_8BIT, uart4_rx_buffer, UART4_RX_BUF_SIZE); // 初始化 FIFO 挂载缓冲区
    uart_init(UART_4, 9600, UART4_TX_C16, UART4_RX_C17);
    uart_rx_interrupt(UART_4, 1);            // 开启 UART4 接收中断
    interrupt_set_priority(LPUART4_IRQn, 2); // 设置中断优先级
}

/**
 * @brief 串口接收中断处理函数
 * @note  该函数需要在对应串口的中断服务函数中调用
 *        示例：在isr.c的LPUART4_IRQHandler中调用此函数
 */
void uart4_rx_interrupt_handler(void)
{
    uint8 data;

    // 读取接收到的数据
    if(uart_query_byte(UART_4, &data))
    {
        // 将数据写入 FIFO 中
        fifo_write_buffer(&uart4_fifo, &data, 1); // 写入一个字节的数据到 FIFO 中
    }   

    // 如果接收到回车换行符，可以在这里处理完整的数据包
    if (data == '\n')
    {
        // 在这里添加数据处理逻辑
        uart_echo_data();
    }
}

/**
 * @brief 获取接收缓冲区中的数据
 * @param buffer 存放接收数据的缓冲区
 * @param len 需要读取的数据长度
 * @return 实际读取到的数据长度
 */
uint16 uart_rx_get_data(uint8 *buffer, uint16 len)
{
    uint16 i;
    fifo4_rx_index = fifo_used(&uart4_fifo); // 获取 FIFO 中的数据长度
    if (len > fifo4_rx_index)
    {
        len = fifo4_rx_index;
    }

    for (i = 0; i < len; i++)
    {
        buffer[i] = fifo4_rx_buffer[i];
    }

    return len;
}

/** 
 * @brief 将接收到的数据立即回传
 */

void uart_echo_data(void)
{

    fifo4_rx_index = fifo_used(&uart4_fifo); // 获取 FIFO 中的数据长度
    if (fifo4_rx_index > 0)
    {
        fifo_read_buffer(&uart4_fifo, fifo4_rx_buffer, &fifo4_rx_index, FIFO_READ_AND_CLEAN); // 从 FIFO 中读取数据并清空 FIFO
        uart_write_buffer(UART_4, fifo4_rx_buffer, fifo4_rx_index);                           // 将数据回传
    }
}
