/*********************************************************************************************************************
 *  @file           wifi.c
 *  @brief          逐飞科技 WIFI-SPI (WF6B21-SPI V2.0) 模块驱动封装 源文件
 *
 *  @description    基于逐飞 zf_device_wifi_spi 底层驱动的二次封装，提供:
 *                  - WiFi 连接和 TCP 通道建立 (模块作为 TCP Client)
 *                  - 从上位机接收逗号分隔浮点数据帧 (如 "2.89,23.23,43.23,5756")
 *                  - 向上位机发送浮点数组或任意字符串
 *
 *  @protocol       TCP (传输控制协议)
 *                  与 UDP 的主要区别:
 *                  - TCP 是面向连接的, 发送前必须先建立连接 (三次握手)
 *                  - TCP 保证数据可靠按序到达, 丢包会自动重传
 *                  - TCP 发送数据后, 数据进入 TCP 发送缓冲区, 由协议栈自动管理发送
 *                    不需要像 UDP 那样调用 wifi_spi_udp_send_now() 手动触发
 *                  - 上位机必须先启动 TCP Server, 模块作为 Client 去连接
 *
 *  @date           2026-04-19
 ********************************************************************************************************************/

#include "wifi.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "App_FreeRTOS.h"
#include "task.h"

//须在libraries\zf_device\zf_device_wifi_spi.h修改IP地址和端口号为上位机的IP地址和监听端口号
/*------------------------------------------------------------------------------------------------------------------
 *  全局变量
 *------------------------------------------------------------------------------------------------------------------*/

/** @brief 全局接收帧结构体, 调用 wifi_receive() 后结果自动存入此变量 */
wifi_frame_t wifi_rx_frame;

/*------------------------------------------------------------------------------------------------------------------
 *  私有变量
 *------------------------------------------------------------------------------------------------------------------*/

/** @brief WiFi SPI 原始接收缓冲区 (模块内部使用, 外部不可直接访问) */
static uint8 wifi_rx_buffer[WIFI_RX_BUF_SIZE];

/*------------------------------------------------------------------------------------------------------------------
 *  函数实现
 *------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief   WiFi 模块初始化 (连接 WiFi + 建立 TCP Client 连接)
 *
 * @details 执行流程:
 *          1. 等待 300ms 确保模块上电稳定
 *          2. 调用 wifi_spi_init() 连接到指定 WiFi 热点 (失败则循环重试)
 *          3. 通过 Debug 串口打印模块固件版本、MAC地址、IP地址
 *          4. 如果 WIFI_SPI_AUTO_CONNECT == 0 (手动连接模式):
 *             调用 wifi_spi_socket_connect() 以 "TCP" 模式建立到上位机的连接 (失败则循环重试)
 *          5. 清空接收帧结构体
 *
 * @param   void    无参数
 *
 * @return  uint8
 *          - 0: 初始化成功 (WiFi 连接 + TCP 连接均已建立)
 *          - 函数内部会阻塞重试, 正常情况下总是返回 0
 *
 * @note    调用前必须先执行:
 *          - clock_init(SYSTEM_CLOCK_600M);  // 系统时钟初始化
 *          - debug_init();                   // 调试串口初始化
 *
 * @note    TCP 连接顺序 (重要!):
 *          因为模块作为 TCP Client, 上位机作为 TCP Server, 所以:
 *          1. 先在电脑上打开网络调试助手, 选择 "TCP Server" 模式, 点击"监听"
 *          2. 然后再给模块上电 / 复位
 *          如果顺序反了, 模块会一直打印 "TCP 连接 Server 失败", 直到上位机启动 Server 为止
 *
 * @warning 此函数是阻塞式的, 连接 WiFi 和建立 TCP 可能耗时数秒
 *          如果一直卡在此函数, 请检查:
 *          - WiFi 名称和密码是否正确
 *          - 模块硬件连接是否正确
 *          - 是否使用了 5V 电源供电
 *          - 上位机 TCP Server 是否已经启动并监听
 *          - WIFI_SPI_TARGET_IP 是否为上位机的实际 IP 地址
 */
uint8 wifi_module_init(void)
{
    /* 第1步: 等待模块上电稳定 */
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 第2步: 连接 WiFi 热点 (失败则循环重试) */
    while(wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("\r\n[WiFi] 连接 WiFi 失败, 正在重试...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* 第3步: 打印模块信息 (通过 Debug 串口输出) */
    printf("\r\n[WiFi] ==================== 模块信息 ====================");
    printf("\r\n[WiFi] 固件版本 : %s", wifi_spi_version);       // 例如 "V2.0.1"
    printf("\r\n[WiFi] MAC 地址 : %s", wifi_spi_mac_addr);      // 例如 "AA:BB:CC:DD:EE:FF"
    printf("\r\n[WiFi] IP  地址 : %s", wifi_spi_ip_addr_port);  // 例如 "192.168.43.50:6666"
    printf("\r\n[WiFi] ====================================================");

    /* 第4步: 如果未开启自动连接, 则手动建立 TCP Client 连接 */
    /*
     * TCP 与 UDP 连接的关键区别:
     * - UDP: wifi_spi_socket_connect("UDP", ...) 只是设置目标地址, 不需要对方在线
     * - TCP: wifi_spi_socket_connect("TCP", ...) 会执行三次握手, 必须对方 Server 在线
     *
     * 因此 TCP 模式下, 如果上位机 TCP Server 还没启动, 这里会一直重试
     */
    if(0 == WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect(
            "TCP",                          // 通信协议: 使用 TCP (模块作为 TCP Client)
            WIFI_SPI_TARGET_IP,             // 上位机(TCP Server)的 IP 地址 (在 zf_device_wifi_spi.h 中配置)
            WIFI_SPI_TARGET_PORT,           // 上位机(TCP Server)的监听端口  (在 zf_device_wifi_spi.h 中配置)
            WIFI_SPI_LOCAL_PORT))           // 本地端口号                     (在 zf_device_wifi_spi.h 中配置)
        {
            printf("\r\n[WiFi] TCP 连接 Server 失败, 正在重试...");
            printf("\r\n[WiFi]   Server IP   = %s", WIFI_SPI_TARGET_IP);
            printf("\r\n[WiFi]   Server 端口 = %s", WIFI_SPI_TARGET_PORT);
            printf("\r\n[WiFi]   本地端口    = %s", WIFI_SPI_LOCAL_PORT);
            printf("\r\n[WiFi]   请确认上位机 TCP Server 已启动并监听!");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    printf("\r\n[WiFi] 初始化完成, WiFi 和 TCP 连接均已建立!\r\n");

    /* 第5步: 清空接收帧结构体 */
    memset(&wifi_rx_frame, 0, sizeof(wifi_rx_frame));

    return 0;
}

/**
 * @brief   解析逗号分隔的浮点数字符串帧
 *
 * @details 将 "2.89,23.23,43.23,5756" 格式的字符串解析为浮点数数组
 *          处理流程:
 *          1. 参数有效性检查
 *          2. 拷贝到临时缓冲区 (避免修改原始字符串)
 *          3. 去除尾部的 '\r', '\n', 空格
 *          4. 以逗号 ',' 为分隔符, 逐个解析浮点数
 *          5. 将结果填入 frame 结构体
 *
 * @param   str     [in]  待解析的原始字符串
 *                  - 不可为 NULL
 *                  - 格式: 逗号分隔的数值, 如 "2.89,23.23,43.23,5756"
 *                  - 支持: 正数、负数、整数、小数 (如 "-1.5,0,100,3.14")
 *                  - 允许尾部有 '\r', '\n', 空格 (会自动去除)
 *                  - 最大长度: WIFI_RX_BUF_SIZE - 1 (默认 255 字节)
 *
 * @param   frame   [out] 解析结果输出到的结构体
 *                  - 不可为 NULL
 *                  - 成功: frame->data[] 按顺序存储浮点数,
 *                          frame->count = 解析到的个数,
 *                          frame->valid = 1
 *                  - 失败: frame->valid = 0, 其他字段不保证有效
 *
 * @return  uint8   解析结果
 *                  - 0: 成功 (解析到 >= 1 个浮点数)
 *                  - 1: 失败 (参数无效 / 输入为空 / 无法解析)
 *
 * @note    浮点数精度: 使用 atof() 转换, 精度约 6~7 位有效数字 (float)
 *          如果超过 WIFI_FLOAT_MAX_COUNT 个数值, 多余部分会被忽略
 *
 * @code
 *          // 示例: 手动解析一个字符串
 *          wifi_frame_t result;
 *          wifi_parse_frame("10.5,-3.14,0,255", &result);
 *          // result.data[0] = 10.5f
 *          // result.data[1] = -3.14f
 *          // result.data[2] = 0.0f
 *          // result.data[3] = 255.0f
 *          // result.count   = 4
 *          // result.valid   = 1
 * @endcode
 */
uint8 wifi_parse_frame(const char *str, wifi_frame_t *frame)
{
    char    buf[WIFI_RX_BUF_SIZE];  // 临时缓冲区, 避免修改原始字符串
    char   *token;                  // 当前分割出的子串指针
    char   *saveptr;                // strtok_r 内部状态指针
    uint8   idx = 0;                // 当前已解析的浮点数计数

    /* 参数有效性检查 */
    if(str == NULL || frame == NULL || strlen(str) == 0)
    {
        if(frame != NULL)
        {
            frame->valid = 0;
        }
        return 1;
    }

    /* 拷贝到临时缓冲区 */
    strncpy(buf, str, WIFI_RX_BUF_SIZE - 1);
    buf[WIFI_RX_BUF_SIZE - 1] = '\0';

    /* 去除尾部的回车、换行、空格 */
    uint16 len = strlen(buf);
    while(len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' '))
    {
        buf[--len] = '\0';
    }

    if(len == 0)
    {
        frame->valid = 0;
        return 1;
    }

    /* 以逗号为分隔符逐个解析浮点数 */
    token = strtok_r(buf, ",", &saveptr);
    while(token != NULL && idx < WIFI_FLOAT_MAX_COUNT)
    {
        frame->data[idx] = (float)atof(token);     // 将字符串转换为 float
        idx++;
        token = strtok_r(NULL, ",", &saveptr);      // 继续解析下一个
    }

    /* 填写解析结果 */
    frame->count = idx;
    frame->valid = (idx > 0) ? 1 : 0;

    return (idx > 0) ? 0 : 1;
}

/**
 * @brief   从 WiFi 模块接收数据并自动解析
 *
 * @details 执行流程:
 *          1. 清空接收缓冲区
 *          2. 调用 wifi_spi_read_buffer() 通过 SPI 读取模块中缓存的数据
 *          3. 如果收到数据, 自动调用 wifi_parse_frame() 解析
 *          4. 解析结果存入全局变量 wifi_rx_frame
 *
 * @param   void    无参数
 *
 * @return  uint32  本次接收到的原始字节数
 *                  - 0:  没有新数据
 *                  - >0: 收到的字节数, wifi_rx_frame 已更新
 *                  - 最大值: WIFI_RX_BUF_SIZE - 1 (默认 255)
 *
 * @note    非阻塞函数, 无数据时立即返回 0
 *          建议调用间隔: 10ms ~ 100ms
 *          调用过于频繁会增加 SPI 总线负载, 过于稀疏可能丢失数据
 *
 * @note    TCP 粘包说明:
 *          TCP 是流式协议, 上位机连续快速发送多帧数据时, 可能被合并为一次到达
 *          例如上位机快速发送 "1.0,2.0\n" 和 "3.0,4.0\n", 模块可能一次收到 "1.0,2.0\n3.0,4.0\n"
 *          本函数只解析第一帧, 建议上位机发送间隔 >= 模块轮询间隔, 或降低发送频率
 *
 * @code
 *          // 典型用法: 在循环中轮询接收
 *          while(1)
 *          {
 *              if(wifi_receive() > 0 && wifi_rx_frame.valid)
 *              {
 *                  // 处理接收到的数据
 *                  float kp = wifi_rx_frame.data[0];
 *                  float ki = wifi_rx_frame.data[1];
 *                  float kd = wifi_rx_frame.data[2];
 *              }
 *              system_delay_ms(50);  // 50ms 轮询间隔
 *          }
 * @endcode
 */
uint32 wifi_receive(void)
{
    uint32 len;

    /* 清空接收缓冲区 */
    memset(wifi_rx_buffer, 0, WIFI_RX_BUF_SIZE);

    /* 通过 SPI 从 WiFi 模块读取数据 */
    len = wifi_spi_read_buffer(wifi_rx_buffer, WIFI_RX_BUF_SIZE - 1);

    if(len > 0)
    {
        /* 确保字符串以 '\0' 结尾 */
        wifi_rx_buffer[len] = '\0';

        /* 自动解析为浮点数帧, 结果存入 wifi_rx_frame */
        wifi_parse_frame((const char *)wifi_rx_buffer, &wifi_rx_frame);
    }

    return len;
}

/**
 * @brief   通过 WiFi (TCP) 发送字符串到上位机
 *
 * @details 执行流程:
 *          1. 参数有效性检查
 *          2. 调用 wifi_spi_send_buffer() 将数据写入 SPI 发送缓冲区
 *          3. 数据进入模块的 TCP 发送缓冲区后, 由 TCP 协议栈自动发出
 *
 *          与 UDP 发送的区别:
 *          - UDP 发送后需要调用 wifi_spi_udp_send_now() 手动触发发包
 *          - TCP 发送后数据自动进入 TCP 流, 由协议栈管理发送时机
 *            不需要手动触发, 也不需要调用 wifi_spi_udp_send_now()
 *
 * @param   str     [in] 要发送的 C 字符串 (以 '\0' 结尾)
 *                  - 不可为 NULL
 *                  - 长度范围: 1 ~ 4088 字节 (受 WIFI_SPI_TRANSFER_SIZE 限制)
 *                  - 可以包含任意可打印字符和控制字符 (\r, \n 等)
 *
 * @return  uint8   发送结果
 *                  - 0: 发送成功 (所有字节已写入 SPI 缓冲)
 *                  - 1: 发送失败 (str 为 NULL, 或部分数据未能写入)
 *
 * @note    TCP 可靠性: TCP 协议保证数据按序可靠到达,
 *          如果网络暂时中断, TCP 会自动重传直到成功或连接断开
 *
 * @code
 *          // 发送简单消息
 *          wifi_send_string("Hello!\n");
 *
 *          // 发送带格式的调试信息
 *          char msg[128];
 *          snprintf(msg, sizeof(msg), "encoder=%d,speed=%.2f\n", enc_val, speed);
 *          wifi_send_string(msg);
 * @endcode
 */
uint8 wifi_send_string(const char *str)
{
    uint32 remain;

    /* 参数检查 */
    if(str == NULL)
    {
        return 1;
    }

    /* 将数据写入 SPI 发送缓冲区, TCP 协议栈会自动管理发送 */
    remain = wifi_spi_send_buffer((const uint8 *)str, strlen(str));

    /*
     * TCP 模式下不需要调用 wifi_spi_udp_send_now()
     * 数据写入 SPI 后, 模块的 TCP 协议栈会自动将数据通过 TCP 连接发出
     * wifi_spi_udp_send_now() 仅用于 UDP 模式下手动触发发包
     */

    /* remain == 0 表示全部写入成功, >0 表示有 remain 字节未能写入 */
    return (remain == 0) ? 0 : 1;
}

/**
 * @brief   将浮点数数组格式化为逗号分隔字符串并通过 TCP 发送
 *
 * @details 执行流程:
 *          1. 参数有效性检查
 *          2. 将 data[0] ~ data[count-1] 格式化为 "%.2f,%.2f,...,%.2f\n"
 *          3. 调用 wifi_send_string() 通过 TCP 发送格式化后的字符串
 *
 *          格式化示例:
 *          - 输入: data = {2.89, 23.23, 43.23, 5756.0}, count = 4
 *          - 输出: "2.89,23.23,43.23,5756.00\n"
 *
 * @param   data    [in] float 数组指针
 *                  - 不可为 NULL
 *                  - 每个元素取值范围: float 有效范围 (约 ±3.4e38)
 *                  - 每个浮点数格式化后约占 3~12 个字符
 *
 * @param   count   [in] 要发送的浮点数个数
 *                  - 取值范围: 1 ~ WIFI_FLOAT_MAX_COUNT (默认 1~10)
 *                  - 不可为 0
 *                  - 注意: count 个浮点数格式化后的总长度不应超过 WIFI_RX_BUF_SIZE
 *
 * @return  uint8   发送结果
 *                  - 0: 格式化并发送成功
 *                  - 1: 失败 (参数无效 / 发送出错)
 *
 * @note    每个浮点数保留 2 位小数 (%.2f 格式)
 *          数据之间用英文逗号 ',' 分隔, 末尾附加换行符 '\n'
 *          如需修改小数位数, 请修改函数内 snprintf 的格式字符串
 *
 * @code
 *          // 示例1: 发送3个传感器数据
 *          float sensor[3] = {25.6f, 1013.25f, 45.0f};
 *          wifi_send_floats(sensor, 3);
 *          // 上位机收到: "25.60,1013.25,45.00\n"
 *
 *          // 示例2: 回传接收到的数据 (回环测试)
 *          if(wifi_receive() > 0 && wifi_rx_frame.valid)
 *          {
 *              wifi_send_floats(wifi_rx_frame.data, wifi_rx_frame.count);
 *          }
 *
 *          // 示例3: 发送 PID 调试数据
 *          float pid[4] = {100.0f, 98.5f, 15.3f, 1.5f};  // 目标, 实际, 输出, 误差
 *          wifi_send_floats(pid, 4);
 *          // 上位机收到: "100.00,98.50,15.30,1.50\n"
 * @endcode
 */
uint8 wifi_send_floats(const float *data, uint8 count)
{
    char    buf[WIFI_RX_BUF_SIZE];  // 格式化输出缓冲区
    uint16  offset = 0;             // 当前写入位置偏移量
    uint8   i;                      // 循环计数

    /* 参数检查 */
    if(data == NULL || count == 0)
    {
        return 1;
    }

    /* 逐个格式化浮点数, 用逗号分隔 */
    for(i = 0; i < count; i++)
    {
        /* 从第2个数开始, 在前面加逗号分隔符 */
        if(i > 0)
        {
            buf[offset++] = ',';
        }

        /* 将浮点数格式化为字符串, 保留2位小数 */
        offset += snprintf(buf + offset,                    // 写入位置
                           WIFI_RX_BUF_SIZE - offset,       // 剩余缓冲区大小
                           "%.2f",                          // 格式: 2位小数
                           data[i]);                        // 当前浮点数

        /* 防止缓冲区溢出 */
        if(offset >= WIFI_RX_BUF_SIZE - 2)
        {
            break;
        }
    }

    /* 末尾添加换行符, 方便上位机按行解析 */
    buf[offset++] = '\n';
    buf[offset]   = '\0';

    /* 调用字符串发送函数 (内部通过 TCP 发出) */
    return wifi_send_string(buf);
}
