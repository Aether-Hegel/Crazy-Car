/*********************************************************************************************************************
 *  @file           wifi.h
 *  @brief          逐飞科技 WIFI-SPI (WF6B21-SPI V2.0) 模块驱动封装 头文件
 *
 *  @description    本模块基于逐飞科技 zf_device_wifi_spi 底层驱动进行二次封装，
 *                  提供 WiFi 连接、TCP 数据收发、浮点数帧解析等功能。
 *                  适用于智能车上位机通信场景，支持接收如 "2.89,23.23,43.23,5756" 的逗号分隔浮点数据帧，
 *                  也支持向上位机发送同格式的数据帧。
 *
 *  @protocol       TCP (传输控制协议)
 *                  - 模块作为 TCP Client, 上位机作为 TCP Server
 *                  - TCP 是面向连接的可靠协议, 保证数据按序到达、不丢包
 *                  - 相比 UDP: 数据更可靠, 但延迟略高
 *                  - 上位机必须先启动 TCP Server 监听, 模块才能连接成功
 *
 *  @module         逐飞科技 高速WIFI模块 WF6B21-SPI V2.0
 *  @interface      SPI 接口
 *
 *  @hardware       硬件连接说明 (引脚定义在 zf_device_wifi_spi.h 中配置):
 *                  -------------------------------------------------------
 *                  | 模块引脚    | 芯片引脚           | 说明              |
 *                  |------------|--------------------|--------------------|
 *                  | RST        | WIFI_SPI_RST_PIN   | 复位引�� (B16)    |
 *                  | INT        | WIFI_SPI_INT_PIN   | 中���引脚 (B17)    |
 *                  | CS         | WIFI_SPI_CS_PIN    | 片选引脚 (D13)    |
 *                  | MISO       | WIFI_SPI_MISO_PIN  | 主入从出 (D15)    |
 *                  | SCK        | WIFI_SPI_SCK_PIN   | 时钟引�� (D12)    |
 *                  | MOSI       | WIFI_SPI_MOSI_PIN  | 主出从入 (D14)    |
 *                  | 5V         | 5V                 | 电源正极           |
 *                  | GND        | GND                | 电源地             |
 *                  -------------------------------------------------------
 *
 *  @attention      1. 必须使用电源供电 (不要仅用调试器供电, 会导致模块欠压)
 *                  2. WiFi 名称/密码在本文件的 WIFI_SSID / WIFI_PASSWORD 宏定义中修改
 *                  3. 目标IP地址和端口在 zf_device_wifi_spi.h 中的 WIFI_SPI_TARGET_IP / WIFI_SPI_TARGET_PORT 修改
 *                  4. isr.c 中 GPIO1_Combined_16_31_IRQHandler 必须调用 wireless_module_spi_handler()
 *                  5. 使用本模块时无法使用并口屏幕 (引脚冲突), 请改用串口屏幕
 *                  6. 上位机必须先启动 TCP Server 监听端口, 然后再给模块上电/复位
 *
 *  @date           2026-04-19
 ********************************************************************************************************************/

#ifndef __WIFI_H__
#define __WIFI_H__

#include "zf_common_typedef.h"

/*------------------------------------------------------------------------------------------------------------------
 *  用户配置宏定���
 *------------------------------------------------------------------------------------------------------------------*/

/**
 * @def     WIFI_SSID
 * @brief   要��接的 WiFi 热点名称 (SSID)
 * @note    修改为你实际使用的路由器或手机热点名称
 *          长度不超过 32 个字符
 */
#define WIFI_SSID               "lly"

/**
 * @def     WIFI_PASSWORD
 * @brief   WiFi 热点密码
 * @note    如果 WiFi 没有密码, 请将 "12345678" 替换为 NULL
 *          长度不超过 64 个���符
 */
#define WIFI_PASSWORD           "p7krsest"

/**
 * @def     WIFI_RX_BUF_SIZE
 * @brief   WiFi 接收缓冲区大小 (单���: 字节)
 * @note    取值范围: 64 ~ 4088, 需要能容纳一帧完整数据
 *          默认 256 字节, 足够容纳常规浮点数据帧
 */
#define WIFI_RX_BUF_SIZE        256

/**
 * @def     WIFI_FLOAT_MAX_COUNT
 * @brief   单帧中最多可解析的浮点数个数
 * @note    取值范围: 1 ~ 255
 *          例如上位机发送 "2.89,23.23,43.23,5756" 包含4个浮点数
 *          如果实际需要更多字段, 请增大此值
 */
#define WIFI_FLOAT_MAX_COUNT    10

/*------------------------------------------------------------------------------------------------------------------
 *  数据结构定义
 *------------------------------------------------------------------------------------------------------------------*/

/**
 * @struct  wifi_frame_t
 * @brief   WiFi 接收数据帧解析结果结构体
 *
 * @details 用于存储从上位机接收到的逗号分隔浮点数据帧解析后的结果
 *          上位机发送的数据格式示例: "2.89,23.23,43.23,5756\n"
 *
 * @member  data    解析���的浮点数数组
 *                  - 类型: float
 *                  - 大小: WIFI_FLOAT_MAX_COUNT 个元素
 *                  - data[0] 对应帧中第1个浮点数, data[1] 对��第2个, 以此类推
 *                  - 未使用的元素值为 0.0f
 *
 * @member  count   本帧中实际解析到的浮点数��数
 *                  - 类型: uint8 (0~255)
 *                  - 取值范围: 0 ~ WIFI_FLOAT_MAX_COUNT
 *                  - 例如 "2.89,23.23,43.23,5756" 解析后 count = 4
 *
 * @member  valid   帧数据是��有效
 *                  - 类型: uint8
 *                  - 0: 无效 (未收到数据或解析失败)
 *                  - 1: 有效 (成功解析到至少1个浮点数)
 */
typedef struct
{
    float   data[WIFI_FLOAT_MAX_COUNT];
    uint8   count;
    uint8   valid;
} wifi_frame_t;

/**
 * @var     wifi_rx_frame
 * @brief   全局接收帧结构体实例
 * @details 调用 wifi_receive() 后, 解析结果自动存入此变量
 *          可在任意位置通过 wifi_rx_frame.data[i] 读取解析后的浮点数
 */
extern wifi_frame_t wifi_rx_frame;

/*------------------------------------------------------------------------------------------------------------------
 *  函数声明
 *------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief   WiFi ���块初始化 (连接WiFi + 建立TCP连接)
 *
 * @param   void    无参数 (WiFi名称和密码通过 WIFI_SSID / WIFI_PASSWORD 宏配置)
 *
 * @return  uint8   初始化结果
 *                  - 0: 初始化成功
 *                  - 该函数内部会一直重试直到成功, 正常情况下总是返回 0
 *
 * @note    调用前提:
 *          1. 必须先调用 clock_init() 和 debug_init()
 *          2. isr.c 中 GPIO1_Combined_16_31_IRQHandler 需调用 wireless_module_spi_handler()
 *          3. 确保 zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_IP 设置为上位机的 IP 地址
 *          4. 确保 zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_PORT 设置为上位机监听的端口号
 *          5. 上位机必须先启动 TCP Server 监听, 否则模块会一直重试连接
 *
 * @note    耗时说明: 首次连接WiFi可能需要数秒, 函数会阻塞直到连接成功
 *
 * @note    TCP 连接说明:
 *          - 模块作为 TCP Client, 主动连接上位机的 TCP Server
 *          - 上位机必须先启动 TCP Server 并监听指定端口
 *          - 如果上位机未启动 TCP Server, 模块会循环重试直到连接成功
 *
 * @warning 初始化过程中不要断电, 否则可能导致模块异��
 *
 * @code
 *          // 在 main.c 中调用
 *          clock_init(SYSTEM_CLOCK_600M);
 *          debug_init();
 *          wifi_module_init();  // 阻塞直到WiFi和TCP连接成功
 * @endcode
 */
uint8 wifi_module_init(void);

/**
 * @brief   解析逗号分隔的浮点数字符串帧
 *
 * @param   str     [in]  待解析的字符串指针
 *                  - 不可为 NULL
 *                  - 格式: 逗号分隔的浮点数, 例如 "2.89,23.23,43.23,5756"
 *                  - 支持整数和浮点数混合, 例如 "100,3.14,0.5"
 *                  - 自动忽略尾部的 '\r', '\n', 空格
 *                  - 最大长度: WIFI_RX_BUF_SIZE - 1 字节
 *
 * @param   frame   [out] 解析结果输出结构体指针
 *                  - 不可为 NULL
 *                  - 解析成功后: frame->data[] 存储浮点数, frame->count 为个数, frame->valid = 1
 *                  - 解析失败后: frame->valid = 0
 *
 * @return  uint8   解析结果
 *                  - 0: 解析成功 (至少解析到1��浮点数)
 *                  - 1: 解析失败 (输入为空/格式错误/参数为NULL)
 *
 * @note    该函数通常由 wifi_receive() 内部自动调用, 一般不需要用户手动调用
 *          如需手动解析自定义字符串, 可以直接调用此函数
 *
 * @code
 *          wifi_frame_t my_frame;
 *          const char *test_str = "2.89,23.23,43.23,5756";
 *
 *          if(0 == wifi_parse_frame(test_str, &my_frame))
 *          {
 *              // 解析成功
 *              // my_frame.data[0] = 2.89f
 *              // my_frame.data[1] = 23.23f
 *              // my_frame.data[2] = 43.23f
 *              // my_frame.data[3] = 5756.0f
 *              // my_frame.count   = 4
 *              // my_frame.valid   = 1
 *          }
 * @endcode
 */
uint8 wifi_parse_frame(const char *str, wifi_frame_t *frame);

/**
 * @brief   从 WiFi 模块接收数据并自动解析为浮���数帧
 *
 * @param   void    无参数
 *
 * @return  uint32  本次���收到的原始字节数
 *                  - 0: 未收到任何数据
 *                  - >0: 收到的字节数, 解析结果已存入全局变量 wifi_rx_frame
 *
 * @note    该���数为非阻塞调用, 如果没有数据则立即返回 0
 *          建议在主循环或 FreeRTOS 任务中周期性调用 (推荐间隔: 10~100ms)
 *
 * @note    接收流程:
 *          1. 通过 SPI 从 WiFi 模块读取原始字节
 *          2. 自动调用 wifi_parse_frame() 解析为浮点数
 *          3. 结果存入 wifi_rx_frame, 可直���读取
 *
 * @note    TCP 特性: TCP 是流式协议, 可能出现粘包 (多帧数据合并到达)
 *          本函数每次只解析缓冲区中的一帧数据, 建议上位机发送时每帧以 '\n' 结尾
 *
 * @code
 *          // 在 FreeRTOS 任务或 while(1) 中周期性调用
 *          uint32 rx_len = wifi_receive();
 *          if(rx_len > 0 && wifi_rx_frame.valid)
 *          {
 *              printf("收到 %d 个浮点数:\r\n", wifi_rx_frame.count);
 *              for(uint8 i = 0; i < wifi_rx_frame.count; i++)
 *              {
 *                  printf("  data[%d] = %.2f\r\n", i, wifi_rx_frame.data[i]);
 *              }
 *          }
 * @endcode
 */
uint32 wifi_receive(void);

/**
 * @brief   通过 WiFi (TCP) 发送字符���到上位机
 *
 * @param   str     [in] 要发送的字符串指针 (以 '\0' 结尾的 C 字符串)
 *                  - 不可为 NULL
 *                  - 最大长度: WIFI_SPI_TRANSFER_SIZE (4088 字节)
 *                  - 支持任意格式��字符串
 *
 * @return  uint8   发送结��
 *                  - 0: 发���成功
 *                  - 1: 发送���败 (参数为NULL 或 SPI传输出错)
 *
 * @note    数据通过 TCP 协议发送, 写入 SPI 缓冲后由模块自动通过 TCP 流发出
 *          TCP 保证数据可靠按序到达, 不需要像 UDP 那样手动触发发送
 *
 * @code
 *          // 发送普通��本
 *          wifi_send_string("Hello from RT1064!\n");
 *
 *          // 发送格式化字符串
 *          char buf[64];
 *          snprintf(buf, sizeof(buf), "speed=%.1f,angle=%.1f\n", 12.5f, 45.0f);
 *          wifi_send_string(buf);
 * @endcode
 */
uint8 wifi_send_string(const char *str);

/**
 * @brief   将浮点数���组格式化为逗号分隔字符串并通过 WiFi (TCP) 发送
 *
 * @param   data    [in] 浮点数数组指针
 *                  - 不可为 NULL
 *                  - 数组中每���元素为 float 类型
 *                  - 取值范��: float 有效范围 (约 ±3.4e38)
 *
 * @param   count   [in] 要发送的浮点数个数
 *                  - 类型: uint8
 *                  - 取值范围: 1 ~ WIFI_FLOAT_MAX_COUNT (默认最大10)
 *                  - 不可为 0
 *
 * @return  uint8   发送结果
 *                  - 0: 发送成功
 *                  - 1: 发送失败 (参数为NULL/count为0/SPI传输出错)
 *
 * @note    发送格式: "浮��数1,浮点数2,...,浮点数N\n"
 *          每个浮点数保留2位小数, 末尾自动添加���行符 '\n'
 *          例如: data = {2.89, 23.23, 43.23}, count = 3
 *                发送内容为 "2.89,23.23,43.23\n"
 *
 * @code
 *          // 发送 PID 调试数���给上位机
 *          float pid_data[4] = {target_speed, actual_speed, pid_output, error};
 *          wifi_send_floats(pid_data, 4);
 *          // 上位机收到: "100.00,98.50,15.30,1.50\n"
 * @endcode
 */
uint8 wifi_send_floats(const float *data, uint8 count);

#endif /* __WIFI_H__ */

/*********************************************************************************************************************
 *
 *  ============================== 完整使用案例 ==============================
 *
 *  ------------------------------ 前提条件 ----------------------------------
 *
 *  1. 硬件连接:
 *     将 WF6B21-SPI V2.0 模块按照上方引脚表连���到 RT1064 核心板
 *     务必使用 5V 电��供电 (不要仅依赖调试器供电)
 *
 *  2. WiFi 配置:
 *     - 修改本文件中 WIFI_SSID 为你的路由器/手机热点名称
 *     - 修改本文件中 WIFI_PASSWORD 为你的 WiFi 密码
 *
 *  3. 上位机 IP 配置:
 *     - 打开 zf_device_wifi_spi.h
 *     - 将 WIFI_SPI_TARGET_IP 修改为你电脑的 IP 地址 (如 "192.168.1.100")
 *     - 将 WIFI_SPI_TARGET_PORT 修改为上位机 TCP Server 监听端口 (如 "8086")
 *
 *  4. 中断配置 (isr.c):
 *     确保 GPIO1_Combined_16_31_IRQHandler 中已调用 wireless_module_spi_handler():
 *
 *     void GPIO1_Combined_16_31_IRQHandler(void)
 *     {
 *         wireless_module_spi_handler();  // WiFi SPI 中断处理 (必须保留)
 *         if(exti_flag_get(B16))
 *         {
 *             exti_flag_clear(B16);
 *         }
 *     }
 *
 *  ------------------------------ 上位机启动顺序 (重要!) ----------------------
 *
 *  因为使用 TCP 协议, 模块是 TCP Client, 上位机是 TCP Server, 所以:
 *
 *      [第1步] 电脑连接 WiFi
 *      [第2步] 打开网络调试助手, 选择 TCP Server 模式, 点击"监听"
 *      [第3步] 给模块上电 / 复位下载程��
 *      [第4步] 等待模块连接成功 (串口会打印 "TCP 连接建立成功")
 *      [第5步] 网络调试助手会显示有客户端连入, 即可开始收发数据
 *
 *      如果顺序反了 (先给模块上电, 后开 TCP Server), 模块会一直重试连接,
 *      串口会反复打印 "TCP 连接 Server 失败", 直到上位机启动 TCP Server 为止.
 *
 *  ------------------------------ 案例1: 裸机 (无OS) 使用 ---------------------
 *
 *  #include "zf_common_headfile.h"
 *  #include "wifi.h"
 *
 *  int main(void)
 *  {
 *      clock_init(SYSTEM_CLOCK_600M);
 *      debug_init();
 *
 *      // ---- 第1步: 初始化 WiFi 模块 ----
 *      wifi_module_init();
 *      // 执行到这里说明 WiFi 连接和 TCP 连接均已成功
 *
 *      // ---- 第2步: 主循环中收发数据 ----
 *      while(1)
 *      {
 *          // --- 接收数据 ---
 *          // 上位机发送: "2.89,23.23,43.23,5756\n"
 *          uint32 rx_len = wifi_receive();
 *          if(rx_len > 0 && wifi_rx_frame.valid)
 *          {
 *              // 成功解析, 读取各字段
 *              float val0 = wifi_rx_frame.data[0];  // 2.89
 *              float val1 = wifi_rx_frame.data[1];  // 23.23
 *              float val2 = wifi_rx_frame.data[2];  // 43.23
 *              float val3 = wifi_rx_frame.data[3];  // 5756.0
 *              uint8 num  = wifi_rx_frame.count;     // 4
 *
 *              printf("收到 %d 个数据: %.2f, %.2f, %.2f, %.2f\r\n",
 *                     num, val0, val1, val2, val3);
 *          }
 *
 *          // --- 发送数据 ---
 *          // 方���A: 发送浮��数组 (自动格式��为逗号分隔)
 *          float send_data[3] = {1.50f, 2.70f, 3.90f};
 *          wifi_send_floats(send_data, 3);
 *          // 上位机收到: "1.50,2.70,3.90\n"
 *
 *          // 方式B: 发送任意字符串
 *          wifi_send_string("status:OK\n");
 *
 *          system_delay_ms(100);
 *      }
 *  }
 *
 *  ------------------------------ 案例2: FreeRTOS 任务中使用 ------------------
 *
 *  // 在 main.c 中初始化
 *  int main(void)
 *  {
 *      clock_init(SYSTEM_CLOCK_600M);
 *      debug_init();
 *      wifi_module_init();   // WiFi 初始化 (阻塞直到成功)
 *      APP_FreeRTOS_Start(); // 启动 FreeRTOS 调度器
 *      while(1) {}
 *  }
 *
 *  // 在 FreeRTOS 任务中收发数据
 *  void WiFi_Task(void *param)
 *  {
 *      float pid_send[4];
 *
 *      while(1)
 *      {
 *          // 接���上位机下发的 PID 参数: "Kp,Ki,Kd"
 *          if(wifi_receive() > 0 && wifi_rx_frame.valid)
 *          {
 *              if(wifi_rx_frame.count >= 3)
 *              {
 *                  float new_kp = wifi_rx_frame.data[0];
 *                  float new_ki = wifi_rx_frame.data[1];
 *                  float new_kd = wifi_rx_frame.data[2];
 *                  // 在这里更新 PID ��数...
 *              }
 *          }
 *
 *          // 上报当前 PID 状态给上位机
 *          pid_send[0] = target_speed;
 *          pid_send[1] = actual_speed;
 *          pid_send[2] = pid_output;
 *          pid_send[3] = error;
 *          wifi_send_floats(pid_send, 4);
 *          // 上位机收到: "100.00,98.50,15.30,1.50\n"
 *
 *          vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 周期
 *      }
 *  }
 *
 *  ------------------------------ 案例3: 回环测试 (收到什么发回什么) -----------
 *
 *  while(1)
 *  {
 *      if(wifi_receive() > 0 && wifi_rx_frame.valid)
 *      {
 *          // 原样回传解析到的浮点数
 *          wifi_send_floats(wifi_rx_frame.data, wifi_rx_frame.count);
 *      }
 *      system_delay_ms(50);
 *  }
 *
 *  ============================== 上位机配置说明 ==============================
 *
 *  1. 打开网络调试助手 (如 NetAssist / SSCOM / 串口猎人 等)
 *  2. 协议类型选择: TCP Server
 *  3. 本地IP选择: 你电脑的 IP (如 192.168.43.100)
 *  4. 本地端口设置为: WIFI_SPI_TARGET_PORT 中定义的端口 (默认 8086)
 *  5. 点击 "监听" 按钮, 开始等待模块连入
 *  6. 给模块上电后, 调试助手会提示有客户端连入
 *  7. 发送格式示例: 2.89,23.23,43.23,5756
 *
 *  注意: 上位机必须选择 "TCP Server" 模式, 不是 "TCP Client"!
 *        因为 WiFi 模块是作为 TCP Client 主动连接上位机的.
 *
 ********************************************************************************************************************/
