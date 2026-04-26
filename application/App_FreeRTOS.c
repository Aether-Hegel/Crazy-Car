#include "App_FreeRTOS.h"
#include "zf_common_headfile.h"
#include "wifi.h"
#include "PID.h"

#define START_TASK_PRIORITY 1
#define START_TASK_STACK_SIZE 128
TaskHandle_t Start_Task_Handle;
void Start_Task(void *param);

#define Motor_Task_PRIORITY 2
#define Motor_Task_STACK_SIZE 256
TaskHandle_t Motor_Task_Handle;
void Motor_Task(void *param);

#define Speed_Task_PRIORITY 3
#define Speed_Task_STACK_SIZE 256
TaskHandle_t Speed_Task_Handle;
void Speed_Task(void *param);

#define Wifi_Task_PRIORITY 4
#define Wifi_Task_STACK_SIZE 256
TaskHandle_t Wifi_Task_Handle;
void Wifi_Task(void *param);

extern Speed_PID Speed_PID_L1;
extern Speed_PID Speed_PID_R1;

extern Fuzzy_PD_Controller pd_L1, pd_L2, pd_R1, pd_R2;

extern int16 Encoder_Current_L1;
extern int16 Encoder_Current_R1;
extern int16 Encoder_Current_L2;
extern int16 Encoder_Current_R2;

void APP_FreeRTOS_Start(void)
{
    // 1. 创建任务
    // (1) taskFunction 任务函数
    // (2) 函数名称  没有实质性作用 就是看名字
    // (3) taskStackDepth 任务堆栈大小  单位是4字节
    // (4) 任务参数  任意类型  可以传递给任务函数  可以为空
    // (5) 任务优先级  数字越大 优先级越高   范围是 0-(configMAX_PRIORITIES-1)
    // (6) 任务句柄  用于控制任务  可以为空

    xTaskCreate(Start_Task, "Start_Task", START_TASK_STACK_SIZE, NULL, START_TASK_PRIORITY, &Start_Task_Handle);
    vTaskStartScheduler();
}

void Start_Task(void *param)
{
    taskENTER_CRITICAL(); // 进入临界区

    // 创建其他任务
    xTaskCreate(Motor_Task, "Motor_Task", Motor_Task_STACK_SIZE, NULL, Motor_Task_PRIORITY, &Motor_Task_Handle);
    xTaskCreate(Speed_Task, "Speed_Task", Speed_Task_STACK_SIZE, NULL, Speed_Task_PRIORITY, &Speed_Task_Handle);
    xTaskCreate(Wifi_Task, "Wifi_Task", Wifi_Task_STACK_SIZE, NULL, Wifi_Task_PRIORITY, &Wifi_Task_Handle);

    taskEXIT_CRITICAL(); // 退出临界区
    vTaskDelete(NULL);   // 删除当前任务
}

void Motor_Task(void *param)
{
    Clear_Motor_Position(0xFF); // 全部电机位置清零，建立新零点
    vTaskDelay(pdMS_TO_TICKS(20)); // 确保清零生效，建议 5~20 ms

    Cascade_PID_Set_Target(CASCADE_CH_L1, 40960); // 正转1圈

    while (1)
    {
        // Speed_PID_Init();                // 只在任务开始时初始化一次
        // Speed_PID_Calculate(&Speed_PID_L1, Encoder_Current_L1); // PID计算
        // pwm_set_duty(PWM_CH2_R1,(uint32)Speed_PID_L1.output); // 设置电机占空比
        gpio_toggle_level(B9);
        // Car_Forward(1500);
        // pwm_set_duty(PWM_CH2_R1 , 1000); // 设置左前电机占空比

        /* 调整为 10ms 控制周期以匹配模糊PD的量化与经验值（注释中推荐 5~20 ms） */
        // Fuzzy_PID_Calculate(FUZZY_POS_PID_L1);
        // if (pd_L1.output > 0)
        // {
        //     gpio_set_level(MOTOR_L1_DIR, 1); // 正转
        //     pwm_set_duty(PWM_CH1_L1,(uint32)pd_L1.output); // 设置电机占空比
        // }
        // else if (pd_L1.output < 0)
        // {
        //     gpio_set_level(MOTOR_L1_DIR, 0); // 反转
        //     pwm_set_duty(PWM_CH1_L1,(uint32)(-pd_L1.output)); // 设置电机占空比
        // }
        Cascade_PID_Calculate(CASCADE_CH_L1);
        Cascade_PID_Drive_Motor(CASCADE_CH_L1);
        printf("%d,%.2f,%d\n", pd_L1.target_position, pd_L1.output, pd_L1.actual_position);

        /* 控制周期 10ms，匹配模糊PD推荐值 5~20ms，避免相位滞后导致振荡 */
        vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms
    }
}

void Speed_Task(void *param)
{
    while (1)
    {

        // 速度控制代码
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100ms
    }
}

void Wifi_Task(void *param)
{
    // wifi_module_init(); // 初始化WiFi模块并建立连接
    while (1)
    {

        // Wifi控制代码
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100ms
    }
}
