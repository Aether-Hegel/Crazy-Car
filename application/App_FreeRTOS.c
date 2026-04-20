#include "App_FreeRTOS.h"
#include "zf_common_headfile.h"

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

extern Speed_PID Speed_PID_L1;
extern Speed_PID Speed_PID_R1;

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

    taskEXIT_CRITICAL(); // 退出临界区
    vTaskDelete(NULL);   // 删除当前任务
    while (1)
    {
    }
}

void Motor_Task(void *param)
{

    Motor_Init();
    gpio_set_level(MOTOR_L1_DIR,1);
    while (1)
    {
        // Speed_PID_Init();                // 只在任务开始时初始化一次
        // Speed_PID_Calculate(&Speed_PID_L1, Ecoder_count_L1); // PID计算
        // PWM_CH1_Set_Duty(PWM_CH1_L1,(uint32)Speed_PID_L1.output); // 设置左前电机占空比
         Car_Forward(1000);
        // pwm_set_duty(PWM_CH1_L1, 1000); // 设置左前电机占空比
        gpio_toggle_level(B9);
        // printf("%f,%f,%f\n", Speed_PID_L1.target_speed, Speed_PID_L1.output, Speed_PID_L1.Actual_speed);
        // printf("%f,%f,%f\n", Speed_PID_L1.target_speed, Speed_PID_L1.output, Speed_PID_L1.Actual_speed);
        // printf("%d,\n",Ecoder_count_L1);
        // 电机控制代码
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100ms
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