# 模糊PID定位置控制使用方法

---

## 一、系统概述

### 1.1 硬件配置

| 硬件 | 型号/参数 | 说明 |
|------|----------|------|
| 电机 | 逐飞RC380有刷电机 | 低功耗、高转速 |
| 编码器 | 1024线正交解码迷你编码器 | 4倍频后4096脉冲/转 |
| 主控 | RT1064 | NXP Cortex-M7内核 |

### 1.2 控制原理

本系统采用**串级PID** + **模糊控制**的方案：

```
┌─────────────────────────────────────────────────────────┐
│                    串级PID结构                            │
│                                                          │
│   目标位置 ──→[位置环PID]──→ 目标速度 ──→[速度环PID]──→ PWM输出 │
│       │                                  │               │
│       ↓                                  ↓               │
│   编码器反馈                        编码器速度反馈       │
│   (位置信息)                        (速度信息)           │
└─────────────────────────────────────────────────────────┘
```

### 1.3 模糊控制原理

模糊控制器根据误差e和误差变化率ec自动调整PID参数：

| 模糊输入 | 说明 | 取值范围 |
|----------|------|----------|
| 误差e | 目标位置-实际位置 | -5000 ~ +5000 脉冲 |
| 误差变化率ec | e(k)-e(k-1) | -2000 ~ +2000 |

| 模糊输出 | 说明 |
|----------|------|
| ΔKp | 比例系数修正量 |
| ΔKi | 积分系数修正量 |
| ΔKd | 微分系数修正量 |

---

## 二、软件接口

### 2.1 文件结构

```
Algorithm_Processor/
├── Fuzzy_Pid.h    // 头文件: 数据结构定义、函数声明
└── Fuzzy_Pid.c    // 源文件: 模糊PID实现
```

### 2.2 核心数据类型

```c
// 电机通道定义
#define FUZZY_POS_PID_L1    0   // 左电机
#define FUZZY_POS_PID_R1    1   // 右电机

// 串级PID输出结构体
typedef struct {
    float position_out;   // 位置环输出(目标速度)
    float speed_out;      // 速度环输出(电机PWM值)
} Cascade_PID_Output;
```

### 2.3 函数接口

| 函数 | 功能 | 调用时机 |
|------|------|----------|
| `Fuzzy_PID_Init()` | 初始化PID参数 | 系统启动时调用一次 |
| `Fuzzy_PID_Set_Target(id, pos)` | 设置目标位置 | 需要移动时调用 |
| `Fuzzy_PID_Calculate(id)` | 执行PID计算 | 定时中断中调用 |
| `Fuzzy_PID_Get_Output(id)` | 获取输出值 | 计算后调用 |
| `Get_Motor_Position(id)` | 获取当前位置 | 需要读取位置时 |
| `Clear_Motor_Position(id)` | 清零位置 | 需要归零时 |

---

## 三、使用步骤

### 3.1 系统初始化

在系统启动时，按以下顺序初始化：

```c
void System_Init(void)
{
    // 1. 底层硬件初始化
    Motor_Init();       // 电机驱动初始化
    Encoder_Init();     // 编码器初始化
    
    // 2. 算法初始化(放在最后)
    Fuzzy_PID_Init();   // 模糊PID初始化
    
    // 3. 位置清零(可选)
    Clear_Motor_Position(0xFF);  // 所有电机位置清零
}
```

### 3.2 定时中断配置

在定时器中断中周期调用PID计算函数：

```c
// 推荐周期: 10ms (100Hz)
void TIM3_IRQHandler(void)
{
    if(timer_check_interrupt(TIM_3))
    {
        timer_clear_interrupt_flag(TIM_3);
        
        // 更新所有电机
        Fuzzy_PID_Calculate(0xFF);
    }
}
```

### 3.3 设置目标位置

```c
// 让左电机转动1圈 (4096脉冲)
Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);

// 让右电机转动2圈 (8192脉冲)
Fuzzy_PID_Set_Target(FUZZY_POS_PID_R1, 8192);

// 让电机反转半圈 (-2048脉冲)
Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, -2048);
```

### 3.4 驱动电机

在主循环或定时任务中获取输出并驱动电机：

```c
void Motor_Control_Task(void)
{
    // 获取左电机输出
    Cascade_PID_Output out_l = Fuzzy_PID_Get_Output(FUZZY_POS_PID_L1);
    
    // 获取右电机输出
    Cascade_PID_Output out_r = Fuzzy_PID_Get_Output(FUZZY_POS_PID_R1);
    
    // 设置电机PWM (根据实际驱动方式调整)
    Motor_Set_PWM(MOTOR_L1, (int16)out_l.speed_out);
    Motor_Set_PWM(MOTOR_R1, (int16)out_r.speed_out);
}
```

### 3.5 完整示例

```c
#include "Fuzzy_Pid.h"
#include "Motor.h"
#include "Encoder.h"

// 任务: 让左电机转动2圈后停止
void Position_Control_Example(void)
{
    // 初始化
    Fuzzy_PID_Init();
    
    // 清零位置
    Clear_Motor_Position(FUZZY_POS_PID_L1);
    
    // 设置目标: 2圈 = 8192脉冲
    Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 8192);
    
    // 等待到达目标
    while(1)
    {
        int32 current_pos = Get_Motor_Position(FUZZY_POS_PID_L1);
        int32 target = 8192;
        
        printf("当前位置: %d / %d 脉冲\n", current_pos, target);
        
        // 到达目标位置附近时停止
        if(abs(current_pos - target) < 10)
        {
            Motor_Set_PWM(MOTOR_L1, 0);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## 四、参数说明

### 4.1 编码器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| ENCODER_RESOLUTION | 1024 | 编码器线数 |
| GEAR_RATIO | 1 | 齿轮减速比 |
| PULSE_PER_REV | 4096 | 每转脉冲数(1024×4) |

### 4.2 PID基础参数

| 参数 | 默认值 | 作用 | 调整建议 |
|------|--------|------|----------|
| FUZZY_KP_BASE | 15.0 | 比例系数 | 影响响应速度，值越大响应越快 |
| FUZZY_KI_BASE | 0.5 | 积分系数 | 消除稳态误差，值越大稳态误差越小 |
| FUZZY_KD_BASE | 8.0 | 微分系数 | 抑制超调，值越大系统越稳定 |

### 4.3 限幅参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| POSITION_PID_MAX_OUT | 500.0 | 位置环输出限幅(速度目标上限) |
| SPEED_PID_MAX_OUT | 8000.0 | 速度环输出限幅(PWM上限) |
| INTEGRAL_LIMIT | 400.0 | 积分限幅 |

### 4.4 速度环参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Speed_Kp | 12.0 | 速度环比例系数 |
| Speed_Ki | 1.0 | 速度环积分系数 |
| Speed_Kd | 5.0 | 速度环微分系数 |

---

## 五、调参指南

### 5.1 调参顺序

```
1. 先将模糊规则表注释掉，使用固定PID参数
2. 先调速度环(内环)，再调位置环(外环)
3. 速度环调好后，开启模糊控制优化位置环
```

### 5.2 速度环调参步骤

1. **设置较小的目标速度**
   ```c
   Fuzzy_Cascade_PID_L1.speed_pid.target = 100;  // 低速测试
   ```

2. **调节Kp**
   - 从小值开始，逐渐增大
   - 观察响应速度
   - 目标：响应快、无振荡

3. **调节Kd**
   - 抑制超调
   - 目标：平滑、无振荡

4. **微调Ki**
   - 消除稳态误差
   - 值不宜过大，防止积分饱和

### 5.3 位置环调参步骤

1. **设置较小的目标位置**
   ```c
   Fuzzy_PID_Set_Target(FUZZY_POS_PID_L1, 4096);  // 1圈
   ```

2. **调节位置环KP**
   - 从小值开始，逐渐增大
   - 观察到达目标的速度

3. **开启模糊控制**
   - 取消模糊规则表的注释
   - 观察在不同位置时的响应

### 5.4 模糊规则调整

如果系统响应不理想，可调整模糊规则表：

| 现象 | 调整方法 |
|------|----------|
| 响应太慢 | 增大Rule_Kp中的Kp_p值 |
| 超调过大 | 增大Rule_Kd中的Kd_p值 |
| 稳态误差 | 增大Rule_Ki中的Ki_p值 |
| 振荡 | 减小Rule_Kp，增大Rule_Kd |

---

## 六、常见问题

### 6.1 电机不转

| 可能原因 | 解决方法 |
|----------|----------|
| PWM输出为0 | 检查Fuzzy_PID_Get_Output返回值 |
| 方向引脚配置错误 | 检查MOTOR_L1_DIR等引脚定义 |
| 编码器无脉冲 | 检查编码器连接和初始化 |

### 6.2 电机持续振荡

| 可能原因 | 解决方法 |
|----------|----------|
| Kp过大 | 减小KP_BASE |
| Kd过小 | 增大KD_BASE |
| 采样周期太长 | 减小定时中断周期 |

### 6.3 电机到达目标后不停

| 可能原因 | 解决方法 |
|----------|----------|
| 积分累积过多 | 减小Ki或积分限幅 |
| 目标位置设置错误 | 检查Set_Target参数 |

---

## 七、附录

### 7.1 脉冲与角度换算

```
电机转角 = 脉冲数 / 4096 × 360°
```

### 7.2 速度计算

```
速度(脉冲/秒) = 速度(脉冲/采样周期) × 采样频率
```

### 7.3 联系支持

如有问题，请联系逐飞科技技术支持。

---

*文档版本: 1.0*
*更新日期: 2026-03-22*
