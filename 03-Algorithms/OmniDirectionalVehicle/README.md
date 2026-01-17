# 麦轮全向车控制系统使用指南

## 概述

本系统实现了一个基于STM32的4轮麦轮全向车控制系统，采用C语言面向对象设计模式，通过依赖注入实现底层解耦。

## 架构设计

### 分层架构

```
┌────────────────────────────────────┐
│   OmniVehiclePort (依赖注入端口)     │
│   - 管理对象生命周期                  │
│   - 统一初始化接口                    │
└─────────────┬──────────────────────┘
              │
    ┌─────────┴──────────┐
    │                    │
┌───▼────┐        ┌────▼──────────┐
│Controller│        │  Kinematics │
│  控制器  │        │   运动学      │
└───┬────┘        └─────────────┘
    │
┌───▼────────────┐
│ MotorDriver   │
│  电机驱动接口  │
└───────────────┘
       │
       ▼
┌──────────────┐
│  BSP层PWM    │
│  (用户实现)  │
└──────────────┘
```

### 设计模式

- **依赖注入**: 通过函数指针注入底层驱动，解耦算法层和硬件层
- **工厂模式**: Port负责创建和管理所有对象
- **门面模式**: Port对外提供统一接口
- **单一职责原则**: 每个模块专注于特定功能

## 文件结构

### 头文件 (inc/)

- `omni_types.h` - 基础数据类型定义
- `omni_motor_driver_interface.h` - 电机驱动接口
- `mecanum_kinematics.h` - 麦轮运动学接口
- `omni_vehicle_controller.h` - 车辆控制器接口
- `omni_vehicle_port.h` - 依赖注入端口

### 源文件 (src/)

- `mecanum_kinematics.c` - 运动学算法实现
- `omni_vehicle_controller.c` - 控制器逻辑实现
- `omni_motor_driver_interface.c` - 电机驱动接口封装
- `omni_vehicle_port.c` - 依赖注入端口实现

## 使用步骤

### 步骤1: 实现BSP层PWM驱动

在 `01-BSP/PWM/` 目录下实现以下函数：

```c
// 01-BSP/PWM/inc/bsp_pwm.h
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "omni_types.h"

void BSP_PWM_Init(void);
void BSP_PWM_SetDutyCycle(MotorIndex_e motor, float duty);
void BSP_PWM_EnableMotor(MotorIndex_e motor, bool enable);
float BSP_PWM_SpeedToPWM(float speed_rad_s);

#endif /* BSP_PWM_H */
```

```c
// 01-BSP/PWM/src/bsp_pwm.c
#include "bsp_pwm.h"

void BSP_PWM_Init(void)
{
    // TODO: 根据CubeMX配置初始化PWM
}

void BSP_PWM_SetDutyCycle(MotorIndex_e motor, float duty)
{
    // TODO: 设置指定电机的PWM占空比
    switch (motor) {
        case MOTOR_FRONT_LEFT:
            // TIM1->CCR1 = (uint32_t)(duty * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_FRONT_RIGHT:
            // TIM1->CCR2 = (uint32_t)(duty * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_REAR_LEFT:
            // TIM1->CCR3 = (uint32_t)(duty * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_REAR_RIGHT:
            // TIM1->CCR4 = (uint32_t)(duty * TIMER_PERIOD / 100.0f);
            break;
    }
}

void BSP_PWM_EnableMotor(MotorIndex_e motor, bool enable)
{
    // TODO: 使能或失能指定电机
}

float BSP_PWM_SpeedToPWM(float speed_rad_s)
{
    // TODO: 实现速度到PWM的转换
    float pwm = speed_rad_s / MAX_WHEEL_SPEED * 100.0f;
    return pwm;
}
```

### 步骤2: 在main.c中使用

```c
/* Includes ------------------------------------------------------------------*/
#include "omni_vehicle_port.h"
#include "bsp_pwm.h"

/* Private variables ---------------------------------------------------------*/
OmniVehiclePort_t omni_port;

/* Private function prototypes -----------------------------------------------*/
static void BSP_PWM_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle);
static void BSP_PWM_EnableMotor(MotorIndex_e motor_index, bool enable);
static float BSP_PWM_ConvertSpeed(float speed_rad_s);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    
    /* 初始化BSP层PWM */
    BSP_PWM_Init();
    
    /* 步骤1: 初始化Port */
    VehicleGeometry_t geometry = {
        .wheel_base = 0.20f,    // 200mm轴距
        .track_width = 0.20f,    // 200mm轮距
        .wheel_radius = 0.05f     // 50mm轮径
    };
    OmniVehiclePort_Init(&omni_port, &geometry);
    
    /* 步骤2: 注入BSP层PWM驱动函数 */
    OmniVehiclePort_InjectPWMDriver(&omni_port,
                                   BSP_PWM_SetMotorPWM,
                                   BSP_PWM_EnableMotor,
                                   BSP_PWM_ConvertSpeed);
    
    /* 步骤3: 获取控制器对象 */
    OmniVehicleController_t* controller = OmniVehiclePort_GetController(&omni_port);
    
    /* 步骤4: 使能控制器 */
    controller->enable(controller, true);
    
    /* 步骤5: 设置目标速度 */
    VehicleVelocity_t target_vel = {
        .vx = 0.5f,      // 前进 0.5 m/s
        .vy = 0.0f,      // 不横向移动
        .omega = 0.0f    // 不旋转
    };
    controller->set_velocity(controller, &target_vel);
    
    /* 主循环 */
    while (1)
    {
        /* 周期调用控制器更新 */
        controller->update(controller);
        
        HAL_Delay(10);  // 10ms控制周期
    }
}

/* BSP层PWM驱动函数实现 */
static void BSP_PWM_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle)
{
    BSP_PWM_SetDutyCycle(motor_index, duty_cycle);
}

static void BSP_PWM_EnableMotor(MotorIndex_e motor_index, bool enable)
{
    BSP_PWM_EnableMotor(motor_index, enable);
}

static float BSP_PWM_ConvertSpeed(float speed_rad_s)
{
    return BSP_PWM_SpeedToPWM(speed_rad_s);
}
```

## 使用示例

### 示例1: 前进
```c
VehicleVelocity_t vel = { .vx = 0.5f, .vy = 0.0f, .omega = 0.0f };
controller->set_velocity(controller, &vel);
controller->update(controller);
```

### 示例2: 后退
```c
VehicleVelocity_t vel = { .vx = -0.5f, .vy = 0.0f, .omega = 0.0f };
controller->set_velocity(controller, &vel);
controller->update(controller);
```

### 示例3: 左移
```c
VehicleVelocity_t vel = { .vx = 0.0f, .vy = 0.5f, .omega = 0.0f };
controller->set_velocity(controller, &vel);
controller->update(controller);
```

### 示例4: 原地旋转
```c
VehicleVelocity_t vel = { .vx = 0.0f, .vy = 0.0f, .omega = 1.57f };
controller->set_velocity(controller, &vel);
controller->update(controller);
```

### 示例5: 斜向移动
```c
VehicleVelocity_t vel = { .vx = 0.5f, .vy = 0.5f, .omega = 0.0f };
controller->set_velocity(controller, &vel);
controller->update(controller);
```

### 示例6: 紧急停止
```c
controller->emergency_stop(controller);
```

## 运动学原理

### 逆运动学公式

```
ω_FL = (Vx - Vy - ω*(Lx+Ly)) / R
ω_FR = (Vx + Vy + ω*(Lx+Ly)) / R
ω_RL = (Vx + Vy + ω*(Lx+Ly)) / R
ω_RR = (Vx - Vy - ω*(Lx+Ly)) / R
```

其中：
- `Vx`: X方向线速度
- `Vy`: Y方向线速度
- `ω`: 角速度
- `Lx`: 前后轴距的一半 (wheel_base/2)，前轮为正
- `Ly`: 左右轮距的一半 (track_width/2)，右轮为正
- `R`: 轮子半径 (wheel_radius)

说明：
- 前左轮(FL): 位置(Lx, -Ly)，对应公式中Lx+Ly项
- 前右轮(FR): 位置(Lx, +Ly)，对应公式中Lx+Ly项
- 后左轮(RL): 位置(-Lx, -Ly)，对应公式中-(Lx+Ly)项
- 后右轮(RR): 位置(-Lx, +Ly)，对应公式中-(Lx+Ly)项

## 注意事项

1. **坐标系定义**:
   - X轴: 车体前进方向
   - Y轴: 车体左侧方向
   - Z轴: 向上（右手坐标系）

2. **速度限制**:
   - 默认最大线速度: 1.0 m/s
   - 默认最大角速度: 3.14 rad/s
   - 默认最大轮速: 20.0 rad/s

3. **PWM占空比**:
   - 范围: -100.0 ~ 100.0
   - 正值: 正转
   - 负值: 反转

4. **控制周期**:
   - 建议10ms-20ms周期调用 `update()` 函数
