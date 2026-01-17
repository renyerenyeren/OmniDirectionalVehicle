# 麦轮运动学算法库使用指南

## 概述

本库实现了麦轮全向车的运动学算法，提供正逆运动学计算功能。采用极简设计，只专注于数学计算，不包含控制逻辑（如PID、限幅等），给用户最大的灵活性。

## 架构设计

### 模块组成

```
┌────────────────────────────────────┐
│   MecanumKinematics            │
│   运动学算法（核心模块）        │
│   - 正运动学                    │
│   - 逆运动学                    │
└────────────────────────────────────┘
              ↑
              │ 依赖
              │
┌─────────────┴──────────────┐
│   omni_types.h              │
│   基础数据类型定义           │
└───────────────────────────────┘

┌────────────────────────────────────┐
│   OmniMotorDriverInterface      │
│   电机驱动接口（可选）          │
│   - 用于底层驱动注入            │
└────────────────────────────────────┘
```

### 设计理念

- **职责单一**：运动学模块只做数学计算
- **极简设计**：不包含控制逻辑（PID、限幅、状态管理等）
- **高度灵活**：用户可以根据需要组织自己的控制流程

## 文件结构

### 头文件 (inc/)

- `omni_types.h` - 基础数据类型定义
- `mecanum_kinematics.h` - 麦轮运动学接口
- `omni_motor_driver_interface.h` - 电机驱动接口（可选）

### 源文件 (src/)

- `mecanum_kinematics.c` - 运动学算法实现
- `omni_motor_driver_interface.c` - 电机驱动接口封装（可选）

## 使用步骤

### 步骤1: 定义车辆几何参数

```c
#include "omni_types.h"
#include "mecanum_kinematics.h"

/* 车辆几何参数 */
VehicleGeometry_t geometry = {
    .wheel_base = 0.20f,    // 200mm前后轴距
    .track_width = 0.20f,    // 200mm左右轮距
    .wheel_radius = 0.05f     // 50mm轮径
};
```

### 步骤2: 初始化运动学对象

```c
/* 创建运动学对象 */
IMecanumKinematics_t kinematics;

/* 初始化（可传入NULL使用默认几何参数） */
MecanumKinematics_Init(&kinematics, &geometry);
```

### 步骤3: 逆运动学计算（车体速度 -> 轮速）

```c
/* 设置目标车体速度 */
VehicleVelocity_t target_vel = {
    .vx = 0.5f,      // 前进 0.5 m/s
    .vy = 0.0f,      // 不横向移动
    .omega = 0.0f     // 不旋转
};

/* 计算目标轮速 */
WheelSpeeds_t wheel_speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &target_vel, &wheel_speeds);

/* 获取各轮速度（rad/s） */
float fl_speed = wheel_speeds.wheel_speeds[MOTOR_FRONT_LEFT];
float fr_speed = wheel_speeds.wheel_speeds[MOTOR_FRONT_RIGHT];
float rl_speed = wheel_speeds.wheel_speeds[MOTOR_REAR_LEFT];
float rr_speed = wheel_speeds.wheel_speeds[MOTOR_REAR_RIGHT];
```

### 步骤4: （可选）使用电机驱动接口

如果需要底层驱动注入，可以使用电机驱动接口。该接口提供三个函数指针：

- **set_motor_pwm**：设置电机PWM占空比
- **enable_motor**：使能/失能电机
- **speed_to_pwm**：速度转PWM转换（可在此实现PID）

```c
#include "omni_motor_driver_interface.h"

/* 定义底层驱动函数 */
void BSP_PWM_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle);
void BSP_PWM_EnableMotor(MotorIndex_e motor_index, bool enable);
float BSP_PWM_ConvertSpeed(float speed_rad_s);

/* 初始化电机驱动接口 */
IOmniMotorDriver_t motor_driver;
OmniMotorDriver_Init(&motor_driver, 
                    BSP_PWM_SetMotorPWM, 
                    BSP_PWM_EnableMotor, 
                    BSP_PWM_ConvertSpeed);

/* 输出到电机 */
for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++) {
    float pwm_duty = motor_driver.speed_to_pwm(wheel_speeds.wheel_speeds[i]);
    motor_driver.set_motor_pwm(i, pwm_duty);
}
```

### 步骤5: （可选）正运动学计算（轮速 -> 车体速度）

```c
/* 根据实际轮速推算车体速度 */
WheelSpeeds_t actual_wheel_speeds = {
    .wheel_speeds[MOTOR_FRONT_LEFT] = 10.0f,
    .wheel_speeds[MOTOR_FRONT_RIGHT] = 10.0f,
    .wheel_speeds[MOTOR_REAR_LEFT] = 10.0f,
    .wheel_speeds[MOTOR_REAR_RIGHT] = 10.0f
};

VehicleVelocity_t actual_vel;
kinematics.calculate_forward_kinematics(&kinematics, &actual_wheel_speeds, &actual_vel);

/* 获取实际车体速度 */
printf("Vx: %.3f m/s, Vy: %.3f m/s, Omega: %.3f rad/s\n", 
       actual_vel.vx, actual_vel.vy, actual_vel.omega);
```

## 使用示例

### 示例1: 前进

```c
VehicleVelocity_t vel = { .vx = 0.5f, .vy = 0.0f, .omega = 0.0f };
WheelSpeeds_t speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &vel, &speeds);

// 结果：四个轮子同向同速
```

### 示例2: 后退

```c
VehicleVelocity_t vel = { .vx = -0.5f, .vy = 0.0f, .omega = 0.0f };
WheelSpeeds_t speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &vel, &speeds);

// 结果：四个轮子反向同速
```

### 示例3: 左移

```c
VehicleVelocity_t vel = { .vx = 0.0f, .vy = 0.5f, .omega = 0.0f };
WheelSpeeds_t speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &vel, &speeds);

// 结果：左侧轮子向前，右侧轮子向后
```

### 示例4: 原地旋转

```c
VehicleVelocity_t vel = { .vx = 0.0f, .vy = 0.0f, .omega = 1.57f };
WheelSpeeds_t speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &vel, &speeds);

// 结果：左侧轮子反向，右侧轮子同向
```

### 示例5: 斜向移动

```c
VehicleVelocity_t vel = { .vx = 0.5f, .vy = 0.5f, .omega = 0.0f };
WheelSpeeds_t speeds;
kinematics.calculate_inverse_kinematics(&kinematics, &vel, &speeds);

// 结果：对角轮子同向，另外对角轮子反向
```

## 完整示例

```c
#include "omni_types.h"
#include "mecanum_kinematics.h"
#include "omni_motor_driver_interface.h"

/* 全局变量 */
IMecanumKinematics_t kinematics;
IOmniMotorDriver_t motor_driver;

/* 底层驱动函数（需要在BSP层实现）*/
void BSP_PWM_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle) {
    // TODO: 根据motor_index设置对应电机PWM
    // 示例：
    switch (motor_index) {
        case MOTOR_FRONT_LEFT:
            TIM1->CCR1 = (uint32_t)(duty_cycle * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_FRONT_RIGHT:
            TIM1->CCR2 = (uint32_t)(duty_cycle * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_REAR_LEFT:
            TIM1->CCR3 = (uint32_t)(duty_cycle * TIMER_PERIOD / 100.0f);
            break;
        case MOTOR_REAR_RIGHT:
            TIM1->CCR4 = (uint32_t)(duty_cycle * TIMER_PERIOD / 100.0f);
            break;
    }
}

void BSP_PWM_EnableMotor(MotorIndex_e motor_index, bool enable) {
    // TODO: 使能或失能对应电机
}

float BSP_PWM_ConvertSpeed(float speed_rad_s) {
    // TODO: 将角速度转换为PWM占空比
    // 示例：假设最大轮速20rad/s对应100% PWM
    return speed_rad_s / 20.0f * 100.0f;
}

int main(void) {
    /* 1. 初始化系统 */
    HAL_Init();
    SystemClock_Config();
    
    /* 2. 定义几何参数 */
    VehicleGeometry_t geometry = {
        .wheel_base = 0.20f,
        .track_width = 0.20f,
        .wheel_radius = 0.05f
    };
    
    /* 3. 初始化运动学对象 */
    MecanumKinematics_Init(&kinematics, &geometry);
    
    /* 4. 初始化电机驱动接口 */
    OmniMotorDriver_Init(&motor_driver, 
                        BSP_PWM_SetMotorPWM, 
                        BSP_PWM_EnableMotor, 
                        BSP_PWM_ConvertSpeed);
    
    /* 5. 主循环 */
    while (1) {
        /* 5.1 设置目标速度（这里可以替换为从遥控或上位机接收）*/
        VehicleVelocity_t target_vel = {
            .vx = 0.3f,
            .vy = 0.2f,
            .omega = 0.5f
        };
        
        /* 5.2 运动学逆解算 */
        WheelSpeeds_t wheel_speeds;
        kinematics.calculate_inverse_kinematics(&kinematics, &target_vel, &wheel_speeds);
        
        /* 5.3 速度限幅（用户自行实现）*/
        for (int i = 0; i < MOTOR_COUNT; i++) {
            wheel_speeds.wheel_speeds[i] = CLAMP(wheel_speeds.wheel_speeds[i], 
                                                -20.0f, 
                                                20.0f);
        }
        
        /* 5.4 输出到电机 */
        for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++) {
            float pwm_duty = motor_driver.speed_to_pwm(wheel_speeds.wheel_speeds[i]);
            pwm_duty = CLAMP(pwm_duty, -100.0f, 100.0f);
            motor_driver.set_motor_pwm(i, pwm_duty);
        }
        
        HAL_Delay(10);  // 10ms控制周期
    }
}
```

## 运动学原理

### 坐标系定义

```
     Y
     ↑
     |
     |
     +------→ X
    (前进方向)
```

- **X轴**：车体前进方向
- **Y轴**：车体左侧方向
- **Z轴**：向上（右手坐标系）
- **角速度ω**：逆时针为正

### 逆运动学公式

```
ω_FL = (Vx - Vy - ω*(Lx+Ly)) / R
ω_FR = (Vx + Vy + ω*(Lx+Ly)) / R
ω_RL = (Vx + Vy + ω*(Lx+Ly)) / R
ω_RR = (Vx - Vy - ω*(Lx+Ly)) / R
```

其中：
- `Vx`: X方向线速度 (m/s)
- `Vy`: Y方向线速度 (m/s)
- `ω`: 角速度 (rad/s)
- `Lx`: 前后轴距的一半 (wheel_base/2)，前轮为正
- `Ly`: 左右轮距的一半 (track_width/2)，右轮为正
- `R`: 轮子半径 (wheel_radius) (m)
- `ω_FL/FR/RL/RR`: 四个轮子的角速度 (rad/s)

### 轮子位置说明

- **前左轮(FL)**: 位置(Lx, -Ly)，对应公式中Lx+Ly项
- **前右轮(FR)**: 位置(Lx, +Ly)，对应公式中Lx+Ly项
- **后左轮(RL)**: 位置(-Lx, -Ly)，对应公式中-(Lx+Ly)项
- **后右轮(RR)**: 位置(-Lx, +Ly)，对应公式中-(Lx+Ly)项

### 正运动学公式

```
Vx = R/4 * (ω_FL + ω_FR + ω_RL + ω_RR)
Vy = R/4 * (-ω_FL + ω_FR + ω_RL - ω_RR)
ω  = R/(2*(Lx+Ly)) * (-ω_FL + ω_FR - ω_RL + ω_RR)
```

## API参考

### 运动学接口

#### MecanumKinematics_Init

```c
int MecanumKinematics_Init(IMecanumKinematics_t* kinematics,
                       const VehicleGeometry_t* geometry);
```

**参数：**
- `kinematics`: 运动学对象指针
- `geometry`: 车辆几何参数指针（NULL则使用默认值）

**返回：**
- 0: 成功
- -1: 失败

#### calculate_inverse_kinematics

```c
void calculate_inverse_kinematics(IMecanumKinematics_t* self,
                             const VehicleVelocity_t* vehicle_vel,
                             WheelSpeeds_t* wheel_speeds);
```

**功能：** 将车体速度转换为轮速

#### calculate_forward_kinematics

```c
void calculate_forward_kinematics(IMecanumKinematics_t* self,
                              const WheelSpeeds_t* wheel_speeds,
                              VehicleVelocity_t* vehicle_vel);
```

**功能：** 将轮速转换为车体速度

#### MecanumKinematics_SetGeometry

```c
void MecanumKinematics_SetGeometry(IMecanumKinematics_t* self,
                                const VehicleGeometry_t* geometry);
```

**功能：** 动态修改车辆几何参数

### 电机驱动接口（可选）

#### OmniMotorDriver_Init

```c
int OmniMotorDriver_Init(IOmniMotorDriver_t* driver,
                      SetMotorPWM_fn set_pwm,
                      EnableMotor_fn enable,
                      SpeedToPWM_fn speed_convert);
```

**参数：**
- `driver`: 电机驱动接口指针
- `set_pwm`: PWM设置函数指针
- `enable`: 电机使能函数指针
- `speed_convert`: 速度转PWM转换函数指针

**返回：**
- 0: 成功
- -1: 失败

#### 函数指针类型

```c
typedef void (*SetMotorPWM_fn)(MotorIndex_e motor_index, float duty_cycle);
typedef void (*EnableMotor_fn)(MotorIndex_e motor_index, bool enable);
typedef float (*SpeedToPWM_fn)(float speed_rad_s);
```

**说明：**
- `SetMotorPWM_fn`: 设置PWM占空比，范围-100.0~100.0，负值表示反转
- `EnableMotor_fn`: 使能或失能电机
- `SpeedToPWM_fn`: 将轮速(rad/s)转换为PWM占空比，可在此实现PID算法

## 注意事项

1. **单位一致性**：
   - 线速度：m/s
   - 角速度：rad/s
   - 几何参数：m

2. **速度范围**：
   - 本库不包含速度限幅功能
   - 用户需要根据实际电机性能自行限幅

3. **控制周期**：
   - 建议10ms-20ms周期调用逆运动学计算
   - 控制周期越短，控制精度越高

4. **轮子安装**：
   - 确保麦轮安装方向正确
   - 错误的安装方向会导致运动异常

5. **PID控制**：
   - 本库只提供运动学解算
   - 如需闭环控制，用户需自行实现PID
   - 可以在`SpeedToPWM_fn`函数中实现PID算法
   - 可以根据编码器反馈实现速度闭环

6. **电机驱动接口**：
   - 电机驱动接口是可选的
   - 如果不需要，可以完全绕过这个接口
   - 直接调用运动学计算，然后用自己的方式输出到电机

## 默认参数

如果初始化时传入NULL，将使用以下默认几何参数：

```c
#define DEFAULT_WHEEL_BASE 0.20f    // 200mm
#define DEFAULT_TRACK_WIDTH 0.20f    // 200mm
#define DEFAULT_WHEEL_RADIUS 0.05f   // 50mm
```

## 常见问题

### Q: 是否必须使用电机驱动接口？

A: 不是必须的。电机驱动接口只是一个便利的封装，你可以：
- 使用该接口（推荐，代码更清晰）
- 直接在BSP层实现PWM输出（更灵活，完全自定义）

### Q: 如何实现PID控制？

A: 在`SpeedToPWM_fn`函数中实现PID算法：

```c
float PID_Controller(float target_speed, float actual_speed) {
    static float integral = 0.0f;
    static float prev_error = 0.0f;
    
    float error = target_speed - actual_speed;
    integral += error;
    float derivative = error - prev_error;
    
    float output = Kp*error + Ki*integral + Kd*derivative;
    prev_error = error;
    
    return output;  // 返回PWM占空比
}
```