# OmniVehicleControl 使用说明

## 概述

`OmniVehicleControl` 是全向移动车辆控制模块的统一接口，封装了运动学计算、电机驱动和 PID 控制器。通过简单的 API 调用，即可实现车体速度控制、PID 参数调整等功能。

## 目录

- [模块架构](#模块架构)
- [快速开始](#快速开始)
- [API 详细说明](#api-详细说明)
  - [初始化](#初始化)
  - [速度控制](#速度控制)
  - [PID 控制](#pid-控制)
  - [电机控制](#电机控制)
  - [状态查询](#状态查询)
  - [资源释放](#资源释放)
- [使用示例](#使用示例)
- [注意事项](#注意事项)

---

## 模块架构

```
OmniVehicleControl (统一控制层)
    ├─ OmniSolve (运动学解算)
    │   └─ Mecanum Kinematics (麦轮运动学)
    ├─ MotorDriver (硬件抽象层)
    │   └─ PWM 输出接口
    └─ Velocity_PID (速度控制)
        └─ 4个 PID 控制器
```

---

## 快速开始

### 1. 头文件包含

```c
#include "omni_vehicle_control.h"
```

### 2. 最小化示例

```c
// 1. 准备车辆几何参数
VehicleGeometry_t geometry = {
    .wheel_radius = 0.05f,  // 轮子半径 (m)
    .wheelbase_length = 0.2f,  // 轴距 (m)
    .wheeltrack_width = 0.2f   // 轮距 (m)
};

// 2. 准备 PID 参数 (4个电机)
float pid_params[MOTOR_COUNT][3] = {
    {1.0f, 0.1f, 0.0f},  // 前左电机 [Kp, Ki, Kd]
    {1.0f, 0.1f, 0.0f},  // 前右电机
    {1.0f, 0.1f, 0.0f},  // 后右电机
    {1.0f, 0.1f, 0.0f}   // 后左电机
};

// 3. 定义编码器读取函数（接收电机编号参数）
float read_encoder(uint8_t motor_index) {
    switch (motor_index) {
        case MOTOR_FRONT_LEFT:
            // 读取前左电机编码器，返回速度 (rad/s)
            return (float)TIM4->CNT * 0.001f;
        case MOTOR_FRONT_RIGHT:
            // 读取前右电机编码器，返回速度 (rad/s)
            return (float)TIM5->CNT * 0.001f;
        case MOTOR_REAR_RIGHT:
            // 读取后右电机编码器，返回速度 (rad/s)
            return (float)TIM6->CNT * 0.001f;
        case MOTOR_REAR_LEFT:
            // 读取后左电机编码器，返回速度 (rad/s)
            return (float)TIM7->CNT * 0.001f;
        default:
            return 0.0f;
    }
}

// 4. 定义 PWM 设置函数
void set_pwm(MotorIndex_e motor, float duty_cycle) {
    // 根据 motor 编号设置对应电机的 PWM
    // duty_cycle 范围: -100.0 ~ 100.0
}

// 5. 定义电机使能函数
void enable_motor(MotorIndex_e motor, bool enable) {
    // 使能或失能指定电机
}

// 6. 初始化控制模块
int ret = OmniVehicleControl_Inst(
    set_pwm,
    enable_motor,
    read_encoder,  // 传入单个编码器函数
    &geometry,
    pid_params
);

if (ret != 0) {
    // 初始化失败处理
    return -1;
}

// 7. 设置车体速度
VehicleVelocity_t vel = {
    .vx = 0.5f,   // 前进速度 (m/s)
    .vy = 0.0f,   // 横移速度 (m/s)
    .omega = 0.0f // 旋转角速度 (rad/s)
};

OmniVehicleControl_SetVelocity(&vel);
```

---

## API 详细说明

### 初始化

#### `OmniVehicleControl_Inst`

初始化全向车控制模块，必须在调用其他 API 之前调用。

```c
int OmniVehicleControl_Inst(
    SetMotorPWM_fn set_pwm,
    EnableMotor_fn enable_motor,
    ReadEncoder_fn read_encoder,
    const VehicleGeometry_t* geometry,
    const float pid_params[MOTOR_COUNT][3]
);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `set_pwm` | `SetMotorPWM_fn` | PWM 设置函数指针 |
| `enable_motor` | `EnableMotor_fn` | 电机使能函数指针 |
| `read_encoder` | `ReadEncoder_fn` | 编码器读取函数（接收电机编号参数） |
| `geometry` | `VehicleGeometry_t*` | 车辆几何参数指针 |
| `pid_params` | `float[][3]` | PID 参数数组，格式为 `{{Kp,Ki,Kd}, ...}` |

**返回值：**

- `0`：成功
- `-1`：失败（参数为空或初始化失败）

**函数指针类型定义：**

```c
// PWM 设置函数
typedef void (*SetMotorPWM_fn)(MotorIndex_e motor_index, float duty_cycle);

// 电机使能函数
typedef void (*EnableMotor_fn)(MotorIndex_e motor_index, bool enable);

// 编码器读取函数（接收电机编号作为参数）
typedef float (*ReadEncoder_fn)(uint8_t motor_index);
```

**使用示例：**

```c
// 定义 PWM 设置函数
void my_set_pwm(MotorIndex_e motor, float duty_cycle) {
    switch (motor) {
        case MOTOR_FRONT_LEFT:
            TIM3->CCR1 = (uint32_t)(abs(duty_cycle) * 10); // 示例值
            break;
        case MOTOR_FRONT_RIGHT:
            TIM3->CCR2 = (uint32_t)(abs(duty_cycle) * 10);
            break;
        case MOTOR_REAR_RIGHT:
            TIM3->CCR3 = (uint32_t)(abs(duty_cycle) * 10);
            break;
        case MOTOR_REAR_LEFT:
            TIM3->CCR4 = (uint32_t)(abs(duty_cycle) * 10);
            break;
    }
}

// 定义电机使能函数
void my_enable_motor(MotorIndex_e motor, bool enable) {
    if (enable) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 + motor, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 + motor, GPIO_PIN_RESET);
    }
}

// 初始化
int ret = OmniVehicleControl_Inst(my_set_pwm, my_enable_motor, read_encoder, &geometry, pid_params);

---

### 速度控制

#### `OmniVehicleControl_SetVelocity`

设置车体速度，内部自动完成：
1. 运动学逆解：车体速度 → 轮速
2. PID 控制：轮速 → PWM
3. 电机驱动：PWM 输出

```c
void OmniVehicleControl_SetVelocity(const VehicleVelocity_t* vehicle_vel);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `vehicle_vel` | `VehicleVelocity_t*` | 车体速度结构体指针 |

**`VehicleVelocity_t` 结构体：**

```c
typedef struct {
    float vx;     // X 轴线速度 (m/s)，正值为前进，负值为后退
    float vy;     // Y 轴线速度 (m/s)，正值为右移，负值为左移
    float omega;  // 旋转角速度 (rad/s)，正值为逆时针旋转
} VehicleVelocity_t;
```

**使用示例：**

```c
// 示例 1：前进 0.5 m/s
VehicleVelocity_t vel = {.vx = 0.5f, .vy = 0.0f, .omega = 0.0f};
OmniVehicleControl_SetVelocity(&vel);

// 示例 2：原地右移 0.3 m/s
VehicleVelocity_t vel = {.vx = 0.0f, .vy = 0.3f, .omega = 0.0f};
OmniVehicleControl_SetVelocity(&vel);

// 示例 3：原地旋转，角速度 1.0 rad/s
VehicleVelocity_t vel = {.vx = 0.0f, .vy = 0.0f, .omega = 1.0f};
OmniVehicleControl_SetVelocity(&vel);

// 示例 4：斜向移动 (前进+右移)
VehicleVelocity_t vel = {.vx = 0.3f, .vy = 0.2f, .omega = 0.0f};
OmniVehicleControl_SetVelocity(&vel);

// 示例 5：停止
VehicleVelocity_t vel = {.vx = 0.0f, .vy = 0.0f, .omega = 0.0f};
OmniVehicleControl_SetVelocity(&vel);
```

---

### PID 控制

#### `OmniVehicleControl_SetPIDParams`

设置指定电机的 PID 参数。

```c
void OmniVehicleControl_SetPIDParams(
    MotorIndex_e motor_index,
    float kp,
    float ki,
    float kd
);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `motor_index` | `MotorIndex_e` | 电机编号 (0-3) |
| `kp` | `float` | 比例系数 |
| `ki` | `float` | 积分系数 |
| `kd` | `float` | 微分系数 |

**电机编号枚举：**

```c
typedef enum {
    MOTOR_FRONT_LEFT = 0,  // 前左电机
    MOTOR_FRONT_RIGHT,     // 前右电机
    MOTOR_REAR_RIGHT,     // 后右电机
    MOTOR_REAR_LEFT,       // 后左电机
    MOTOR_COUNT            // 电机总数
} MotorIndex_e;
```

**使用示例：**

```c
// 设置前左电机的 PID 参数
OmniVehicleControl_SetPIDParams(MOTOR_FRONT_LEFT, 1.5f, 0.2f, 0.05f);

// 设置所有电机的 PID 参数
OmniVehicleControl_SetPIDParams(MOTOR_FRONT_LEFT, 1.0f, 0.1f, 0.0f);
OmniVehicleControl_SetPIDParams(MOTOR_FRONT_RIGHT, 1.0f, 0.1f, 0.0f);
OmniVehicleControl_SetPIDParams(MOTOR_REAR_RIGHT, 1.0f, 0.1f, 0.0f);
OmniVehicleControl_SetPIDParams(MOTOR_REAR_LEFT, 1.0f, 0.1f, 0.0f);
```

#### `OmniVehicleControl_ResetPID`

复位指定电机的 PID 控制器，清除积分项等状态。

```c
void OmniVehicleControl_ResetPID(MotorIndex_e motor_index);
```

**使用示例：**

```c
// 复位前左电机的 PID 控制器
OmniVehicleControl_ResetPID(MOTOR_FRONT_LEFT);
```

#### `OmniVehicleControl_ResetAllPID`

复位所有电机的 PID 控制器。

```c
void OmniVehicleControl_ResetAllPID(void);
```

**使用示例：**

```c
// 复位所有 PID 控制器（例如在急停或重启时）
OmniVehicleControl_ResetAllPID();
```

---

### 电机控制

#### `OmniVehicleControl_EnableMotor`

使能或失能指定电机。

```c
void OmniVehicleControl_EnableMotor(MotorIndex_e motor_index, bool enable);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `motor_index` | `MotorIndex_e` | 电机编号 (0-3) |
| `enable` | `bool` | true=使能, false=失能 |

**使用示例：**

```c
// 使能前左电机
OmniVehicleControl_EnableMotor(MOTOR_FRONT_LEFT, true);

// 失能前右电机
OmniVehicleControl_EnableMotor(MOTOR_FRONT_RIGHT, false);
```

#### `OmniVehicleControl_EnableAllMotors`

使能或失能所有电机。

```c
void OmniVehicleControl_EnableAllMotors(bool enable);
```

**使用示例：**

```c
// 使能所有电机
OmniVehicleControl_EnableAllMotors(true);

// 失能所有电机（急停）
OmniVehicleControl_EnableAllMotors(false);
```

#### `OmniVehicleControl_SetMotorPWM`

直接设置指定电机的 PWM 占空比，绕过 PID 控制。

```c
void OmniVehicleControl_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `motor_index` | `MotorIndex_e` | 电机编号 (0-3) |
| `duty_cycle` | `float` | PWM 占空比，范围 -100.0 ~ 100.0 |

**使用示例：**

```c
// 设置前左电机 PWM 为 50%
OmniVehicleControl_SetMotorPWM(MOTOR_FRONT_LEFT, 50.0f);

// 设置前右电机 PWM 为 -30%（反转）
OmniVehicleControl_SetMotorPWM(MOTOR_FRONT_RIGHT, -30.0f);

// 停止所有电机
OmniVehicleControl_SetMotorPWM(MOTOR_FRONT_LEFT, 0.0f);
OmniVehicleControl_SetMotorPWM(MOTOR_FRONT_RIGHT, 0.0f);
OmniVehicleControl_SetMotorPWM(MOTOR_REAR_RIGHT, 0.0f);
OmniVehicleControl_SetMotorPWM(MOTOR_REAR_LEFT, 0.0f);
```

---

### 状态查询

#### `OmniVehicleControl_GetWheelSpeed`

获取指定轮子的目标速度（从运动学计算得出）。

```c
float OmniVehicleControl_GetWheelSpeed(MotorIndex_e motor_index);
```

**参数说明：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `motor_index` | `MotorIndex_e` | 电机编号 (0-3) |

**返回值：**

- `float`：目标轮速 (rad/s)

**使用示例：**

```c
// 设置车体速度
VehicleVelocity_t vel = {.vx = 0.5f, .vy = 0.0f, .omega = 0.0f};
OmniVehicleControl_SetVelocity(&vel);

// 查询各轮子的目标速度
float speed_fl = OmniVehicleControl_GetWheelSpeed(MOTOR_FRONT_LEFT);
float speed_fr = OmniVehicleControl_GetWheelSpeed(MOTOR_FRONT_RIGHT);
float speed_rr = OmniVehicleControl_GetWheelSpeed(MOTOR_REAR_RIGHT);
float speed_rl = OmniVehicleControl_GetWheelSpeed(MOTOR_REAR_LEFT);

printf("Front Left: %.2f rad/s\n", speed_fl);
printf("Front Right: %.2f rad/s\n", speed_fr);
printf("Rear Right: %.2f rad/s\n", speed_rr);
printf("Rear Left: %.2f rad/s\n", speed_rl);
```

---

### 资源释放

#### `OmniVehicleControl_Deinit`

销毁全向车控制模块，释放初始化时分配的资源。

```c
void OmniVehicleControl_Deinit(void);
```

**使用示例：**

```c
// 在程序结束或模块不再使用时调用
OmniVehicleControl_Deinit();
```

---

## 使用示例

### 完整示例：基础运动控制

```c
#include "omni_vehicle_control.h"

// 硬件相关函数
// 编码器读取函数（接收电机编号参数）
float read_encoder(uint8_t motor_index) {
    switch (motor_index) {
        case MOTOR_FRONT_LEFT:
            // 读取前左电机编码器，返回速度 (rad/s)
            return (float)TIM4->CNT * 0.001f;
        case MOTOR_FRONT_RIGHT:
            // 读取前右电机编码器，返回速度 (rad/s)
            return (float)TIM5->CNT * 0.001f;
        case MOTOR_REAR_RIGHT:
            // 读取后右电机编码器，返回速度 (rad/s)
            return (float)TIM6->CNT * 0.001f;
        case MOTOR_REAR_LEFT:
            // 读取后左电机编码器，返回速度 (rad/s)
            return (float)TIM7->CNT * 0.001f;
        default:
            return 0.0f;
    }
}

void set_pwm(MotorIndex_e motor, float duty_cycle) {
    // 设置 PWM
    TIM_TypeDef* timers[] = {TIM3, TIM3, TIM3, TIM3};
    uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    
    uint32_t pwm_value = (uint32_t)(abs(duty_cycle) * 10);
    __HAL_TIM_SET_COMPARE(&htim3, channels[motor], pwm_value);
}

void enable_motor(MotorIndex_e motor, bool enable) {
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 + motor, state);
}

int main(void) {
    // 1. 配置几何参数
    VehicleGeometry_t geometry = {
        .wheel_radius = 0.05f,
        .wheelbase_length = 0.2f,
        .wheeltrack_width = 0.2f
    };
    
    // 2. 配置 PID 参数
    float pid_params[MOTOR_COUNT][3] = {
        {1.0f, 0.1f, 0.0f},  // 前左
        {1.0f, 0.1f, 0.0f},  // 前右
        {1.0f, 0.1f, 0.0f},  // 后右
        {1.0f, 0.1f, 0.0f}   // 后左
    };
    
    // 3. 初始化控制模块
    if (OmniVehicleControl_Inst(set_pwm, enable_motor, read_encoder, &geometry, pid_params) != 0) {
        Error_Handler();
    }
    
    // 5. 使能所有电机
    OmniVehicleControl_EnableAllMotors(true);
    
    // 6. 主循环：控制车辆运动
    while (1) {
        // 前进 1 秒
        VehicleVelocity_t vel = {.vx = 0.5f, .vy = 0.0f, .omega = 0.0f};
        OmniVehicleControl_SetVelocity(&vel);
        HAL_Delay(1000);
        
        // 右移 1 秒
        vel = (VehicleVelocity_t){.vx = 0.0f, .vy = 0.3f, .omega = 0.0f};
        OmniVehicleControl_SetVelocity(&vel);
        HAL_Delay(1000);
        
        // 原地旋转 1 秒
        vel = (VehicleVelocity_t){.vx = 0.0f, .vy = 0.0f, .omega = 1.0f};
        OmniVehicleControl_SetVelocity(&vel);
        HAL_Delay(1000);
        
        // 停止 1 秒
        vel = (VehicleVelocity_t){.vx = 0.0f, .vy = 0.0f, .omega = 0.0f};
        OmniVehicleControl_SetVelocity(&vel);
        HAL_Delay(1000);
    }
    
    // 7. 清理（通常不会执行到）
    OmniVehicleControl_Deinit();
}
```

### 示例：动态调整 PID 参数

```c
// 运动过程中调整 PID 参数
void adjust_pid_parameters(void) {
    // 如果发现速度响应慢，增加 Kp
    float current_kp = 1.0f;
    OmniVehicleControl_SetPIDParams(MOTOR_FRONT_LEFT, current_kp + 0.5f, 0.1f, 0.0f);
    
    // 如果发现超调，增加 Kd
    OmniVehicleControl_SetPIDParams(MOTOR_FRONT_RIGHT, 1.0f, 0.1f, 0.1f);
    
    // 如果发现稳态误差，增加 Ki
    OmniVehicleControl_SetPIDParams(MOTOR_REAR_RIGHT, 1.0f, 0.2f, 0.0f);
}
```

### 示例：急停处理

```c
void emergency_stop(void) {
    // 方法 1：设置速度为零
    VehicleVelocity_t vel = {.vx = 0.0f, .vy = 0.0f, .omega = 0.0f};
    OmniVehicleControl_SetVelocity(&vel);
    
    // 方法 2：直接失能电机（更快速）
    OmniVehicleControl_EnableAllMotors(false);
    
    // 复位 PID 控制器
    OmniVehicleControl_ResetAllPID();
}
```

---

## 注意事项

### 1. 初始化顺序
必须先调用 `OmniVehicleControl_Inst` 初始化模块，才能调用其他 API。

### 2. 函数指针回调
- `SetMotorPWM_fn` 必须能够处理 -100.0 ~ 100.0 范围的 duty_cycle
- `EnableMotor_fn` 应该在失能时停止电机输出（例如输出低电平）
- `ReadEncoder_fn` 应该返回电机的实际速度（rad/s），而不是编码器计数值

### 3. PID 参数调优
PID 参数需要根据实际硬件进行调优：
- **Kp（比例）**：响应速度，过大可能导致震荡
- **Ki（积分）**：消除稳态误差，过大可能导致超调
- **Kd（微分）**：抑制震荡，但可能放大噪声

建议调优步骤：
1. 先设置 Kd = 0, Ki = 0，逐步增加 Kp 直到出现轻微震荡
2. 增加少量 Kd 抑制震荡
3. 如果存在稳态误差，逐步增加 Ki

### 4. 速度限制
- 车体速度不应超过电机和机械结构的物理极限
- 建议在应用层进行速度限制，避免设置过大的速度值

### 5. 编码器精度
编码器读取函数应返回高精度的速度值，建议：
- 使用定时器的编码器模式
- 使用浮点数运算（rad/s）
- 定期校准编码器零点

### 6. 安全考虑
- 实现急停功能
- 添加超时保护（例如检测到编码器无响应时停止电机）
- 在初始化时检查电机是否正常响应

### 7. 内存使用
本模块使用全局静态变量存储状态，不占用堆内存，适合嵌入式系统。

### 8. 线程安全
当前实现**不是线程安全的**。如果在多任务环境中使用，请：
- 在 RTOS 中使用互斥锁保护 API 调用
- 或确保只有一个任务调用控制 API

---

## 常见问题

### Q1: 如何调整车辆几何参数？

**A:** 在初始化时修改 `VehicleGeometry_t` 结构体：

```c
VehicleGeometry_t geometry = {
    .wheel_radius = 0.065f,      // 更改轮子半径
    .wheelbase_length = 0.25f,   // 更改轴距
    .wheeltrack_width = 0.25f    // 更改轮距
};
```

### Q2: 电机不转怎么办？

**A:** 检查以下几点：
1. 是否调用了 `OmniVehicleControl_EnableAllMotors(true)`
2. 编码器函数是否返回正确值
3. PWM 设置函数是否正确实现
4. PID 参数是否合理

### Q3: 如何实现闭环速度控制？

**A:** 本模块已经内置闭环速度控制，只需提供正确的编码器读取函数即可。模块会自动将目标轮速（通过运动学计算得出）与实际轮速（通过编码器读取）进行比较，通过 PID 算法调整 PWM。

### Q4: 可以同时使用 `SetMotorPWM` 和 `SetVelocity` 吗？

**A:** 不建议。`SetMotorPWM` 会绕过 PID 控制直接设置 PWM，与 `SetVelocity` 的闭环控制冲突。建议统一使用 `SetVelocity` 进行速度控制。

---

## 附录：数据类型定义

```c
// 电机编号枚举
typedef enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_COUNT
} MotorIndex_e;

// 车辆几何参数
typedef struct {
    float wheel_radius;      // 轮子半径 (m)
    float wheelbase_length;  // 轴距 (m)
    float wheeltrack_width;  // 轮距 (m)
} VehicleGeometry_t;

// 车体速度
typedef struct {
    float vx;     // X 轴线速度 (m/s)
    float vy;     // Y 轴线速度 (m/s)
    float omega;  // 旋转角速度 (rad/s)
} VehicleVelocity_t;

// 轮速数组
typedef struct {
    float wheel_speeds[MOTOR_COUNT];  // 4个轮子的速度 (rad/s)
} WheelSpeeds_t;
```

---

## 更新日志

### v1.0.0 (2026-01-18)
- 初始版本
- 实现基本的运动控制 API
- 支持 PID 参数调整
- 支持电机使能/失能

---

## 技术支持

如有问题或建议，请联系项目维护者或提交 Issue。
