# Implementation Plan

将 OmniDirectionalVehicle 模块重构为两个独立的模块：OmniSolve（全向解算）和 MotorDriver（硬件抽象），以提高代码模块化和可维护性。

本次重构旨在将现有 OmniDirectionalVehicle 模块按照功能职责进行拆分。OmniSolve 模块负责纯数学计算（运动学解算），MotorDriver 模块负责硬件抽象（电机驱动接口）。这种分离遵循单一职责原则，使得各模块职责更加清晰，便于后续维护和扩展。重构后，OmniVehicleControl 模块将同时依赖这两个模块，保持原有的API接口不变。

[Types]

不涉及新的类型系统变更，主要是在现有类型之间进行重新组织和模块划分。

**OmniSolve 模块（全向解算）**:

- **MotorIndex_e** (从 omni_types.h 移动)
  - 类型：枚举
  - 值：MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_COUNT
  - 用途：标识四个电机

- **VehicleVelocity_t** (从 omni_types.h 移动)
  - 类型：结构体
  - 字段：float vx, float vy, float omega
  - 用途：车体速度向量

- **WheelSpeeds_t** (从 omni_types.h 移动)
  - 类型：结构体
  - 字段：float wheel_speeds[MOTOR_COUNT]
  - 用途：四个轮子的角速度

- **VehicleGeometry_t** (从 omni_types.h 移动)
  - 类型：结构体
  - 字段：float wheel_base, float track_width, float wheel_radius
  - 用途：车辆几何参数

- **宏定义** (从 omni_types.h 移动)
  - DEFAULT_WHEEL_BASE, DEFAULT_TRACK_WIDTH, DEFAULT_WHEEL_RADIUS
  - MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, MAX_WHEEL_SPEED
  - CLAMP, DEG_TO_RAD, RAD_TO_DEG

**MotorDriver 模块（硬件抽象）**:

无新增类型，仅依赖 OmniSolve 模块的 MotorIndex_e 类型。

[Files]

创建新模块目录并重新组织文件，更新所有相关引用。

**新建文件**:

1. **03-Algorithms/OmniSolve/inc/omni_solve_types.h**
   - 从 omni_types.h 移动所有类型定义和宏定义
   - 包含 MotorIndex_e, VehicleVelocity_t, WheelSpeeds_t, VehicleGeometry_t
   - 包含所有宏定义

2. **03-Algorithms/OmniSolve/inc/mecanum_kinematics.h**
   - 从 OmniDirectionalVehicle/inc/ 移动
   - 修改 include 从 "omni_types.h" 改为 "omni_solve_types.h"

3. **03-Algorithms/OmniSolve/src/mecanum_kinematics.c**
   - 从 OmniDirectionalVehicle/src/ 移动
   - 修改 include 从 "omni_types.h" 改为 "omni_solve_types.h"

4. **03-Algorithms/OmniSolve/CMakeLists.txt**
   - 新建 CMake 构建文件
   - 定义 omni_solve_lib 静态库

5. **03-Algorithms/MotorDriver/inc/motor_driver_interface.h**
   - 从 OmniDirectionalVehicle/inc/omni_motor_driver_interface.h 移动并重命名
   - 修改 include 从 "omni_types.h" 改为 "../OmniSolve/inc/omni_solve_types.h"
   - 移除对 mecanum_kinematics.h 的依赖（仅在 .c 文件中需要）

6. **03-Algorithms/MotorDriver/src/motor_driver_interface.c**
   - 从 OmniDirectionalVehicle/src/omni_motor_driver_interface.c 移动并重命名
   - 添加 include "mecanum_kinematics.h"
   - 修改 include 路径为 "../OmniSolve/inc/omni_solve_types.h"

7. **03-Algorithms/MotorDriver/CMakeLists.txt**
   - 新建 CMake 构建文件
   - 定义 motor_driver_lib 静态库
   - 链接 omni_solve_lib

**修改文件**:

1. **03-Algorithms/OmniVehicleControl/inc/omni_vehicle_control.h**
   - 修改 include 从 "omni_types.h" 改为 "omni_solve_types.h"
   - 修改 include 从 "omni_motor_driver_interface.h" 改为 "motor_driver_interface.h"
   - 修改 include 从 "mecanum_kinematics.h" 改为 "mecanum_kinematics.h"（路径不变）

2. **03-Algorithms/OmniVehicleControl/src/omni_vehicle_control.c**
   - 修改 include 从 "omni_types.h" 改为 "omni_solve_types.h"
   - 修改 include 从 "omni_motor_driver_interface.h" 改为 "motor_driver_interface.h"

3. **03-Algorithms/CMakeLists.txt**
   - 移除 add_subdirectory(OmniDirectionalVehicle)
   - 添加 add_subdirectory(OmniSolve)
   - 添加 add_subdirectory(MotorDriver)
   - 更新 target_link_libraries 中的依赖库名称

4. **03-Algorithms/OmniVehicleControl/CMakeLists.txt**
   - 更新 target_link_libraries：odv_lib 改为 motor_driver_lib

**删除文件**:

1. **03-Algorithms/OmniDirectionalVehicle/** (整个目录)
   - 删除整个目录及其下所有文件

[Functions]

主要更新函数的 include 头文件，函数签名和实现逻辑保持不变。

**修改函数**:

所有在以下文件中的函数需要更新 include 头文件：
- OmniSolve/inc/mecanum_kinematics.h 中的函数声明
- OmniSolve/src/mecanum_kinematics.c 中的函数实现
- MotorDriver/inc/motor_driver_interface.h 中的函数声明
- MotorDriver/src/motor_driver_interface.c 中的函数实现
- OmniVehicleControl/inc/omni_vehicle_control.h 中的函数声明
- OmniVehicleControl/src/omni_vehicle_control.c 中的函数实现

**新增函数**:
无新增函数。

**删除函数**:
无删除函数。

[Classes]

不涉及类结构变更，仅更新 include 依赖。

**修改类**:

- **IMecanumKinematics_t** (在 mecanum_kinematics.h 中)
  - 结构体定义不变
  - 更新 include 从 "omni_types.h" 改为 "omni_solve_types.h"

- **IOmniMotorDriver_t** (在 motor_driver_interface.h 中)
  - 结构体定义不变
  - 更新 include 从 "omni_types.h" 改为 "../OmniSolve/inc/omni_solve_types.h"

**新增类**:
无新增类。

**删除类**:
无删除类。

[Dependencies]

更新模块间的依赖关系。

**新依赖**:
- OmniSolve 模块：无外部依赖（仅标准库）
- MotorDriver 模块：依赖 OmniSolve 模块

**依赖变更**:
- 原 OmniDirectionalVehicle 模块 (odv_lib) 拆分为：
  - OmniSolve 模块 (omni_solve_lib)
  - MotorDriver 模块 (motor_driver_lib)
- OmniVehicleControl 模块依赖更新：
  - 原：odv_lib, velocity_pid_lib
  - 新：omni_solve_lib, motor_driver_lib, velocity_pid_lib

**版本要求**:
无新增第三方依赖，CMake 版本要求保持 3.10。

[Testing]

确保重构后所有模块正常编译和链接，功能保持不变。

**编译测试**:
1. 验证 OmniSolve 模块独立编译成功
2. 验证 MotorDriver 模块独立编译成功
3. 验证 OmniVehicleControl 模块编译成功
4. 验证整个项目构建成功

**功能测试**:
1. 验证运动学计算功能正常（正逆运动学）
2. 验证电机驱动接口功能正常
3. 验证 OmniVehicleControl API 功能正常
4. 验证 PID 集成功能正常

**回归测试**:
确保所有现有测试用例（如果有）通过，无功能退化。

[Implementation Order]

按照依赖关系从底向上逐步实施重构，确保每一步都可以独立验证。

1. 创建 OmniSolve 模块目录结构和 CMakeLists.txt
2. 创建 omni_solve_types.h 文件（从 omni_types.h 移动内容）
3. 移动 mecanum_kinematics.h/c 到 OmniSolve 模块并更新 include
4. 创建 MotorDriver 模块目录结构和 CMakeLists.txt
5. 移动并重命名电机驱动接口文件到 MotorDriver 模块，更新 include
6. 更新 OmniVehicleControl 模块的 include 头文件
7. 更新 03-Algorithms/CMakeLists.txt 以引用新模块
8. 更新 OmniVehicleControl/CMakeLists.txt 的依赖
9. 删除原 OmniDirectionalVehicle 目录
10. 执行完整构建测试
11. 验证功能完整性
