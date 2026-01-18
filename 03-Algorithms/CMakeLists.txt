cmake_minimum_required(VERSION 3.10)

# 引入 OmniSolve 子模块（全向解算）
add_subdirectory(OmniSolve)
# 引入 MotorDriver 子模块（硬件抽象）
add_subdirectory(MotorDriver)
# 引入 Velocity_PID 子模块
add_subdirectory(Velocity_PID)
# 引入 OmniVehicleControl 子模块
add_subdirectory(OmniVehicleControl)

# ------------------------------
# 定义 Algorithms 接口库（用于聚合所有 Algorithms 组件）
# ------------------------------
# 使用 INTERFACE 关键字，表明这是一个接口库
add_library(algorithms INTERFACE)

# 任何链接了 "algorithms" 的目标，都会自动链接
target_link_libraries(algorithms INTERFACE
        omni_solve_lib
        motor_driver_lib
        velocity_pid_lib
        vehicle_control_lib

)

# 暴露 Algorithms 公共头文件路径（如果 Algorithms 目录下有顶层的公共头文件）
 target_include_directories(algorithms INTERFACE
     ${CMAKE_CURRENT_SOURCE_DIR}/inc
 )
