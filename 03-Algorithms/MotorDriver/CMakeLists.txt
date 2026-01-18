cmake_minimum_required(VERSION 3.10)

# 定义 MotorDriver 库（硬件抽象模块）
add_library(motor_driver_lib STATIC)

# 添加 MotorDriver 模块的源文件
target_sources(motor_driver_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/motor_driver_interface.c
)

# 暴露 MotorDriver 模块的头文件路径
target_include_directories(motor_driver_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（OmniSolve 模块）
target_link_libraries(motor_driver_lib PRIVATE
        omni_solve_lib
)
