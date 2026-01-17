cmake_minimum_required(VERSION 3.10)

# 定义 PID 驱动库
add_library(pid_lib STATIC)

# 添加 PID 模块的源文件
target_sources(pid_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pid_controller.c
)

# 暴露 PID 模块的头文件路径
target_include_directories(pid_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（根据实际需求添加，例如硬件驱动库）
target_link_libraries(pid_lib PRIVATE
        STM32_Drivers
)