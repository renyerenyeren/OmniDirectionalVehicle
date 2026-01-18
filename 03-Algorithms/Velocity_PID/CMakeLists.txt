cmake_minimum_required(VERSION 3.10)

# 定义 Velocity_PID 驱动库
add_library(velocity_pid_lib STATIC)

# 添加 Velocity_PID 模块的源文件
target_sources(velocity_pid_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pid_controller.c
)

# 暴露 Velocity_PID 模块的头文件路径
target_include_directories(velocity_pid_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（根据实际需求添加，例如硬件驱动库）
target_link_libraries(velocity_pid_lib PRIVATE

)