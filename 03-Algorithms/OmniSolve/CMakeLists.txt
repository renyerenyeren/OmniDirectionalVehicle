cmake_minimum_required(VERSION 3.10)

# 定义 OmniSolve 库（全向解算模块）
add_library(omni_solve_lib STATIC)

# 添加 OmniSolve 模块的源文件
target_sources(omni_solve_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mecanum_kinematics.c
)

# 暴露 OmniSolve 模块的头文件路径
target_include_directories(omni_solve_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（根据实际需求添加，例如硬件驱动库）
target_link_libraries(omni_solve_lib PRIVATE

)
