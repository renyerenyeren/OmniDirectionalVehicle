cmake_minimum_required(VERSION 3.10)

# 定义 ODV 驱动库
add_library(odv_lib STATIC)

# 添加 ODV 模块的源文件
target_sources(odv_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mecanum_kinematics.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/omni_motor_driver_interface.c
)

# 暴露 ODV 模块的头文件路径
target_include_directories(odv_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（根据实际需求添加，例如硬件驱动库）
target_link_libraries(odv_lib PRIVATE
        STM32_Drivers
)
