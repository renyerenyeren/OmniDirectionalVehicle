cmake_minimum_required(VERSION 3.10)

# 定义 OmniVehicleControl 驱动库
add_library(vehicle_control_lib STATIC)

# 添加 OmniVehicleControl 模块的源文件
target_sources(vehicle_control_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/omni_vehicle_control.c
)

# 暴露 OmniVehicleControl 模块的头文件路径
target_include_directories(vehicle_control_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 链接依赖库（根据实际需求添加，例如硬件驱动库）
target_link_libraries(vehicle_control_lib PRIVATE
        STM32_Drivers
        omni_solve_lib
        motor_driver_lib
        velocity_pid_lib
)