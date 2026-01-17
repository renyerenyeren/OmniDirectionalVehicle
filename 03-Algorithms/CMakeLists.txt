cmake_minimum_required(VERSION 3.10)

# 引入 ODV 子模块
add_subdirectory(OmniDirectionalVehicle)

# ------------------------------
# 定义 Algorithms 接口库（用于聚合所有 Algorithms 组件）
# ------------------------------
# 使用 INTERFACE 关键字，表明这是一个接口库
add_library(algorithms INTERFACE)

# 链接 ODV 模块（公开链接）
# 任何链接了 "bsp" 的目标，都会自动链接
target_link_libraries(algorithms INTERFACE
        odv_lib

)

# 暴露 BSP 公共头文件路径（如果 BSP 目录下有顶层的公共头文件）
 target_include_directories(algorithms INTERFACE
     ${CMAKE_CURRENT_SOURCE_DIR}/inc
 )