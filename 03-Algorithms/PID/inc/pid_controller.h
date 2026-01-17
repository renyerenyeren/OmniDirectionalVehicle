/**
 ******************************************************************************
 * @file           : pid_controller.h
 * @brief          : PID控制器接口定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了PID控制器的抽象接口
 * 使用C语言模拟面向对象的设计模式
 * 支持多种PID算法（标准PID、积分分离PID、增量式PID等）
 * 通过依赖注入方式接收编码器读取函数
 * 对外暴露SpeedToPWM接口，内部调用PID计算
 ******************************************************************************
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include <stdbool.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Typedefs *********************************//
/**
 * @brief 编码器读取函数指针类型
 * @return 实际轮速
 */
typedef float (*ReadEncoder_fn)(void);

/**
 * @brief SpeedToPWM函数指针类型
 * @note 与电机驱动接口保持一致
 * @param target_speed_rad_s 目标轮速
 * @return PWM占空比 (-100.0 ~ 100.0)
 */
typedef float (*SpeedToPWM_pid)(void* self,float target_speed_rad_s);

//******************************** Typedefs *********************************//
//---------------------------------------------------------------------------//
//**************************** Interface Structs ****************************//
/**
 * @brief PID控制器内部方法结构体（虚函数表）
 * @note 将内部函数封装成独立结构体，进一步解耦
 *       所有同类型PID实例共享同一个方法表（节省内存）
 */
typedef struct PID_Methods {
    /* PID计算方法 */
    float (*calculate)(void* self, float target, float current);
    
    /* 参数设置方法 */
    void (*set_params)(void* self, float kp, float ki, float kd);
    
    /* 复位方法 */
    void (*reset)(void* self);
} PID_Methods_t;
//**************************** Interface Structs ****************************//
//---------------------------------------------------------------------------//
//******************************** Classes **********************************//
/**
 * @brief PID控制器类（结构体）
 */
typedef struct IPIDController {
    /* 外部依赖注入 */
    ReadEncoder_fn read_encoder;  /**< 编码器读取函数指针 */

    /* 对外暴露的接口 */
    SpeedToPWM_pid speed_to_pwm;  /**< 速度转PWM接口（对外暴露） */

    /* 内部方法（进一步封装）*/
    const PID_Methods_t* methods;  /**< 内部方法指针（只读） */

    /* 私有数据（PID状态）*/
    void* private_data;  /**< 指向具体PID算法的私有数据 */
} IPIDController_t;

//******************************** Classes **********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//
/**
 * @brief PID控制器实例化函数（inst）
 * @param pid PID控制器对象指针
 * @param read_encoder 编码器读取函数指针（外部依赖注入）
 * @param pid_type PID算法类型（0=标准PID, 1=积分分离PID, 2=增量式PID）
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 * @return 0=成功, -1=失败
 * 
 * @note 此函数完成以下工作：
 *       1. 注入外部依赖（编码器读取函数）
 *       2. 初始化PID私有数据
 *       3. 绑定内部方法表（methods）
 *       4. 绑定对外暴露的接口（speed_to_pwm）
 */
int PID_Inst(IPIDController_t* pid,
           ReadEncoder_fn read_encoder,
           int pid_type,
           float kp, float ki, float kd);

//******************************** Functions ********************************//

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
