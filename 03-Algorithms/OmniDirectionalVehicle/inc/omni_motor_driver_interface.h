/**
 ******************************************************************************
 * @file           : omni_motor_driver_interface.h
 * @brief          : 电机驱动接口定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了电机驱动的抽象接口，通过函数指针实现依赖注入
 * 遵循单一职责原则，仅定义电机控制相关的接口
 ******************************************************************************
 */

#ifndef OMNI_MOTOR_DRIVER_INTERFACE_H
#define OMNI_MOTOR_DRIVER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include "omni_types.h"
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Typedefs *********************************//
/**
 * @brief PWM设置函数指针类型
 * @param motor_index 电机编号
 * @param duty_cycle PWM占空比 (-100.0 ~ 100.0, 负值表示反转)
 */
typedef void (*SetMotorPWM_fn)(MotorIndex_e motor_index, float duty_cycle);

/**
 * @brief PWM使能函数指针类型
 * @param motor_index 电机编号
 * @param enable 使能标志 (true=使能, false=失能)
 */
typedef void (*EnableMotor_fn)(MotorIndex_e motor_index, bool enable);

/**
 * @brief 速度转PWM转换函数指针类型
 * @note 这里注入你完成的PID算法接口，实现轮速到PWM的转换
 * @param speed_rad_s 轮速 (rad/s)
 * @return PWM占空比 (-100.0 ~ 100.0)
 */
typedef float (*SpeedToPWM_fn)(float speed_rad_s);

//******************************** Typedefs *********************************//
//---------------------------------------------------------------------------//
//**************************** Interface Structs ****************************//
/**
 * @brief 电机驱动接口结构体
 * @note 使用函数指针实现虚函数表，支持依赖注入
 */
typedef struct IOmniMotorDriver {
    /* 函数指针(虚函数表) */
    SetMotorPWM_fn set_motor_pwm;       /**< 设置电机PWM占空比 */
    EnableMotor_fn enable_motor;        /**< 使能/失能电机 */
    SpeedToPWM_fn speed_to_pwm;         /**< 速度转PWM转换 */

    /* 私有数据指针 */
    void* private_data;                /**< 子类私有数据指针 */
} IOmniMotorDriver_t;

//**************************** Interface Structs ****************************//
//---------------------------------------------------------------------------//
//******************************** 函数声明 ***********************************//
/**
 * @brief 初始化电机驱动接口
 * @param driver 电机驱动接口指针
 * @param set_pwm PWM设置函数指针
 * @param enable 使能函数指针
 * @param speed_convert 速度转换函数指针
 * @return 0=成功, -1=失败
 */
int OmniMotorDriver_Init(IOmniMotorDriver_t* driver, 
                      SetMotorPWM_fn set_pwm, 
                      EnableMotor_fn enable,
                      SpeedToPWM_fn speed_convert);
//******************************** 函数声明 ***********************************//

#ifdef __cplusplus
}
#endif

#endif /* OMNI_MOTOR_DRIVER_INTERFACE_H */
