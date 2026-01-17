/**
 ******************************************************************************
 * @file           : omni_motor_driver_interface.c
 * @brief          : 电机驱动接口实现
 * @author         : Your Name
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了电机驱动接口的封装函数
 * 通过函数指针实现依赖注入
 ******************************************************************************
 */

//*****************************************************************************//
//******************************** Includes **********************************//
//*****************************************************************************//
#include "omni_motor_driver_interface.h"
#include <stddef.h>

//*****************************************************************************//
//******************************** Defines ***********************************//
//*****************************************************************************//

//*****************************************************************************//
//******************************** Macros ************************************//
//*****************************************************************************//

//*****************************************************************************//
//****************************** 内部辅助函数声明 ******************************//
//*****************************************************************************//

//*****************************************************************************//
//******************************** Variables *********************************//
//*****************************************************************************//

//*****************************************************************************//
//******************************** Functions ********************************//
//*****************************************************************************//

/**
 * @brief 初始化电机驱动接口
 */
int OmniMotorDriver_Init(IOmniMotorDriver_t* driver, 
                      SetMotorPWM_fn set_pwm, 
                      EnableMotor_fn enable,
                      SpeedToPWM_fn speed_convert)
{
    if (driver == NULL) {
        return -1;
    }
    
    /* 注入函数指针 */
    driver->set_motor_pwm = set_pwm;
    driver->enable_motor = enable;
    driver->speed_to_pwm = speed_convert;
    
    /* 初始化私有数据指针 */
    driver->private_data = NULL;
    
    return 0;
}

/**
 * @brief 设置电机PWM占空比
 */
void OmniMotorDriver_SetPWM(IOmniMotorDriver_t* driver, 
                          MotorIndex_e motor_index, 
                          float duty_cycle)
{
    if (driver == NULL) {
        return;
    }
    
    /* 检查函数指针是否有效 */
    if (driver->set_motor_pwm != NULL) {
        driver->set_motor_pwm(motor_index, duty_cycle);
    }
}

/**
 * @brief 使能/失能电机
 */
void OmniMotorDriver_Enable(IOmniMotorDriver_t* driver,
                         MotorIndex_e motor_index,
                         bool enable)
{
    if (driver == NULL) {
        return;
    }
    
    /* 检查函数指针是否有效 */
    if (driver->enable_motor != NULL) {
        driver->enable_motor(motor_index, enable);
    }
}

/**
 * @brief 使能所有电机
 */
void OmniMotorDriver_EnableAll(IOmniMotorDriver_t* driver, bool enable)
{
    if (driver == NULL) {
        return;
    }
    
    /* 使能所有4个电机 */
    for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++) {
        OmniMotorDriver_Enable(driver, i, enable);
    }
}

/**
 * @brief 将轮速转换为PWM占空比
 */
float OmniMotorDriver_SpeedToPWM(IOmniMotorDriver_t* driver, float speed_rad_s)
{
    if (driver == NULL) {
        return 0.0f;
    }
    
    /* 检查函数指针是否有效 */
    if (driver->speed_to_pwm != NULL) {
        return driver->speed_to_pwm(speed_rad_s);
    }
    
    return 0.0f;
}
