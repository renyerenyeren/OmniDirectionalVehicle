/**
 ******************************************************************************
 * @file           : omni_motor_driver_interface.c
 * @brief          : 电机驱动接口实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了电机驱动接口的封装函数
 * 通过函数指针实现依赖注入
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "omni_motor_driver_interface.h"
#include <stddef.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//

/**
 * @brief 初始化电机驱动接口
 */
int OmniMotorDriver_Init(IOmniMotorDriver_t* driver, 
                      SetMotorPWM_fn set_pwm, 
                      EnableMotor_fn enable,
                      SpeedToPWM_fn speed_convert)
{
    if (driver == NULL)
    {
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
//******************************** Functions ********************************//