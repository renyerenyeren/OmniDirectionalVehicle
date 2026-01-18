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

/**
 * @brief 电机驱动控制函数
 * @details 该函数实现了全向移动车辆的电机控制流程，包括运动学计算和电机PWM控制
 * @param driver 电机驱动接口指针，用于调用电机控制相关函数
 * @param kinematics 运动学接口指针，用于计算轮子目标转速
 * @param pid_handler PID控制器数组指针，包含4个电机的PID控制器
 * @param vehicle_vel 车辆目标速度，包含线速度和角速度
 * @note 函数执行流程：
 *       1. 检查输入参数有效性
 *       2. 通过逆运动学计算四个轮子的目标转速
 *       3. 对每个电机，使用PID控制器将转速转换为PWM
 *       4. 将PWM值输出到对应的电机
 */
void OmniMotorDriver_Control(IOmniMotorDriver_t* driver,
                         IMecanumKinematics_t* kinematics,
                         const void* pid_handler[MOTOR_COUNT],
                         const VehicleVelocity_t* vehicle_vel)
{
    // 函数实现保持不变
    if (driver == NULL || kinematics == NULL || pid_handler == NULL || vehicle_vel == NULL)
    {
        return;
    }

    /* 1. 使用运动学计算轮速（逆运动学）*/
    WheelSpeeds_t wheel_speeds;
    kinematics->calculate_inverse_kinematics(kinematics, vehicle_vel, &wheel_speeds);

    /* 2. 遍历4个电机 */
    for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++)
    {
        /* 3. 速度转PWM（调用PID控制器）*/
        float pwm = driver->speed_to_pwm((void*)pid_handler[i], wheel_speeds.wheel_speeds[i]);

        /* 4. 设置电机PWM */
        driver->set_motor_pwm(i, pwm);
    }
}
//******************************** Functions ********************************//
