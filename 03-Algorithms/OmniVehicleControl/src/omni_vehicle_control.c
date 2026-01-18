/**
 ******************************************************************************
 * @file           : omni_vehicle_control.c
 * @brief          : 全向车控制模块实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-18
 ******************************************************************************
 * @description
 * 本文件实现了全向车控制模块的统一接口
 * 封装了运动学、电机驱动、PID控制器
 * 提供简单的set_velocity API
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "omni_vehicle_control.h"
#include <stdlib.h>
#include <string.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//****************************** Global Variables ****************************//
/* 全局变量 - 运动学、电机驱动、4个PID控制器 */
static IMecanumKinematics_t g_kinematics;
static IOmniMotorDriver_t g_motor_driver;
static IPIDController_t g_pid_controllers[MOTOR_COUNT];

/* 轮速计算结果缓存 */
static WheelSpeeds_t g_wheel_speeds;

//***************************** Global Variables ****************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//
/**
 * @brief 实例化全向车控制模块（inst）
 * @param set_pwm PWM设置函数指针
 * @param enable_motor 电机使能函数指针
 * @param read_encoders 编码器读取函数数组（4个电机）
 * @param geometry 车辆几何参数指针
 * @param pid_params PID参数数组[MOTOR_COUNT][3]，每个元素为{kp, ki, kd}
 * @return 0=成功, -1=失败
 * 
 * @note 此函数内部完成：
 *       1. 调用MecanumKinematics_Init初始化全局运动学对象
 *       2. 调用OmniMotorDriver_Init初始化全局电机驱动对象
 *       3. 调用PID_Inst初始化4个全局PID控制器
 */
int OmniVehicleControl_Inst(SetMotorPWM_fn set_pwm,
                            EnableMotor_fn enable_motor,
                            ReadEncoder_fn read_encoders[MOTOR_COUNT],
                            const VehicleGeometry_t* geometry,
                            const float pid_params[MOTOR_COUNT][3])
{
    /* 参数检查 */
    if (set_pwm == NULL || enable_motor == NULL ||
        geometry == NULL || pid_params == NULL)
    {
        return -1;
    }
    
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (read_encoders[i] == NULL)
        {
            return -1;
        }
    }
    
    /* 清零全局变量 */
    memset(&g_kinematics, 0, sizeof(IMecanumKinematics_t));
    memset(&g_motor_driver, 0, sizeof(IOmniMotorDriver_t));
    memset(g_pid_controllers, 0, sizeof(g_pid_controllers));
    memset(&g_wheel_speeds, 0, sizeof(WheelSpeeds_t));
    
    /* 1. 调用MecanumKinematics_Init初始化全局运动学对象 */
    if (MecanumKinematics_Init(&g_kinematics, geometry) != 0)
    {
        return -1;
    }
    
    /* 2. 调用OmniMotorDriver_Init初始化全局电机驱动对象 */
    if (OmniMotorDriver_Init(&g_motor_driver, 
                             set_pwm, 
                             enable_motor, 
                             NULL) != 0)  /* speed_convert稍后绑定 */
    {
        return -1;
    }
    
    /* 3. 调用PID_Inst初始化4个全局PID控制器 */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (PID_Inst(&g_pid_controllers[i], 
                     read_encoders[i],  // 注入编码器函数
                     0,  // PID类型：0=标准PID
                     pid_params[i][0],  // kp
                     pid_params[i][1],  // ki
                     pid_params[i][2]) != 0)  // kd
        {
            return -1;
        }
    }
    
    /* 4. 绑定speed_to_pwm接口 - 使用第一个PID控制器的speed_to_pwm接口 */
    /* 注意：由于所有电机共享同一个speed_to_pwm函数（通过motor_index区分），
          所以只需要绑定第一个即可 */
    if (g_pid_controllers[0].speed_to_pwm != NULL)
    {
        g_motor_driver.speed_to_pwm = (SpeedToPWM_fn)g_pid_controllers[0].speed_to_pwm;
    }
    
    return 0;
}

/**
 * @brief 设置车体速度
 * @param vehicle_vel 车体速度指针
 * 
 * @note 内部流程：
 *       1. 调用运动学逆解：VehicleVelocity -> WheelSpeeds
 *       2. 调用PID控制器：WheelSpeed -> PWM
 *       3. 调用电机驱动：设置PWM
 */
void OmniVehicleControl_SetVelocity(const VehicleVelocity_t* vehicle_vel)
{
    if (vehicle_vel == NULL)
    {
        return;
    }
    
    /* 1. 调用运动学逆解：VehicleVelocity -> WheelSpeeds */
    g_kinematics.calculate_inverse_kinematics(&g_kinematics, 
                                                  vehicle_vel, 
                                                  &g_wheel_speeds);
    
    /* 2. 调用PID控制器：WheelSpeed -> PWM */
    float pwm[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        /* 注意：需要确保g_motor_driver.speed_to_pwm已经被正确绑定 */
        if (g_motor_driver.speed_to_pwm != NULL)
        {
            pwm[i] = g_motor_driver.speed_to_pwm(
                &g_pid_controllers[i], 
                g_wheel_speeds.wheel_speeds[i]);
        }
    }
    
    /* 3. 调用电机驱动：设置PWM */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (g_motor_driver.set_motor_pwm != NULL)
        {
            g_motor_driver.set_motor_pwm(i, pwm[i]);
        }
    }
}

/**
 * @brief 设置指定电机的PID参数
 * @param motor_index 电机编号
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
void OmniVehicleControl_SetPIDParams(MotorIndex_e motor_index, 
                                   float kp, float ki, float kd)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }
    
    /* 调用PID模块的参数设置接口 */
    PID_SetParams(&g_pid_controllers[motor_index], kp, ki, kd);
}

/**
 * @brief 复位指定电机的PID控制器
 * @param motor_index 电机编号
 */
void OmniVehicleControl_ResetPID(MotorIndex_e motor_index)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }
    
    /* 调用PID模块的复位接口 */
    if (g_pid_controllers[motor_index].methods != NULL &&
        g_pid_controllers[motor_index].methods->reset != NULL)
    {
        g_pid_controllers[motor_index].methods->reset(
            g_pid_controllers[motor_index].private_data);
    }
}

/**
 * @brief 复位所有PID控制器
 */
void OmniVehicleControl_ResetAllPID(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        OmniVehicleControl_ResetPID((MotorIndex_e)i);
    }
}

/**
 * @brief 使能指定电机
 * @param motor_index 电机编号
 * @param enable 使能标志
 */
void OmniVehicleControl_EnableMotor(MotorIndex_e motor_index, bool enable)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }
    
    /* 调用电机驱动的使能接口 */
    if (g_motor_driver.enable_motor != NULL)
    {
        g_motor_driver.enable_motor(motor_index, enable);
    }
}

/**
 * @brief 使能所有电机
 * @param enable 使能标志
 */
void OmniVehicleControl_EnableAllMotors(bool enable)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        OmniVehicleControl_EnableMotor((MotorIndex_e)i, enable);
    }
}

/**
 * @brief 设置电机PWM（直接调用电机驱动接口）
 * @param motor_index 电机编号
 * @param duty_cycle PWM占空比 (-100.0 ~ 100.0)
 */
void OmniVehicleControl_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }
    
    /* 调用电机驱动的PWM设置接口 */
    if (g_motor_driver.set_motor_pwm != NULL)
    {
        g_motor_driver.set_motor_pwm(motor_index, duty_cycle);
    }
}

/**
 * @brief 获取指定轮速
 * @param motor_index 电机编号
 * @return 轮速（rad/s）
 */
float OmniVehicleControl_GetWheelSpeed(MotorIndex_e motor_index)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return 0.0f;
    }
    
    /* 返回目标轮速（从运动学计算结果）*/
    return g_wheel_speeds.wheel_speeds[motor_index];
}

/**
 * @brief 销毁全向车控制模块
 * @note 释放inst函数分配的资源
 */
void OmniVehicleControl_Deinit(void)
{
    /* 清零全局变量 */
    memset(&g_kinematics, 0, sizeof(IMecanumKinematics_t));
    memset(&g_motor_driver, 0, sizeof(IOmniMotorDriver_t));
    memset(g_pid_controllers, 0, sizeof(g_pid_controllers));
    memset(&g_wheel_speeds, 0, sizeof(WheelSpeeds_t));
    
    /* 运动学实现对象不需要释放（由mecanum_kinematics模块管理）*/
}

//******************************** Functions ********************************//
