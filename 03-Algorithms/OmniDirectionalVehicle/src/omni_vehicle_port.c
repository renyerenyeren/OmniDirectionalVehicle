/**
 ******************************************************************************
 * @file           : omni_vehicle_port.c
 * @brief          : 依赖注入端口实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了依赖注入端口，管理所有对象的创建和依赖绑定
 * 遵循工厂模式和门面模式
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "omni_vehicle_port.h"
#include <string.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//
/**
 * @brief 初始化依赖注入端口
 * 
 * 此函数完成以下工作：
 * 1. 初始化车辆几何参数
 * 2. 创建运动学对象
 * 3. 初始化电机驱动接口结构体
 * 4. 创建控制器对象
 * 5. 建立对象间的依赖关系
 */
int OmniVehiclePort_Init(OmniVehiclePort_t* port,
                        const VehicleGeometry_t* geometry)
{
    if (port == NULL) {
        return -1;
    }
    
    /* 1. 初始化车辆几何参数 */
    if (geometry == NULL) {
        /* 使用默认值 */
        port->geometry.wheel_base = DEFAULT_WHEEL_BASE;
        port->geometry.track_width = DEFAULT_TRACK_WIDTH;
        port->geometry.wheel_radius = DEFAULT_WHEEL_RADIUS;
    } else {
        port->geometry = *geometry;
    }
    
    /* 2. 创建运动学对象 - 传递geometry的地址 */
    if (MecanumKinematics_Init(&port->kinematics, &port->geometry) != 0) {
        return -1;
    }
    
    /* 3. 初始化电机驱动接口结构体 */
    /* 注意：此时函数指针还未注入，先初始化为NULL */
    if (OmniMotorDriver_Init(&port->motor_driver, NULL, NULL, NULL) != 0) {
        return -1;
    }
    
    /* 4. 创建控制器对象 */
    /* 建立依赖关系：控制器依赖运动学和电机驱动 */
    if (OmniVehicleController_Init(&port->controller, 
                                   &port->kinematics, 
                                   &port->motor_driver) != 0) {
        return -1;
    }
    
    /* 5. 初始化函数指针占位 */
    port->pwm_set_fn = NULL;
    port->pwm_enable_fn = NULL;
    port->speed_convert_fn = NULL;
    
    return 0;
}

/**
 * @brief 注入PWM底层驱动函数
 * 
 * 必须在Init之后调用
 * 将BSP层的PWM驱动函数注入到算法层
 */
int OmniVehiclePort_InjectPWMDriver(OmniVehiclePort_t* port,
                                    SetMotorPWM_fn set_pwm,
                                    EnableMotor_fn enable,
                                    SpeedToPWM_fn speed_convert)
{
    if (port == NULL) {
        return -1;
    }
    
    /* 检查函数指针是否有效 */
    if (set_pwm == NULL || enable == NULL || speed_convert == NULL) {
        return -1;
    }
    
    /* 保存函数指针到端口 */
    port->pwm_set_fn = set_pwm;
    port->pwm_enable_fn = enable;
    port->speed_convert_fn = speed_convert;
    
    /* 注入到电机驱动接口 */
    if (OmniMotorDriver_Init(&port->motor_driver, 
                               set_pwm, 
                               enable, 
                               speed_convert) != 0) {
        return -1;
    }
    
    return 0;
}

/**
 * @brief 获取控制器对象指针
 * 
 * 供外部调用使用
 */
OmniVehicleController_t* OmniVehiclePort_GetController(OmniVehiclePort_t* port)
{
    if (port == NULL) {
        return NULL;
    }
    
    return &port->controller;
}

/**
 * @brief 获取运动学对象指针
 */
IMecanumKinematics_t* OmniVehiclePort_GetKinematics(OmniVehiclePort_t* port)
{
    if (port == NULL) {
        return NULL;
    }
    
    return &port->kinematics;
}

/**
 * @brief 获取电机驱动对象指针
 */
IOmniMotorDriver_t* OmniVehiclePort_GetMotorDriver(OmniVehiclePort_t* port)
{
    if (port == NULL) {
        return NULL;
    }
    
    return &port->motor_driver;
}

/**
 * @brief 设置车辆几何参数
 */
int OmniVehiclePort_SetGeometry(OmniVehiclePort_t* port,
                              const VehicleGeometry_t* geometry)
{
    if (port == NULL || geometry == NULL) {
        return -1;
    }
    
    /* 更新端口的几何参数 */
    port->geometry = *geometry;
    
    /* 更新运动学对象的几何参数 - 传递端口内部geometry的地址 */
    MecanumKinematics_SetGeometry(&port->kinematics, &port->geometry);
    
    return 0;
}

/**
 * @brief 获取车辆几何参数
 */
void OmniVehiclePort_GetGeometry(OmniVehiclePort_t* port,
                                VehicleGeometry_t* geometry)
{
    if (port == NULL || geometry == NULL) {
        return;
    }
    
    *geometry = port->geometry;
}

/**
 * @brief 检查端口是否已完全初始化
 * 
 * 完全初始化的条件：
 * 1. 运动学对象已初始化
 * 2. 电机驱动函数指针已注入
 */
bool OmniVehiclePort_IsInitialized(OmniVehiclePort_t* port)
{
    if (port == NULL) {
        return false;
    }
    
    /* 检查函数指针是否已注入 */
    if (port->pwm_set_fn == NULL || 
        port->pwm_enable_fn == NULL || 
        port->speed_convert_fn == NULL) {
        return false;
    }
    
    return true;
}
//******************************** Functions ********************************//
