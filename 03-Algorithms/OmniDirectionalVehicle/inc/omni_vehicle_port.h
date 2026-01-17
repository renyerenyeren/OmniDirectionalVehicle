/**
 ******************************************************************************
 * @file           : omni_vehicle_port.h
 * @brief          : 依赖注入端口定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了依赖注入端口，用于管理所有对象的创建和依赖绑定
 * 遵循门面模式和工厂模式，对外提供统一接口
 ******************************************************************************
 */

#ifndef OMNI_VEHICLE_PORT_H
#define OMNI_VEHICLE_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include "omni_types.h"
#include "omni_motor_driver_interface.h"
#include "mecanum_kinematics.h"
#include "omni_vehicle_controller.h"
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//**************************** Interface Structs ****************************//
/**
 * @brief 依赖注入端口结构体
 * @note 管理所有对象的生命周期和依赖关系
 */
typedef struct OmniVehiclePort {
    /* 待注入的底层驱动函数指针 */
    SetMotorPWM_fn pwm_set_fn;           /**< PWM设置函数指针 */
    EnableMotor_fn pwm_enable_fn;         /**< PWM使能函数指针 */
    SpeedToPWM_fn speed_convert_fn;       /**< 速度转换函数指针 */

    /* 内部对象实例 */
    IOmniMotorDriver_t motor_driver;      /**< 电机驱动对象 */
    IMecanumKinematics_t kinematics;    /**< 运动学对象 */
    OmniVehicleController_t controller;    /**< 控制器对象 */

    /* 车辆几何参数 */
    VehicleGeometry_t geometry;           /**< 车辆几何参数 */
    
} OmniVehiclePort_t;
//**************************** Interface Structs ****************************//
//---------------------------------------------------------------------------//
//******************************** 函数声明 ***********************************//
/**
 * @brief 初始化依赖注入端口
 * @param port 端口对象指针
 * @param geometry 车辆几何参数指针 (NULL则使用默认值)
 * @return 0=成功, -1=失败
 * 
 * @note 此函数完成以下工作：
 *       1. 初始化车辆几何参数
 *       2. 创建运动学对象
 *       3. 初始化电机驱动接口结构体
 *       4. 创建控制器对象
 *       5. 建立对象间的依赖关系
 */
int OmniVehiclePort_Init(OmniVehiclePort_t* port,
                        const VehicleGeometry_t* geometry);

/**
 * @brief 注入PWM底层驱动函数
 * @param port 端口对象指针
 * @param set_pwm PWM设置函数指针
 * @param enable PWM使能函数指针
 * @param speed_convert 速度转换函数指针
 * @return 0=成功, -1=失败
 * 
 * @note 必须在Init之后调用
 *       将BSP层的PWM驱动函数注入到算法层
 */
int OmniVehiclePort_InjectPWMDriver(OmniVehiclePort_t* port,
                                    SetMotorPWM_fn set_pwm,
                                    EnableMotor_fn enable,
                                    SpeedToPWM_fn speed_convert);

/**
 * @brief 获取控制器对象指针
 * @param port 端口对象指针
 * @return 控制器对象指针
 * 
 * @note 供外部调用使用
 */
OmniVehicleController_t* OmniVehiclePort_GetController(OmniVehiclePort_t* port);

/**
 * @brief 获取运动学对象指针
 * @param port 端口对象指针
 * @return 运动学对象指针
 */
IMecanumKinematics_t* OmniVehiclePort_GetKinematics(OmniVehiclePort_t* port);

/**
 * @brief 获取电机驱动对象指针
 * @param port 端口对象指针
 * @return 电机驱动对象指针
 */
IOmniMotorDriver_t* OmniVehiclePort_GetMotorDriver(OmniVehiclePort_t* port);

/**
 * @brief 设置车辆几何参数
 * @param port 端口对象指针
 * @param geometry 车辆几何参数指针
 * @return 0=成功, -1=失败
 */
int OmniVehiclePort_SetGeometry(OmniVehiclePort_t* port,
                              const VehicleGeometry_t* geometry);

/**
 * @brief 获取车辆几何参数
 * @param port 端口对象指针
 * @param geometry 输出几何参数指针
 */
void OmniVehiclePort_GetGeometry(OmniVehiclePort_t* port,
                                VehicleGeometry_t* geometry);

/**
 * @brief 检查端口是否已完全初始化
 * @param port 端口对象指针
 * @return true=已初始化, false=未初始化
 */
bool OmniVehiclePort_IsInitialized(OmniVehiclePort_t* port);
//******************************** 函数声明 ***********************************//

#ifdef __cplusplus
}
#endif

#endif /* OMNI_VEHICLE_PORT_H */
