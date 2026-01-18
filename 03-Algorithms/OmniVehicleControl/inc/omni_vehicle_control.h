/**
 ******************************************************************************
 * @file           : omni_vehicle_control.h
 * @brief          : 全向车控制模块接口定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-18
 ******************************************************************************
 * @description
 * 本文件定义了全向车控制模块的统一接口
 * 封装了运动学、电机驱动、PID控制器
 * 提供简单的set_velocity API，内部完成车体速度到轮速、轮速到PWM、PWM到电机的控制流程
 ******************************************************************************
 */

#ifndef OMNI_VEHICLE_CONTROL_H
#define OMNI_VEHICLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include "omni_solve_types.h"
#include "motor_driver_interface.h"
#include "pid_controller.h"
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Typedefs *********************************//

//******************************** Typedefs *********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//
/**
 * @brief 实例化全向车控制模块（inst）
 * @param set_pwm PWM设置函数指针
 * @param enable_motor 电机使能函数指针
 * @param read_encoder 编码器读取函数（接收电机编号参数）
 * @param geometry 车辆几何参数指针
 * @param pid_params PID参数数组[MOTOR_COUNT][3]，每个元素为{kp, ki, kd}
 * @return 0=成功, -1=失败
 * 
 * @note 此函数内部完成：
 *       1. 调用MecanumKinematics_Init初始化全局运动学对象
 *       2. 调用OmniMotorDriver_Init初始化全局电机驱动对象
 *       3. 调用PID_Inst初始化4个全局PID控制器
 */
int OmniVehicleControl_Inst(
    SetMotorPWM_fn set_pwm,
    EnableMotor_fn enable_motor,
    ReadEncoder_fn read_encoder,
    const VehicleGeometry_t* geometry,
    const float pid_params[MOTOR_COUNT][3]);

/**
 * @brief 设置车体速度
 * @param vehicle_vel 车体速度指针
 * 
 * @note 内部流程：
 *       1. 调用运动学逆解：VehicleVelocity -> WheelSpeeds
 *       2. 调用PID控制器：WheelSpeed -> PWM
 *       3. 调用电机驱动：设置PWM
 */
void OmniVehicleControl_SetVelocity(const VehicleVelocity_t* vehicle_vel);

/**
 * @brief 设置指定电机的PID参数
 * @param motor_index 电机编号
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
void OmniVehicleControl_SetPIDParams(MotorIndex_e motor_index, 
                                   float kp, float ki, float kd);

/**
 * @brief 复位指定电机的PID控制器
 * @param motor_index 电机编号
 */
void OmniVehicleControl_ResetPID(MotorIndex_e motor_index);

/**
 * @brief 复位所有PID控制器
 */
void OmniVehicleControl_ResetAllPID(void);

/**
 * @brief 使能指定电机
 * @param motor_index 电机编号
 * @param enable 使能标志
 */
void OmniVehicleControl_EnableMotor(MotorIndex_e motor_index, bool enable);

/**
 * @brief 使能所有电机
 * @param enable 使能标志
 */
void OmniVehicleControl_EnableAllMotors(bool enable);

/**
 * @brief 设置电机PWM（直接调用电机驱动接口）
 * @param motor_index 电机编号
 * @param duty_cycle PWM占空比 (-100.0 ~ 100.0)
 */
void OmniVehicleControl_SetMotorPWM(MotorIndex_e motor_index, float duty_cycle);

/**
 * @brief 获取指定轮速
 * @param motor_index 电机编号
 * @return 轮速（rad/s）
 */
float OmniVehicleControl_GetWheelSpeed(MotorIndex_e motor_index);

/**
 * @brief 销毁全向车控制模块
 * @note 释放inst函数分配的资源
 */
void OmniVehicleControl_Deinit(void);

//******************************** Functions ********************************//

#ifdef __cplusplus
}
#endif

#endif /* OMNI_VEHICLE_CONTROL_H */
