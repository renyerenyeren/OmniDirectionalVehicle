/**
 ******************************************************************************
 * @file           : omni_vehicle_controller.c
 * @brief          : 车辆控制器逻辑实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了麦轮全向车的控制器逻辑
 * 整合运动学和电机驱动，实现速度控制
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "omni_vehicle_controller.h"
#include <string.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//
/**
 * @brief 初始化车辆控制器
 */
int OmniVehicleController_Init(OmniVehicleController_t* controller,
                             IMecanumKinematics_t* kinematics,
                             IOmniMotorDriver_t* motor_driver)
{
    if (controller == NULL || kinematics == NULL || motor_driver == NULL) {
        return -1;
    }
    
    /* 注入依赖对象 */
    controller->kinematics = kinematics;
    controller->motor_driver = motor_driver;
    
    /* 初始化内部状态 */
    memset(&controller->current_velocity, 0, sizeof(VehicleVelocity_t));
    memset(&controller->current_wheel_speeds, 0, sizeof(WheelSpeeds_t));
    controller->is_enabled = false;
    
    /* 绑定虚函数表 */
    controller->set_velocity = OmniVehicleController_SetVelocity;
    controller->update = OmniVehicleController_Update;
    controller->enable = OmniVehicleController_Enable;
    controller->emergency_stop = OmniVehicleController_EmergencyStop;
    controller->get_current_velocity = OmniVehicleController_GetCurrentVelocity;
    controller->get_current_wheel_speeds = OmniVehicleController_GetCurrentWheelSpeeds;
    
    return 0;
}

/**
 * @brief 设置目标速度
 */
void OmniVehicleController_SetVelocity(OmniVehicleController_t* controller,
                                    const VehicleVelocity_t* target_vel)
{
    if (controller == NULL || target_vel == NULL) {
        return;
    }
    
    /* 直接保存目标速度到内部状态 */
    controller->current_velocity.vx = target_vel->vx;
    controller->current_velocity.vy = target_vel->vy;
    controller->current_velocity.omega = target_vel->omega;
}

/**
 * @brief 控制器更新函数
 * 
 * 执行完整的控制流程：
 * 1. 速度限幅
 * 2. 运动学逆解算（车体速度 -> 轮速）
 * 3. 速度转PWM
 * 4. PWM输出
 */
void OmniVehicleController_Update(OmniVehicleController_t* controller)
{
    if (controller == NULL) {
        return;
    }
    
    /* 检查是否使能 */
    if (!controller->is_enabled) {
        return;
    }
    
    /* 1. 速度限幅 */
    VehicleVelocity_t limited_vel = controller->current_velocity;
    limited_vel.vx = CLAMP(limited_vel.vx, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    limited_vel.vy = CLAMP(limited_vel.vy, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    limited_vel.omega = CLAMP(limited_vel.omega, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    /* 2. 运动学逆解算（零拷贝）*/
    controller->kinematics->calculate_inverse_kinematics(
        controller->kinematics,
        &limited_vel,
        &controller->current_wheel_speeds);
    
    /* 3. 轮速限幅 */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        controller->current_wheel_speeds.wheel_speeds[i] = CLAMP(
            controller->current_wheel_speeds.wheel_speeds[i],
            -MAX_WHEEL_SPEED,
            MAX_WHEEL_SPEED
        );
    }
    
    /* 4. 速度转PWM并输出 */
    for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++) {
        float pwm_duty = controller->motor_driver->speed_to_pwm(
            controller->current_wheel_speeds.wheel_speeds[i]);
        
        /* PWM占空比限幅 */
        pwm_duty = CLAMP(pwm_duty, -100.0f, 100.0f);
        
        /* 调用底层驱动设置PWM */
        controller->motor_driver->set_motor_pwm(i, pwm_duty);
    }
}

/**
 * @brief 使能/失能控制器
 */
void OmniVehicleController_Enable(OmniVehicleController_t* controller, bool enable)
{
    if (controller == NULL) {
        return;
    }
    
    controller->is_enabled = enable;
    
    /* 使能或失能所有电机 */
    controller->motor_driver->enable_motor(MOTOR_FRONT_LEFT, enable);
    controller->motor_driver->enable_motor(MOTOR_FRONT_RIGHT, enable);
    controller->motor_driver->enable_motor(MOTOR_REAR_LEFT, enable);
    controller->motor_driver->enable_motor(MOTOR_REAR_RIGHT, enable);
}

/**
 * @brief 紧急停止
 * 
 * 立即停止所有电机，清零速度状态
 */
void OmniVehicleController_EmergencyStop(OmniVehicleController_t* controller)
{
    if (controller == NULL) {
        return;
    }
    
    /* 清零速度状态 */
    memset(&controller->current_velocity, 0, sizeof(VehicleVelocity_t));
    memset(&controller->current_wheel_speeds, 0, sizeof(WheelSpeeds_t));
    
    /* 立即停止所有电机 */
    for (MotorIndex_e i = MOTOR_FRONT_LEFT; i < MOTOR_COUNT; i++) {
        controller->motor_driver->set_motor_pwm(i, 0.0f);
    }
}

/**
 * @brief 获取当前车体速度
 */
void OmniVehicleController_GetCurrentVelocity(OmniVehicleController_t* controller,
                                         VehicleVelocity_t* velocity)
{
    if (controller == NULL || velocity == NULL) {
        return;
    }
    
    *velocity = controller->current_velocity;
}

/**
 * @brief 获取当前轮速
 */
void OmniVehicleController_GetCurrentWheelSpeeds(OmniVehicleController_t* controller,
                                              WheelSpeeds_t* wheel_speeds)
{
    if (controller == NULL || wheel_speeds == NULL) {
        return;
    }
    
    *wheel_speeds = controller->current_wheel_speeds;
}
//******************************** Functions ********************************//