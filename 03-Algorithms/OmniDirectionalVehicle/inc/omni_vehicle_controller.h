/**
 ******************************************************************************
 * @file           : omni_vehicle_controller.h
 * @brief          : 车辆控制器接口定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了麦轮全向车的控制器接口
 * 整合运动学和电机驱动，实现高层控制逻辑
 * 遵循单一职责原则，负责速度控制和管理
 ******************************************************************************
 */

#ifndef OMNI_VEHICLE_CONTROLLER_H
#define OMNI_VEHICLE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include "omni_types.h"
#include "mecanum_kinematics.h"
#include "omni_motor_driver_interface.h"
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//**************************** Interface Structs ****************************//
/**
 * @brief 车辆控制器结构体
 * @note 依赖运动学对象和电机驱动对象
 */
typedef struct OmniVehicleController {
    /* 依赖对象指针 */
    IMecanumKinematics_t* kinematics;       /**< 运动学对象指针 */
    IOmniMotorDriver_t* motor_driver;       /**< 电机驱动对象指针 */

    /* 内部状态 */
    VehicleVelocity_t current_velocity;     /**< 当前车体速度 */
    WheelSpeeds_t current_wheel_speeds;     /**< 当前轮速 */
    bool is_enabled;                        /**< 控制器使能状态 */
    

    /* 函数指针(虚函数表) */
    /**
     * @brief 设置目标速度
     * @param self 控制器对象指针
     * @param target_vel 目标速度指针
     * @note 速度会立即保存到内部状态，但不立即执行
     */
    void (*set_velocity)(
        struct OmniVehicleController* self,
        const VehicleVelocity_t* target_vel);
    
    /**
     * @brief 控制器更新函数
     * @param self 控制器对象指针
     * @note 周期调用，执行运动学解算和PWM输出
     */
    void (*update)(struct OmniVehicleController* self);
    
    /**
     * @brief 使能/失能控制器
     * @param self 控制器对象指针
     * @param enable 使能标志 (true=使能, false=失能)
     */
    void (*enable)(struct OmniVehicleController* self, bool enable);
    
    /**
     * @brief 紧急停止
     * @param self 控制器对象指针
     * @note 立即停止所有电机，清零速度状态
     */
    void (*emergency_stop)(struct OmniVehicleController* self);
    
    /**
     * @brief 获取当前车体速度
     * @param self 控制器对象指针
     * @param velocity 输出速度指针
     */
    void (*get_current_velocity)(
        struct OmniVehicleController* self,
        VehicleVelocity_t* velocity);
    
    /**
     * @brief 获取当前轮速
     * @param self 控制器对象指针
     * @param wheel_speeds 输出轮速指针
     */
    void (*get_current_wheel_speeds)(
        struct OmniVehicleController* self,
        WheelSpeeds_t* wheel_speeds);
    
} OmniVehicleController_t;
//**************************** Interface Structs ****************************//
//---------------------------------------------------------------------------//
//******************************** 函数声明 ***********************************//
/**
 * @brief 初始化车辆控制器
 * @param controller 控制器对象指针
 * @param kinematics 运动学对象指针
 * @param motor_driver 电机驱动对象指针
 * @return 0=成功, -1=失败
 */
int OmniVehicleController_Init(OmniVehicleController_t* controller,
                             IMecanumKinematics_t* kinematics,
                             IOmniMotorDriver_t* motor_driver);

/**
 * @brief 设置目标速度
 * @param controller 控制器对象指针
 * @param target_vel 目标速度指针
 */
void OmniVehicleController_SetVelocity(OmniVehicleController_t* controller,
                                    const VehicleVelocity_t* target_vel);

/**
 * @brief 控制器更新
 * @param controller 控制器对象指针
 * @note 执行完整的控制流程：
 *       1. 速度限幅
 *       2. 运动学逆解算
 *       3. 速度转PWM
 *       4. PWM输出
 */
void OmniVehicleController_Update(OmniVehicleController_t* controller);

/**
 * @brief 使能/失能控制器
 * @param controller 控制器对象指针
 * @param enable 使能标志
 */
void OmniVehicleController_Enable(OmniVehicleController_t* controller, bool enable);

/**
 * @brief 紧急停止
 * @param controller 控制器对象指针
 */
void OmniVehicleController_EmergencyStop(OmniVehicleController_t* controller);

/**
 * @brief 获取当前车体速度
 * @param controller 控制器对象指针
 * @param velocity 输出速度指针
 */
void OmniVehicleController_GetCurrentVelocity(OmniVehicleController_t* controller,
                                         VehicleVelocity_t* velocity);

/**
 * @brief 获取当前轮速
 * @param controller 控制器对象指针
 * @param wheel_speeds 输出轮速指针
 */
void OmniVehicleController_GetCurrentWheelSpeeds(OmniVehicleController_t* controller,
                                              WheelSpeeds_t* wheel_speeds);
//******************************** 函数声明 ***********************************//

#ifdef __cplusplus
}
#endif

#endif /* OMNI_VEHICLE_CONTROLLER_H */
