/**
 ******************************************************************************
 * @file           : mecanum_kinematics.h
 * @brief          : 麦轮运动学接口定义
 * @author         : Your Name
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了麦轮全向车的运动学计算接口
 * 实现正运动学和逆运动学计算
 * 遵循单一职责原则，仅负责速度转换计算
 ******************************************************************************
 */

#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************//
//******************************** Includes **********************************//
//*****************************************************************************//
#include "omni_types.h"

//*****************************************************************************//
//******************************** Defines ***********************************//
//*****************************************************************************//

//*****************************************************************************//
//******************************** Typedefs **********************************//
//*****************************************************************************//

//*****************************************************************************//
//**************************** Interface Structs *******************************//
//*****************************************************************************//

/**
 * @brief 麦轮运动学接口结构体
 * @note 包含车辆几何参数和运动学计算函数
 */
typedef struct IMecanumKinematics {
    /*************************************************************************/
    /* 车辆几何参数 */
    /*************************************************************************/
    VehicleGeometry_t geometry;     /**< 车辆几何参数 */
    
    /*************************************************************************/
    /* 函数指针(虚函数表) */
    /*************************************************************************/
    /**
     * @brief 逆运动学计算：车体速度 -> 轮速
     * @param self 运动学对象指针
     * @param vehicle_vel 车体速度指针 (输入)
     * @param wheel_speeds 轮速数组指针 (输出)
     */
    void (*calculate_inverse_kinematics)(
        struct IMecanumKinematics* self,
        const VehicleVelocity_t* vehicle_vel,
        WheelSpeeds_t* wheel_speeds);
    
    /**
     * @brief 正运动学计算：轮速 -> 车体速度
     * @param self 运动学对象指针
     * @param wheel_speeds 轮速数组指针 (输入)
     * @param vehicle_vel 车体速度指针 (输出)
     */
    void (*calculate_forward_kinematics)(
        struct IMecanumKinematics* self,
        const WheelSpeeds_t* wheel_speeds,
        VehicleVelocity_t* vehicle_vel);
    
} IMecanumKinematics_t;

//*****************************************************************************//
//******************************** Classes ***********************************//
//*****************************************************************************//

//*****************************************************************************//
//**************************** Extern Variables *******************************//
//*****************************************************************************//

//*****************************************************************************//
//******************************** 函数声明 **********************************//
//*****************************************************************************//

/**
 * @brief 初始化麦轮运动学对象
 * @param kinematics 运动学对象指针
 * @param geometry 车辆几何参数指针
 * @return 0=成功, -1=失败
 */
int MecanumKinematics_Init(IMecanumKinematics_t* kinematics,
                           const VehicleGeometry_t* geometry);

/**
 * @brief 逆运动学计算：车体速度 -> 轮速
 * @param self 运动学对象指针
 * @param vehicle_vel 车体速度指针
 * @param wheel_speeds 轮速数组指针
 * 
 * 运动学公式：
 * ω_FL = (Vx - Vy - ω*(Lx+Ly)) / R
 * ω_FR = (Vx + Vy + ω*(Lx+Ly)) / R
 * ω_RL = (Vx + Vy - ω*(Lx+Ly)) / R
 * ω_RR = (Vx - Vy + ω*(Lx+Ly)) / R
 * 
 * 其中：
 * - Lx = track_width / 2 (左右轮距的一半)
 * - Ly = wheel_base / 2 (前后轴距的一半)
 * - R = wheel_radius (轮子半径)
 */
void MecanumKinematics_InverseKinematics(IMecanumKinematics_t* self,
                                        const VehicleVelocity_t* vehicle_vel,
                                        WheelSpeeds_t* wheel_speeds);

/**
 * @brief 正运动学计算：轮速 -> 车体速度
 * @param self 运动学对象指针
 * @param wheel_speeds 轮速数组指针
 * @param vehicle_vel 车体速度指针
 * 
 * 运动学公式（逆运动学的逆运算）：
 * Vx = R/4 * (ω_FL + ω_FR + ω_RL + ω_RR)
 * Vy = R/4 * (-ω_FL + ω_FR + ω_RL - ω_RR)
 * ω  = R/(2*(Lx+Ly)) * (-ω_FL + ω_FR - ω_RL + ω_RR)
 */
void MecanumKinematics_ForwardKinematics(IMecanumKinematics_t* self,
                                      const WheelSpeeds_t* wheel_speeds,
                                      VehicleVelocity_t* vehicle_vel);

/**
 * @brief 设置车辆几何参数
 * @param self 运动学对象指针
 * @param geometry 车辆几何参数指针
 */
void MecanumKinematics_SetGeometry(IMecanumKinematics_t* self,
                                   const VehicleGeometry_t* geometry);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_KINEMATICS_H */
