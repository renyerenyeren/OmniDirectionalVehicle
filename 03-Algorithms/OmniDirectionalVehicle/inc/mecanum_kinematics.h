/**
 ******************************************************************************
 * @file           : mecanum_kinematics.h
 * @brief          : 麦轮运动学接口定义
 * @author         : Renyerenyeren
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

//******************************** Includes *********************************//
#include "omni_types.h"
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//**************************** Interface Structs ****************************//
/**
 * @brief 麦轮运动学接口结构体
 * @note 包含车辆几何参数和运动学计算函数
 */
typedef struct IMecanumKinematics {
    /* 车辆几何参数指针 */
    const VehicleGeometry_t* p_geometry;     /**< 车辆几何参数指针 */

    /* 函数指针(虚函数表) */
    /**
     * @brief 逆运动学计算：车体速度 -> 轮速
     * @param self 运动学对象指针
     * @param[in]  vehicle_vel 车体速度指针
     * @param[out] wheel_speeds 轮速数组指针
     */
    void (*calculate_inverse_kinematics)(
        struct IMecanumKinematics* self,
        const VehicleVelocity_t* vehicle_vel,
        WheelSpeeds_t* wheel_speeds);
    
    /**
     * @brief 正运动学计算：轮速 -> 车体速度
     * @param self 运动学对象指针
     * @param[in]  wheel_speeds 轮速数组指针
     * @param[out] vehicle_vel 车体速度指针
     * @note  这个轮速到车体速度没啥用，如果你想用它作为惯导的话，可以试试
     */
    void (*calculate_forward_kinematics)(
        struct IMecanumKinematics* self,
        const WheelSpeeds_t* wheel_speeds,
        VehicleVelocity_t* vehicle_vel);
    
} IMecanumKinematics_t;
//**************************** Interface Structs ****************************//
//---------------------------------------------------------------------------//
//******************************** 函数声明 ***********************************//
/**
 * @brief 设置车辆几何参数
 * @param self 运动学对象指针
 * @param geometry 车辆几何参数指针
 */
void MecanumKinematics_SetGeometry(IMecanumKinematics_t* self,
                                   const VehicleGeometry_t* geometry);

/**
 * @brief 初始化麦轮运动学对象
 * @param kinematics 运动学对象指针
 * @param geometry 车辆几何参数指针
 * @return 0=成功, -1=失败
 */
int MecanumKinematics_Init(IMecanumKinematics_t* kinematics,
                           const VehicleGeometry_t* geometry);
//******************************** 函数声明 ***********************************//

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_KINEMATICS_H */
