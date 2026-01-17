/**
 ******************************************************************************
 * @file           : mecanum_kinematics.c
 * @brief          : 麦轮运动学算法实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了麦轮全向车的正逆运动学计算算法
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "mecanum_kinematics.h"
#include <string.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//

/**
 * @brief 逆运动学计算：车体速度 -> 轮速
 * 
 * 麦轮运动学公式（基于标准四轮麦轮布局）：
 * ω_FL = (Vx - Vy - ω*(Lx+Ly)) / R
 * ω_FR = (Vx + Vy + ω*(Lx+Ly)) / R
 * ω_RL = (Vx + Vy + ω*(Lx+Ly)) / R
 * ω_RR = (Vx - Vy - ω*(Lx+Ly)) / R
 * 
 * 其中：
 * - Lx = wheel_base / 2 (前后轴距的一半，前轮为正)
 * - Ly = track_width / 2 (左右轮距的一半，右轮为正)
 * - R = wheel_radius (轮子半径)
 * 
 * 说明：
 * - 前左轮(FL): 位置(Lx, -Ly)，对应公式中Lx+Ly项
 * - 前右轮(FR): 位置(Lx, +Ly)，对应公式中Lx+Ly项
 * - 后左轮(RL): 位置(-Lx, -Ly)，对应公式中-(Lx+Ly)项
 * - 后右轮(RR): 位置(-Lx, +Ly)，对应公式中-(Lx+Ly)项
 */
void MecanumKinematics_InverseKinematics(IMecanumKinematics_t* self,
                                        const VehicleVelocity_t* vehicle_vel,
                                        WheelSpeeds_t* wheel_speeds)
{
    if (self == NULL || vehicle_vel == NULL || wheel_speeds == NULL)
    {
        return;
    }
    
    /* 计算几何参数 */
    float Lx = self->p_geometry->wheel_base / 2.0f;   /* 前后轴距的一半（前轮为正） */
    float Ly = self->p_geometry->track_width / 2.0f;  /* 左右轮距的一半（右轮为正） */
    float R = self->p_geometry->wheel_radius;         /* 轮子半径 */
    
    /* 速度分量 */
    float Vx = vehicle_vel->vx;
    float Vy = vehicle_vel->vy;
    float omega = vehicle_vel->omega;
    
    /* 麦轮逆运动学公式 */
    wheel_speeds->wheel_speeds[MOTOR_FRONT_LEFT]  = (Vx - Vy - omega * (Lx + Ly)) / R;
    wheel_speeds->wheel_speeds[MOTOR_FRONT_RIGHT] = (Vx + Vy + omega * (Lx + Ly)) / R;
    wheel_speeds->wheel_speeds[MOTOR_REAR_LEFT]   = (Vx + Vy + omega * (Lx + Ly)) / R;
    wheel_speeds->wheel_speeds[MOTOR_REAR_RIGHT]  = (Vx - Vy - omega * (Lx + Ly)) / R;
}

/**
 * @brief 正运动学计算：轮速 -> 车体速度
 * @note  这个轮速到车体速度没啥用，如果你想用它作为惯导的话，可以试试
 * 
 * 正运动学公式（逆运动学的逆运算）：
 * Vx = R/4 * (ω_FL + ω_FR + ω_RL + ω_RR)
 * Vy = R/4 * (-ω_FL + ω_FR + ω_RL - ω_RR)
 * ω  = R/(2*(Lx+Ly)) * (-ω_FL + ω_FR - ω_RL + ω_RR)
 * 
 * 其中：
 * - Lx = wheel_base / 2 (前后轴距的一半，前轮为正)
 * - Ly = track_width / 2 (左右轮距的一半，右轮为正)
 * - R = wheel_radius (轮子半径)
 */
void MecanumKinematics_ForwardKinematics(IMecanumKinematics_t* self,
                                      const WheelSpeeds_t* wheel_speeds,
                                      VehicleVelocity_t* vehicle_vel)
{
    if (self == NULL || wheel_speeds == NULL || vehicle_vel == NULL)
    {
        return;
    }
    
    /* 计算几何参数 */
    float Lx = self->p_geometry->wheel_base / 2.0f;   /* 前后轴距的一半（前轮为正） */
    float Ly = self->p_geometry->track_width / 2.0f;     /* 左右轮距的一半（右轮为正） */
    float R = self->p_geometry->wheel_radius;            /* 轮子半径 */
    
    /* 轮速分量 */
    float omega_FL = wheel_speeds->wheel_speeds[MOTOR_FRONT_LEFT];
    float omega_FR = wheel_speeds->wheel_speeds[MOTOR_FRONT_RIGHT];
    float omega_RL = wheel_speeds->wheel_speeds[MOTOR_REAR_LEFT];
    float omega_RR = wheel_speeds->wheel_speeds[MOTOR_REAR_RIGHT];
    
    /* 麦轮正运动学公式 */
    vehicle_vel->vx = R / 4.0f * (omega_FL + omega_FR + omega_RL + omega_RR);
    vehicle_vel->vy = R / 4.0f * (-omega_FL + omega_FR + omega_RL - omega_RR);
    vehicle_vel->omega = R / (2.0f * (Lx + Ly)) * (-omega_FL + omega_FR - omega_RL + omega_RR);
}

/**
 * @brief 设置车辆几何参数
 */
void MecanumKinematics_SetGeometry(IMecanumKinematics_t* self,
                                   const VehicleGeometry_t* geometry)
{
    if (self != NULL && geometry != NULL)
    {
        self->p_geometry = geometry;
    }
}

/**
 * @brief 初始化麦轮运动学对象
 */
int MecanumKinematics_Init(IMecanumKinematics_t* kinematics,
                           const VehicleGeometry_t* geometry)
{
    if (kinematics == NULL)
    {
        return -1;
    }

    /* 如果未提供几何参数，使用默认值 */
    if (geometry == NULL)
    {
        static VehicleGeometry_t default_geometry = {
            .wheel_base = DEFAULT_WHEEL_BASE,
            .track_width = DEFAULT_TRACK_WIDTH,
            .wheel_radius = DEFAULT_WHEEL_RADIUS
        };
        kinematics->p_geometry = &default_geometry;
    }
    else
    {
        kinematics->p_geometry = geometry;
    }

    /* 绑定虚函数表 */
    kinematics->calculate_inverse_kinematics = MecanumKinematics_InverseKinematics;
    kinematics->calculate_forward_kinematics = MecanumKinematics_ForwardKinematics;

    return 0;
}
//******************************** Functions ********************************//
