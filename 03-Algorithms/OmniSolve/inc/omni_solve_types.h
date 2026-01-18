/**
 ******************************************************************************
 * @file           : omni_solve_types.h
 * @brief          : 全向解算系统通用类型定义
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件定义了麦轮全向车控制系统中使用的所有基础数据类型
 ******************************************************************************
 */

#ifndef OMNI_SOLVE_TYPES_H
#define OMNI_SOLVE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************** Includes *********************************//
#include <stdint.h>
#include <stdbool.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Defines **********************************//
#define DEFAULT_WHEEL_BASE      0.20f   /**< 默认轴距 200mm */
#define DEFAULT_TRACK_WIDTH     0.20f   /**< 默认轮距 200mm */
#define DEFAULT_WHEEL_RADIUS    0.05f   /**< 默认轮半径 50mm */
//******************************** Defines **********************************//
//---------------------------------------------------------------------------//
//******************************** Typedefs *********************************//
/**
 * @brief 电机编号枚举
 */
typedef enum {
    MOTOR_FRONT_LEFT = 0,   /**< 前左轮电机 */
    MOTOR_FRONT_RIGHT = 1,  /**< 前右轮电机 */
    MOTOR_REAR_LEFT = 2,    /**< 后左轮电机 */
    MOTOR_REAR_RIGHT = 3,   /**< 后右轮电机 */
    MOTOR_COUNT = 4         /**< 电机总数 */
} MotorIndex_e;

/**
 * @brief 车体速度结构
 */
typedef struct {
    float vx;        /**< X方向线速度 (m/s) */
    float vy;        /**< Y方向线速度 (m/s) */
    float omega;     /**< 绕Z轴角速度 (rad/s) */
} VehicleVelocity_t;

/**
 * @brief 轮速数组结构
 */
typedef struct {
    float wheel_speeds[MOTOR_COUNT];  /**< 4个轮子的角速度数组 (rad/s) */
} WheelSpeeds_t;

/**
 * @brief 车辆几何参数结构
 */
typedef struct {
    float wheel_base;      /**< 轴距 (m) */
    float track_width;     /**< 轮距 (m) */
    float wheel_radius;    /**< 轮子半径 (m) */
} VehicleGeometry_t;
//******************************** Typedefs *********************************//
//---------------------------------------------------------------------------//
//******************************** Macros ***********************************//
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define DEG_TO_RAD(deg) ((deg) * 3.14159265359f / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / 3.14159265359f)
//******************************** Macros ***********************************//

#ifdef __cplusplus
}
#endif

#endif /* OMNI_SOLVE_TYPES_H */
