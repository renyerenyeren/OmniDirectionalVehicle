/**
 ******************************************************************************
 * @file           : pid_controller.c
 * @brief          : PID控制器接口实现
 * @author         : Renyerenyeren
 * @date           : 2026-01-17
 ******************************************************************************
 * @description
 * 本文件实现了PID控制器的抽象接口
 * 提供多种PID算法实现（标准PID、积分分离PID、增量式PID等）
 * 通过虚函数表实现多态
 ******************************************************************************
 */

//******************************** Includes *********************************//
#include "pid_controller.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
//******************************** Includes *********************************//
//---------------------------------------------------------------------------//
//******************************** Private Data Structures *********************************//

/**
 * @brief 标准PID私有数据
 */
typedef struct {
    float integral;    /**< 积分项 */
    float prev_error;  /**< 上一次误差 */
    float kp, ki, kd;  /**< PID参数 */
} StandardPID_t;

/**
 * @brief 积分分离PID私有数据
 */
typedef struct {
    float integral;          /**< 积分项 */
    float prev_error;       /**< 上一次误差 */
    float kp, ki, kd;     /**< PID参数 */
    float separation_threshold; /**< 积分分离阈值 */
} IntegralSeparationPID_t;

/**
 * @brief 增量式PID私有数据
 */
typedef struct {
    float prev_error_1;  /**< 上一次误差 */
    float prev_error_2;  /**< 上上一次误差 */
    float kp, ki, kd;   /**< PID参数 */
} IncrementalPID_t;

//******************************** Private Data Structures *********************************//
//---------------------------------------------------------------------------//
//**************************** Private Function *****************************//

/* 对外暴露的SpeedToPWM接口（内部调用PID计算）*/
static float PID_SpeedToPWM(void* self, float target_speed_rad_s);

/* 标准PID方法 */
static float StandardPID_Calculate(void* self, float target, float current);
static void StandardPID_SetParams(void* self, float kp, float ki, float kd);
static void StandardPID_Reset(void* self);

/* 积分分离PID方法 */
static float IntegralSeparationPID_Calculate(void* self, float target, float current);
static void IntegralSeparationPID_SetParams(void* self, float kp, float ki, float kd);
static void IntegralSeparationPID_Reset(void* self);

/* 增量式PID方法 */
static float IncrementalPID_Calculate(void* self, float target, float current);
static void IncrementalPID_SetParams(void* self, float kp, float ki, float kd);
static void IncrementalPID_Reset(void* self);

//**************************** Private Function *****************************//
//---------------------------------------------------------------------------//
//***************************** Method Tables *******************************//

/* 全局方法表（单例，所有标准PID共享）*/
static const PID_Methods_t standard_pid_methods = {
    .calculate = StandardPID_Calculate,
    .set_params = StandardPID_SetParams,
    .reset = StandardPID_Reset
};

/* 全局方法表（单例，所有积分分离PID共享）*/
static const PID_Methods_t integral_separation_pid_methods = {
    .calculate = IntegralSeparationPID_Calculate,
    .set_params = IntegralSeparationPID_SetParams,
    .reset = IntegralSeparationPID_Reset
};

/* 全局方法表（单例，所有增量式PID共享）*/
static const PID_Methods_t incremental_pid_methods = {
    .calculate = IncrementalPID_Calculate,
    .set_params = IncrementalPID_SetParams,
    .reset = IncrementalPID_Reset
};

//***************************** Method Tables *******************************//
//---------------------------------------------------------------------------//
//******************************** Functions ********************************//

/**
 * @brief 对外暴露的SpeedToPWM接口
 * @note 此函数在内部调用PID计算
 * @param self PID控制器对象指针
 * @param target_speed_rad_s 目标轮速
 * @return PWM占空比 (-100.0 ~ 100.0)
 */
static float PID_SpeedToPWM(void* self, float target_speed_rad_s)
{
    IPIDController_t* pid_ctrl = (IPIDController_t*)self;

    if (self == NULL || pid_ctrl->read_encoder == NULL || pid_ctrl->methods == NULL)
    {
        return 0.0f;
    }
    
    /* 1. 读取编码器获取实际速度 */
    float current_speed = pid_ctrl->read_encoder();
    
    /* 2. 调用内部PID计算方法 */
    float pwm = pid_ctrl->methods->calculate(pid_ctrl->private_data, target_speed_rad_s, current_speed);
    
    /* 3. 返回PWM占空比（限幅）*/
    if (pwm > 100.0f)
    {
        return 100.0f;
    }
    else if (pwm < -100.0f)
    {
        return -100.0f;
    }
    else
    {
        return pwm;
    }
}

/**
 * @brief PID控制器实例化函数（inst）
 * @param pid PID控制器对象指针
 * @param read_encoder 编码器读取函数指针（外部依赖注入）
 * @param pid_type PID算法类型（0=标准PID, 1=积分分离PID, 2=增量式PID）
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 * @return 0=成功, -1=失败
 */
int PID_Inst(IPIDController_t* pid,
           ReadEncoder_fn read_encoder,
           int pid_type,
           float kp, float ki, float kd)
{
    if (pid == NULL || read_encoder == NULL)
    {
        return -1;
    }
    
    /* 1. 注入外部依赖 */
    pid->read_encoder = read_encoder;
    
    /* 2. 根据类型分配并初始化私有数据 */
    switch (pid_type)
    {
        case 0:  /* 标准PID */
            {
                StandardPID_t* priv = (StandardPID_t*)malloc(sizeof(StandardPID_t));
                if (priv == NULL)
                {
                    return -1;
                }
                
                priv->integral = 0.0f;
                priv->prev_error = 0.0f;
                priv->kp = kp;
                priv->ki = ki;
                priv->kd = kd;
                
                pid->private_data = priv;
                pid->methods = &standard_pid_methods;  /* 绑定方法表 */
            }
            break;
            
        case 1:  /* 积分分离PID */
            {
                IntegralSeparationPID_t* priv = (IntegralSeparationPID_t*)malloc(sizeof(IntegralSeparationPID_t));
                if (priv == NULL)
                {
                    return -1;
                }
                
                priv->integral = 0.0f;
                priv->prev_error = 0.0f;
                priv->kp = kp;
                priv->ki = ki;
                priv->kd = kd;
                priv->separation_threshold = 5.0f;  /* 默认阈值 */
                
                pid->private_data = priv;
                pid->methods = &integral_separation_pid_methods;  /* 绑定方法表 */
            }
            break;
            
        case 2:  /* 增量式PID */
            {
                IncrementalPID_t* priv = (IncrementalPID_t*)malloc(sizeof(IncrementalPID_t));
                if (priv == NULL)
                {
                    return -1;
                }
                
                priv->prev_error_1 = 0.0f;
                priv->prev_error_2 = 0.0f;
                priv->kp = kp;
                priv->ki = ki;
                priv->kd = kd;
                
                pid->private_data = priv;
                pid->methods = &incremental_pid_methods;  /* 绑定方法表 */
            }
            break;
            
        default:
            return -1;
    }
    
    /* 3. 绑定对外暴露的接口（SpeedToPWM）*/
    pid->speed_to_pwm = PID_SpeedToPWM;
    
    return 0;
}

//******************************** Functions ********************************//
//---------------------------------------------------------------------------//
//******************************* 标准PID实例 *********************************//

/**
 * @brief 标准PID计算方法
 * @param self 私有数据指针
 * @param target 目标速度
 * @param current 实际速度
 * @return 输出值（PWM占空比）
 */
static float StandardPID_Calculate(void* self, float target, float current)
{
    StandardPID_t* pid = (StandardPID_t*)self;
    
    float error = target - current;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    pid->prev_error = error;
    
    return output;
}

/**
 * @brief 标准PID参数设置方法
 * @param self 私有数据指针
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
static void StandardPID_SetParams(void* self, float kp, float ki, float kd)
{
    StandardPID_t* pid = (StandardPID_t*)self;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 标准PID复位方法
 * @param self 私有数据指针
 */
static void StandardPID_Reset(void* self)
{
    StandardPID_t* pid = (StandardPID_t*)self;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

//******************************* 标准PID实例 *********************************//
//---------------------------------------------------------------------------//
//***************************** 积分分离PID实例 ********************************//

/**
 * @brief 积分分离PID计算方法
 * @param self 私有数据指针
 * @param target 目标速度
 * @param current 实际速度
 * @return 输出值（PWM占空比）
 */
static float IntegralSeparationPID_Calculate(void* self, float target, float current)
{
    IntegralSeparationPID_t* pid = (IntegralSeparationPID_t*)self;
    
    float error = target - current;
    
    /* 积分分离逻辑 */
    if (fabs(error) < pid->separation_threshold)
    {
        pid->integral += error;
    }
    else
    {
        pid->integral = 0.0f;
    }
    
    float derivative = error - pid->prev_error;
    
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    pid->prev_error = error;
    
    return output;
}

/**
 * @brief 积分分离PID参数设置方法
 * @param self 私有数据指针
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
static void IntegralSeparationPID_SetParams(void* self, float kp, float ki, float kd)
{
    IntegralSeparationPID_t* pid = (IntegralSeparationPID_t*)self;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 积分分离PID复位方法
 * @param self 私有数据指针
 */
static void IntegralSeparationPID_Reset(void* self)
{
    IntegralSeparationPID_t* pid = (IntegralSeparationPID_t*)self;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

//***************************** 积分分离PID实例 ********************************//
//---------------------------------------------------------------------------//
//***************************** 增量式PID实例 *********************************//

/**
 * @brief 增量式PID计算方法
 * @param self 私有数据指针
 * @param target 目标速度
 * @param current 实际速度
 * @return 输出值（PWM增量）
 */
static float IncrementalPID_Calculate(void* self, float target, float current)
{
    IncrementalPID_t* pid = (IncrementalPID_t*)self;
    
    float error = target - current;
    
    float delta_u = pid->kp * (error - pid->prev_error_1) + 
                   pid->ki * error + 
                   pid->kd * (error - 2 * pid->prev_error_1 + pid->prev_error_2);
    
    pid->prev_error_2 = pid->prev_error_1;
    pid->prev_error_1 = error;
    
    return delta_u;  /* 返回增量 */
}

/**
 * @brief 增量式PID参数设置方法
 * @param self 私有数据指针
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
static void IncrementalPID_SetParams(void* self, float kp, float ki, float kd)
{
    IncrementalPID_t* pid = (IncrementalPID_t*)self;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 增量式PID复位方法
 * @param self 私有数据指针
 */
static void IncrementalPID_Reset(void* self)
{
    IncrementalPID_t* pid = (IncrementalPID_t*)self;
    pid->prev_error_1 = 0.0f;
    pid->prev_error_2 = 0.0f;
}

//***************************** 增量式PID实例 *********************************//
