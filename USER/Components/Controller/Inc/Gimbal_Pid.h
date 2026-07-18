/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Gimbal_Pid.h
  * @brief          : 赛级优化版 云台 PID 头文件 (支持过零处理与积分分离)
  * @version        : v2.0 (RM 赛级终极形态)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __GIMBAL_PID_H
#define __GIMBAL_PID_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

// 通用限幅宏定义 (如果你在别的地方定义了，可以注释掉这行)
#ifndef VAL_LIMIT
#define VAL_LIMIT(val, min, max) \
do {\
    if((val) > (max)) { (val) = (max); }\
    else if((val) < (min)) { (val) = (min); }\
} while(0)
#endif

/* PID 参数数量从 7 个扩充到 9 个 */
#define PID_PARAMETER_NUM 9

// 你的低通滤波器结构体 (假设已在其他文件中定义，这里提供基础结构备用)
typedef struct {
    float Alpha;
    float Out;
} LowPassFilter1p_TypeDef;

//extern void LowPassFilter1p_Init(LowPassFilter1p_TypeDef *lpf, float alpha);
//extern float LowPassFilter1p_Update(LowPassFilter1p_TypeDef *lpf, float input);

// PID 模式枚举
typedef enum {
    PID_Type_None = 0,
    PID_POSITION,        // 普通位置环 (如电机编码器位置)
    PID_VELOCITY,        // 普通速度环 (增量式 PID)
    PID_POSITION_ANGLE   // 【RM 赛级新增】：角度位置环 (自带 360度/180度 过零处理)
} PID_Type_e;

// PID 状态枚举
typedef enum {
    PID_ERROR_NONE = 0,
    PID_FAILED_INIT,
    PID_CALC_NANINF
} PID_Status_e;

typedef struct {
    PID_Status_e Status;
    uint32_t ErrorCount;
} PID_ErrorHandler_t;

// PID 参数结构体
typedef struct {
    float KP;
    float KI;
    float KD;
    float Alpha;               // D 项低通滤波系数 (0~1)
    float Deadband;            // 误差死区
    float LimitIntegral;       // 积分限幅
    float LimitOutput;         // 输出限幅

    // 【RM 赛级新增参数】
    float IntegralSeparation;  // 积分分离阈值 (误差小于此值才开启积分，防止大范围转向时严重超调)
    float MaxAngleRange;       // 角度过零处理范围 (例如 360.0f 或者 8192.0f)
} PID_Param_TypeDef;

// PID 核心结构体
typedef struct PID_Info_TypeDef {
    PID_Type_e Type;
    PID_Param_TypeDef Param;

    float Target;
    float Measure;
    float Err[3];              // [0]当前误差, [1]上次误差, [2]上上次误差

    float Pout;
    float Iout;
    float Dout;
    float Output;

    float Integral;            // 积分累加值

    LowPassFilter1p_TypeDef Dout_LPF; // D项低通滤波器
    PID_ErrorHandler_t ERRORHandler;

    // 函数指针
    void (*PID_Calc_Clear)(struct PID_Info_TypeDef *PID);
    PID_Status_e (*PID_Param_Init)(struct PID_Info_TypeDef *PID, float Param[PID_PARAMETER_NUM]);
} PID_Info_TypeDef;

/* 对外暴露的 API */
void Gimbal_PID_Init(PID_Info_TypeDef *PID, PID_Type_e Type, float Param[PID_PARAMETER_NUM]);
extern float Gimbal_PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure);

#endif /* __GIMBAL_PID_H */