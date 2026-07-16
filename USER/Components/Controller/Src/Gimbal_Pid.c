/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Gimbal_Pid.c
  * @brief          : RM 赛级云台优化版 PID 实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Gimbal_Pid.h"

/**
 * @brief 初始化PID参数
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID, float Param[PID_PARAMETER_NUM])
{
    if(PID->Type == PID_Type_None || Param == NULL) {
        return PID_FAILED_INIT;
    }

    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];

    if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
        LowPassFilter1p_Init(&PID->Dout_LPF, PID->Param.Alpha);
    }

    PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    // 【RM 赛级新增参数解析】
    PID->Param.IntegralSeparation = Param[7];
    PID->Param.MaxAngleRange = Param[8];

    PID->ERRORHandler.ErrorCount = 0;
    return PID_ERROR_NONE;
}

/**
 * @brief 清空PID历史数据
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
    memset(PID->Err, 0, sizeof(PID->Err));
    PID->Integral = 0;
    PID->Pout = 0;
    PID->Iout = 0;
    PID->Dout = 0;
    PID->Output = 0;
}

/**
 * @brief 初始化PID控制器
 */
void Gimbal_PID_Init(PID_Info_TypeDef *PID, PID_Type_e Type, float Param[PID_PARAMETER_NUM])
{
    PID->Type = Type;
    PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

    PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}

/**
  * @brief 判断PID计算状态 (防御性编程)
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    if(isnan(PID->Output) || isinf(PID->Output)) {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}

/**
  * @brief  RM赛级 PID核心计算函数
  */
float Gimbal_PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure)
{
    PID_ErrorHandle(PID);
    if(PID->ERRORHandler.Status != PID_ERROR_NONE) {
        PID->PID_Calc_Clear(PID);
        return 0;
    }

    PID->Target = Target;
    PID->Measure = Measure;

    // 更新误差序列
    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Target - PID->Measure;

    /* ==================================================================== */
    /* ==================== RM 赛级核心 1：角度过零处理 ==================== */
    /* ==================================================================== */
    if (PID->Type == PID_POSITION_ANGLE && PID->Param.MaxAngleRange > 0.0f)
    {
        float half_range = PID->Param.MaxAngleRange / 2.0f;
        // 走最短路径！如果误差超过半圈，自动翻转到另一边
        while (PID->Err[0] > half_range)  PID->Err[0] -= PID->Param.MaxAngleRange;
        while (PID->Err[0] < -half_range) PID->Err[0] += PID->Param.MaxAngleRange;
    }

    // 死区判断
    if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
    {
        /* ----------------------- 1. 位置环 / 角度环 ----------------------- */
        if(PID->Type == PID_POSITION || PID->Type == PID_POSITION_ANGLE)
        {
            /* ==================================================================== */
            /* ==================== RM 赛级核心 2：积分分离防超调 ================== */
            /* ==================================================================== */
            if(PID->Param.KI != 0)
            {
                // 如果没设置积分分离（值为0），或者误差进入了分离阈值内，才允许积分累加
                if(PID->Param.IntegralSeparation == 0.0f || fabsf(PID->Err[0]) < PID->Param.IntegralSeparation)
                {
                    PID->Integral += PID->Err[0];
                }
                else
                {
                    // 误差太大时清空积分，绝对禁止其发力，保证急停稳如老狗
                    PID->Integral = 0;
                }
                VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
            }
            else
            {
                PID->Integral = 0;
            }

            // 计算三项输出
            PID->Pout = PID->Param.KP * PID->Err[0];
            PID->Iout = PID->Param.KI * PID->Integral;
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);

            // D项低通滤波，极大地消除云台震荡和高频噪声
            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            // 汇总并限幅
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }

        /* --------------------------- 2. 速度环 --------------------------- */
        else if(PID->Type == PID_VELOCITY)
        {
            // 速度环（增量式）不需要积分分离，因为它本身输出的就是增量
            PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
            PID->Iout = PID->Param.KI * (PID->Err[0]);
            PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f * PID->Err[1] + PID->Err[2]);

            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            PID->Output += PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }
    }

    return PID->Output;
}