/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Task.h
  * @brief          : 六轴机械臂控制任务头文件
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 基于原Gimbal_Task改造，适配六轴串联机械臂
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "stdint.h"
#include "stdbool.h"
#include "Gimbal_Pid.h"     /* PID_Info_TypeDef */
#include "Motor.h"          /* DM_Motor_Info_Typedef */

/* 机械臂关节数量 */
#define ARM_JOINT_NUM  6

/**
 * @brief 机械臂关节控制结构体
 *        每个关节包含一组串级PID（角度环+速度环）和相关状态
 */
typedef struct
{
    // ---- PID 实例 ----
  //  PID_Info_TypeDef Angle_PID;   /*!< 外环：角度环 PID */
  //  PID_Info_TypeDef Speed_PID;   /*!< 内环：速度环 PID */

    // ---- 状态变量 ----
    float Target_Angle;           /*!< 目标角度 （单位：度）*/
    float Current_Angle;          /*!< 当前角度 （单位：度），来自电机编码器反馈 */
    float Current_Velocity;       /*!< 当前速度 （单位：度/秒），来自电机编码器反馈 */

    // ---- 关节限位 ----
    float Max_Angle;              /*!< 关节角度上限 （单位：度）*/
    float Min_Angle;              /*!< 关节角度下限 （单位：度）*/
    float Max_Velocity;           /*!< 关节最大速度 （单位：度/秒）*/
    float Max_Torque;             /*!< 关节最大力矩 （单位：N·m）*/

    // ---- 重力前馈 ----
    float Gravity_Offset;         /*!< 关节水平时的重力补偿力矩 （单位：N·m）*/

    // ---- 达妙电机指针 ----
    DM_Motor_Info_Typedef *Motor; /*!< 指向该关节对应的达妙电机结构体 */
} Arm_Joint_Control_Typedef;

/* 外部接口 */
extern Arm_Joint_Control_Typedef Arm_Joints[ARM_JOINT_NUM];
extern void Arm_Control_Task(void const * argument);

#endif //ARM_TASK_H
