/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Controller.h
  * @brief          : 六轴机械臂控制器（Controller 层）
  * @note           : 集中拥有机械臂关节配置表 / 零位表 / 重力补偿表 / 坐标转换，
  *                   以及关节控制逻辑（增量控制、夹爪、急停/失联保护）。
  *                   由原 Arm_Task.c 拆分而来，Task 层只做任务编排。
  *                   所有角度统一弧度 (rad)；Target_Angle / Current_Angle 均为
  *                   "关节坐标"（编码器角减零位偏移后的角度）。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "stdint.h"
#include "stdbool.h"
#include "Motor.h"          /* DM_Motor_Info_Typedef */

/* 机械臂关节数量 */
#ifndef ARM_JOINT_NUM
#define ARM_JOINT_NUM  6
#endif

/**
 * @brief 机械臂关节控制结构体
 * @note  所有角度/速度统一使用弧度 (rad)，与达妙 MIT 协议保持一致。
 *        Target_Angle / Current_Angle 均为"关节坐标"（编码器角减零位偏移后的角度）。
 */
typedef struct
{
    // ---- 状态变量 ----
    float Target_Angle;           /*!< 目标角度（关节坐标，单位：rad）*/
    float Current_Angle;          /*!< 当前角度（关节坐标，单位：rad），编码器反馈减零位偏移 */
    float Current_Velocity;       /*!< 当前速度 （单位：rad/s），来自电机编码器反馈 */
    uint8_t Target_Initialized;   /*!< 目标角度是否已从有效反馈锁定（1=允许闭环控制）*/

    // ---- 关节限位 ----
    float Max_Angle;              /*!< 关节角度上限 （单位：rad）*/
    float Min_Angle;              /*!< 关节角度下限 （单位：rad）*/
    float Max_Velocity;           /*!< 关节最大速度 （单位：rad/s），预留暂未使用 */
    float Max_Torque;             /*!< 关节最大力矩 （单位：N·m），同时用作重力前馈输出限幅 */

    // ---- 达妙电机指针 ----
    DM_Motor_Info_Typedef *Motor; /*!< 指向该关节对应的达妙电机结构体 */
} Arm_Joint_Control_Typedef;

/**
 * @brief 关节参数配置结构体（专用于开头集中修改）
 * @note  所有角度/速度统一使用弧度 (rad)
 */
typedef struct {
    DM_Motor_Info_Typedef *motor_ptr; /*!< 电机指针 */
    float kp;                         /*!< 位置环 Kp */
    float kd;                         /*!< 速度环 Kd */
    float min_angle;                  /*!< 关节角度下限 (rad) */
    float max_angle;                  /*!< 关节角度上限 (rad) */
    float max_velocity;               /*!< 关节最大速度 (rad/s)，预留 */
    float max_torque;                 /*!< 关节最大力矩 (N·m)，用于重力前馈限幅 */
    uint8_t ctrl_type;                /*!< 控制类型：0=摇杆增量，1=拨杆增量 */
    int8_t  rc_ch;                    /*!< 摇杆/拨杆通道索引 */
    float   step;                     /*!< 摇杆满偏时的每周期增量 (rad)，或拨杆每周期增量 (rad) */
} Arm_Motor_Config_t;

/* 外部变量 ---------------------------------------------------------------- */

/** @brief 全局关节数组（由 Arm_Controller 单点拥有）*/
extern Arm_Joint_Control_Typedef Arm_Joints[ARM_JOINT_NUM];

/* 坐标转换（零位表单点拥有，编码器角 <-> 关节角）------------------------------- */
/**
 * @brief 编码器角 → 关节角（减零位偏移）
 * @param idx            关节索引 0~5
 * @param encoder_angle  编码器反馈角度 (rad)
 * @return 关节坐标角度 (rad)
 */
float Arm_Encoder_To_Joint(uint8_t idx, float encoder_angle);

/**
 * @brief 关节角 → 编码器角（加回零位偏移）
 * @param idx           关节索引 0~5
 * @param joint_angle   关节坐标角度 (rad)
 * @return 编码器坐标角度 (rad)
 */
float Arm_Joint_To_Encoder(uint8_t idx, float joint_angle);

/* 控制器接口 ---------------------------------------------------------------- */
/** @brief 初始化机械臂控制器（加载配置表/限位）*/
void Arm_Controller_Init(void);

/**
 * @brief 上电状态机推进（非阻塞，每控制周期调用一次）
 * @note  状态流：BOOT_ENABLE(使能+50ms) → WAIT_STABLE(反馈稳定检测+超时) → RUN
 *        配合 Arm_Controller_Is_Running() 使用：RUN 后才允许闭环控制。
 */
void Arm_Controller_Poll(void);

/**
 * @brief 查询上电状态机是否已进入正常运行态
 * @return 1=RUN（允许闭环控制），0=仍在启动流程
 */
uint8_t Arm_Controller_Is_Running(void);

/** @brief 读取各关节反馈（编码器角 → 关节角）*/
void Arm_Controller_Read_Feedback(void);

/** @brief 对所有关节尝试锁定目标（处理上电/晚到反馈）*/
void Arm_Controller_Try_Init_Target_All(void);

/** @brief 周期控制更新：急停/失联保护 → 重力补偿 → 六关节增量控制 → 夹爪*/
void Arm_Controller_Update(void);

/**
 * @brief 单关节安全下发（内部做 零位回加 + DM_Motor_Ctrl_Safe）
 * @param idx          关节索引 0~5
 * @param joint_angle  目标角度（关节坐标，rad）
 * @param kp           位置环 Kp
 * @param kd           速度环 Kd
 * @param torque       前馈力矩 (N·m)
 */
void Arm_Controller_Send_Joint(uint8_t idx, float joint_angle, float kp, float kd, float torque);

#endif /* ARM_CONTROLLER_H */
