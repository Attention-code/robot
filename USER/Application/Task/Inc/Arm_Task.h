/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Task.h
  * @brief          : 六轴机械臂控制任务头文件
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : 控制逻辑已拆分至 Components/Controller/Arm_Controller.*，
  *                   本头文件仅保留任务入口，关节结构体/配置见 Arm_Controller.h
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "Arm_Controller.h"   /* Arm_Joints, ARM_JOINT_NUM 等由控制器层提供 */

/* 外部接口 */
extern void Arm_Control_Task(void const * argument);

#endif //ARM_TASK_H
