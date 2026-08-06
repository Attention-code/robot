#include "main.h"
#include "cmsis_os.h"
#include "Arm_Task.h"
#include "Arm_Controller.h"   /* 控制器层：初始化/反馈/控制更新 */
#include "usart_printf_task.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/**
 * @brief 机械臂控制任务（200Hz，5ms 周期）
 * @note  控制逻辑全部由 Arm_Controller（Controller 层）承担，本任务仅做编排：
 *         初始化 → 上电启动 → 周期循环（读反馈 → 锁目标 → 控制更新 → 打印）
 * @param argument 未使用
 */
void Arm_Control_Task(void const * argument)
{
    (void)argument;

    Arm_Controller_Init();
 
    TickType_t Arm_Task_SysTick = osKernelSysTick();
    uint32_t print_cnt = 0;

    for (;;)
    {
        Arm_Controller_Read_Feedback();

        /* 上电状态机：使能电机 → 50ms → 反馈稳定检测 → RUN（非阻塞）*/
        Arm_Controller_Poll();

        /* RUN 后才允许闭环控制（增量控制/夹爪下发）*/
        if (Arm_Controller_Is_Running())
        {
            /* 处理启动后晚到的反馈：首次有效反馈时锁定目标并允许闭环 */
            Arm_Controller_Try_Init_Target_All();

            Arm_Controller_Update();
        }

/* 串口打印六个电机编码器值 + J1/J2 力矩（弧度 / N·m） */
        if (++print_cnt >= 100) /* 循环周期 5ms，100 次 = 500ms 打印一次 */
        {
            print_cnt = 0;
            usart_printf("Enc0:%.3f Enc1:%.3f Enc2:%.3f Enc3:%.3f Enc4:%.3f Enc5:%.3f | Tq1:%.3f Tq2:%.3f\r\n",
                         Arm_Joints[0].Motor ? Arm_Joints[0].Motor->Data.Position : 0.0f,
                         Arm_Joints[1].Motor ? Arm_Joints[1].Motor->Data.Position : 0.0f,
                         Arm_Joints[2].Motor ? Arm_Joints[2].Motor->Data.Position : 0.0f,
                         Arm_Joints[3].Motor ? Arm_Joints[3].Motor->Data.Position : 0.0f,
                         Arm_Joints[4].Motor ? Arm_Joints[4].Motor->Data.Position : 0.0f,
                         Arm_Joints[5].Motor ? Arm_Joints[5].Motor->Data.Position : 0.0f,
                         Arm_Joints[1].Motor ? Arm_Joints[1].Motor->Data.Torque : 0.0f,
                         Arm_Joints[2].Motor ? Arm_Joints[2].Motor->Data.Torque : 0.0f);
        }
        /* ============================ */

        osDelayUntil(&Arm_Task_SysTick, 5);
    }
}
