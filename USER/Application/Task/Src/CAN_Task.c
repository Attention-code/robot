#include "cmsis_os.h"
#include "CAN_Task.h"
#include "main.h"
#include "B2B_Communication.h"
#include "Control_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "fdcan.h"
#include "usart_printf_task.h"
#include "Ix6_Remote.h"

static void Motor_System_Boot(void);

void CAN_Task(void const * argument)
{
    // // 1. 【一键集成】：调用启动准备函数，搞定所有前戏
    // Motor_System_Boot();


    TickType_t CAN_Task_SysTick = osKernelSysTick();

    DM_Motor_Command(&FDCAN2_TxFrame,&DM_4310_Motor[0],Motor_Enable);
    osDelay(30);

    for(;;)
    {
        // 达妙 4310 控制
        float target_speed = 6.0f;
        float kd_value = 1.0f;
        DM_Motor_Ctrl_Safe(&FDCAN2_TxFrame, &DM_4310_Motor[0], 0, target_speed, 0.0f, kd_value, 0);

        // 大疆 GM6020 控制
        int16_t gm6020_voltage = 8000;
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, gm6020_voltage, gm6020_voltage, gm6020_voltage, gm6020_voltage);

        // if(CAN_Task_SysTick % 20 == 0)
        // {
        // usart_printf("DM_St:%d | DM_Vel:%.2f | DJI_Vel:%d\r\n",
        //              DM_4310_Motor.Data.State,
        //              DM_4310_Motor.Data.Velocity,
        //              DJI_GIMBAL_Motor[1].Data.Encoder);
        // }

        // if(CAN_Task_SysTick % 20 == 0)
        // {
        // // 打印遥控器摇杆与拨杆状态
        // usart_printf("RC | R_X:%d R_Y:%d | L_X:%d L_Y:%d | SW_L:%d SW_R:%d\r\n",
        //              remote_ctrl.rc.ch[0],  // 右摇杆左右 (Right X)
        //              remote_ctrl.rc.ch[1],  // 右摇杆上下 (Right Y)
        //              remote_ctrl.rc.ch[2],  // 左摇杆左右 (Left X)
        //              remote_ctrl.rc.ch[3],  // 左摇杆上下 (Left Y)
        //              remote_ctrl.rc.s[0],   // 左侧拨杆
        //              remote_ctrl.rc.s[1]);  // 右侧拨杆

        // usart_printf("%f,%f,%f\r\n", INS_Info.Yaw_Angle, INS_Info.Pitch_Angle, INS_Info.Roll_Angle);

        // usart_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",i6x_ctrl.ch[0],i6x_ctrl.ch[1],i6x_ctrl.ch[2],i6x_ctrl.ch[3],i6x_ctrl.ch[4],i6x_ctrl.ch[5],i6x_ctrl.s[0],i6x_ctrl.s[1],i6x_ctrl.s[2],i6x_ctrl.s[3]);
        // }

        osDelayUntil(&CAN_Task_SysTick, 5);
    }
}

// void CAN_Task(void const *argument) {
//     TickType_t CAN_Task_SysTick = osKernelSysTick();
//     // 调度计数器，用来实现分频
//     uint32_t loop_cnt = 0;
//     for (;;) {
//
//         if (loop_cnt % 10 == 0) // 假设 Task 周期 1ms
//         {
//             // #if (REMOTE_TYPE_SWITCH == REMOTE_TYPE_I6X)
//             // B2B_Send_Ix6Remote_Data();
//             // #endif
//
//             // //打印遥控器摇杆与拨杆状态
//             // usart_printf("RC | R_X:%d R_Y:%d | L_X:%d L_Y:%d | SW_L:%d SW_R:%d\r\n",
//             //                          remote_ctrl.rc.ch[0],  // 右摇杆左右 (Right X)
//             //                          remote_ctrl.rc.ch[1],  // 右摇杆上下 (Right Y)
//             //                          remote_ctrl.rc.ch[2],  // 左摇杆左右 (Left X)
//             //                          remote_ctrl.rc.ch[3],  // 左摇杆上下 (Left Y)
//             //                          remote_ctrl.rc.s[0],   // 左侧拨杆
//             //                          remote_ctrl.rc.s[1]);  // 右侧拨杆5
//
//             // usart_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//             //                             i6x_ctrl.ch[0], // 右摇杆左右 (Right X)
//             //                             i6x_ctrl.ch[1], // 左摇杆上下 (Left Y)
//             //                             i6x_ctrl.ch[2], // 右摇杆上下 (Right Y)
//             //                             i6x_ctrl.ch[3], // 左摇杆左右 (Left X)
//             //                             i6x_ctrl.ch[4], // vaa左旋
//             //                             i6x_ctrl.ch[5], // vbb右旋
//             //                             i6x_ctrl.s[0],  // 左1拨杆
//             //                             i6x_ctrl.s[1],  // 左2拨杆
//             //                             i6x_ctrl.s[2],  // 右1拨杆
//             //                             i6x_ctrl.s[3]); // 右2拨杆
//
//             // B2B_Send_DjRemote_Data();
//             B2B_Send_Ix6Remote_Data();
//         }
//
//         osDelayUntil(&CAN_Task_SysTick, 5);
//     }
// }

// 4310 和 6020 的启动前期准备工作
static void Motor_System_Boot(void)
{
    TickType_t boot_tick = osKernelSysTick();

    // 第一阶段 (1秒)：大疆解锁 + 达妙专注清错
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[0], 0xFB); // 狂发清错
        osDelayUntil(&boot_tick, 5);
    }

    // 第二阶段 (1秒)：大疆解锁 + 达妙专注失能喂狗 (分离指令，防止打架！)
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[0], Motor_Disable); // 喂狗进入就绪态
        osDelayUntil(&boot_tick, 5);
    }
}