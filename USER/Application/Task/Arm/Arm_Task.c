#include "main.h"
#include "cmsis_os.h"
#include "Arm_Task.h"           /* 自己 */
#include "Motor.h"              /* 达妙电机驱动 */
#include "Remote_Control.h"     /* SBUS遥控器（标准）*/
#include "Ix6_Remote.h"        /* FS-i6X 遥控器（专用解析）*/
#include "usart_printf_task.h"  /* 串口打印 */
#include "config.h"            /* Rad_to_angle, DegreesToRadians, VAL_LIMIT */
#include "Ramp.h"
#include "LPF.h"
#include "FreeRTOSConfig.h"
#include "task.h"
/**
 * @brief 六轴机械臂关节控制数组
 *        每个元素对应一个关节
 */
Arm_Joint_Control_Typedef Arm_Joints[ARM_JOINT_NUM];

/* 遥控器摇杆值范围 -660 ~ +660 */
#define RC_DEADBAND            10

/* ========================================================================= */
/* ======================== 1. 初始化机械臂控制 ============================ */
/* ========================================================================= */

static void Arm_Control_Init(void)
{
    Arm_Joints[0].Motor = &DM_4310_Motor[0];
    Arm_Joints[1].Motor = &DM_4310_Motor[1];
    Arm_Joints[2].Motor = &DM_4310_Motor[2];
    Arm_Joints[3].Motor = &DM_4340_Motor[0];
    Arm_Joints[4].Motor = &DM_8009_Motor[0];
   // Arm_Joints[5].Motor = &DM_8009_Motor[1];
    
  
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        Arm_Joints[i].Target_Angle = 0.0f;
        Arm_Joints[i].Current_Angle = 0.0f;
        Arm_Joints[i].Current_Velocity = 0.0f;
    }

     usart_printf("[Arm] Arm_Control_Init OK, %d joints initialized.\r\n", ARM_JOINT_NUM);
}

/* ========================================================================= */
/* ======================== 2. 读取各关节反馈 ============================== */
/* ========================================================================= */

static void Arm_Read_Feedback(void)
{
    /* 只读取前4个已绑定的关节，避免访问 NULL 指针 */
    for (uint8_t i = 0; i < 5; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;
        Arm_Joints[i].Current_Angle    = Arm_Joints[i].Motor->Data.Position * Rad_to_angle;
        Arm_Joints[i].Current_Velocity = Arm_Joints[i].Motor->Data.Velocity * Rad_to_angle;
    }
}

/* ========================================================================= */
/* ======================== 3. 控制解算 =================================== */
/* ========================================================================= */

static void Arm_Cascade_PID_Update(void)
{
    /* ---- 关节0、1（4310）：摇杆增量位置控制 ---- */
    for (uint8_t i = 0; i < 2; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;

        float delta = (float)i6x_ctrl.ch[i] / 660.0f * 0.01f;
        if (i6x_ctrl.ch[i] > -RC_DEADBAND && i6x_ctrl.ch[i] < RC_DEADBAND)
            delta = 0.0f;

        Arm_Joints[i].Target_Angle += delta;
        if (Arm_Joints[i].Target_Angle > 12.0f)  Arm_Joints[i].Target_Angle = 12.0f;
        if (Arm_Joints[i].Target_Angle < -12.0f) Arm_Joints[i].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[i].Motor,
            Arm_Joints[i].Target_Angle,
            0.0f,
            20.0f,
            0.5f,
            0.0f
        );
    }

    /* ---- 关节3（4340）：摇杆 ch[2] 增量控制 ---- */
    if (Arm_Joints[3].Motor != NULL)
    {
        float delta_3 = (float)i6x_ctrl.ch[2] / 660.0f * 0.02f;
        if (i6x_ctrl.ch[2] > -RC_DEADBAND && i6x_ctrl.ch[2] < RC_DEADBAND)
            delta_3 = 0.0f;

        Arm_Joints[3].Target_Angle += delta_3;
        if (Arm_Joints[3].Target_Angle > 12.0f)
            Arm_Joints[3].Target_Angle = 12.0f;
        if (Arm_Joints[3].Target_Angle < -12.0f)
            Arm_Joints[3].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[3].Motor,
            Arm_Joints[3].Target_Angle,
            0.0f,
            24.0f,
            0.8f,
            0.0f
        );
    }

    /* ---- 关节4（8009）：摇杆 ch[3] 增量控制 ---- */
    if (Arm_Joints[4].Motor != NULL)
    {
        float delta_4 = (float)i6x_ctrl.ch[3] / 660.0f * 0.02f;
        if (i6x_ctrl.ch[3] > -RC_DEADBAND && i6x_ctrl.ch[3] < RC_DEADBAND)
            delta_4 = 0.0f;

        Arm_Joints[4].Target_Angle += delta_4;
        if (Arm_Joints[4].Target_Angle > 12.0f)
            Arm_Joints[4].Target_Angle = 12.0f;
        if (Arm_Joints[4].Target_Angle < -12.0f)
            Arm_Joints[4].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[4].Motor,
            Arm_Joints[4].Target_Angle,
            0.0f,
            20.0f,
            0.5f,
            0.0f
        );
    }

    //  /* ---- 关节5（8009）：摇杆 ch[3] 增量控制 ---- */
    // if (Arm_Joints[5].Motor != NULL)
    // {
    //     float delta_5 = (float)i6x_ctrl.ch[3] / 660.0f * 0.02f;
    //     if (i6x_ctrl.ch[3] > -RC_DEADBAND && i6x_ctrl.ch[3] < RC_DEADBAND)
    //         delta_5 = 0.0f;

    //     Arm_Joints[5].Target_Angle += delta_5;
    //     if (Arm_Joints[5].Target_Angle > 12.0f)
    //         Arm_Joints[5].Target_Angle = 12.0f;
    //     if (Arm_Joints[5].Target_Angle < -12.0f)
    //         Arm_Joints[5].Target_Angle = -12.0f;

    //     DM_Motor_CAN_TxMessage(
    //         &FDCAN2_TxFrame,
    //         Arm_Joints[5].Motor,
    //         Arm_Joints[5].Target_Angle,
    //         0.0f,
    //         20.0f,
    //         0.5f,
    //         0.0f
    //     );
    // }
    /* ---- 关节2（4310）：拨杆 swa 平滑过渡 ---- */
    if (Arm_Joints[2].Motor != NULL)
    {
        static float smooth_target_2 = 0.0f;
        static uint8_t j2_init = 1;
        if (j2_init) {
            smooth_target_2 = Arm_Joints[2].Motor->Data.Position;
            j2_init = 0;
        }

        float desired_2 = (i6x_ctrl.s[0] == 1) ? 0.5f : 2.8f;

        const float max_step = 0.015f;
        if (desired_2 > smooth_target_2) {
            smooth_target_2 += max_step;
            if (smooth_target_2 > desired_2) smooth_target_2 = desired_2;
        } else if (desired_2 < smooth_target_2) {
            smooth_target_2 -= max_step;
            if (smooth_target_2 < desired_2) smooth_target_2 = desired_2;
        }

        Arm_Joints[2].Target_Angle = smooth_target_2;

        DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,
                               Arm_Joints[2].Motor,
                               Arm_Joints[2].Target_Angle,
                               0.0f,
                               8.0f,
                               0.5f,
                               0.0f);
    }
}

void Arm_Control_Task(void const * argument)
{
    Arm_Control_Init();

    TickType_t Arm_Task_SysTick = osKernelSysTick();

    /* 使能电机 */
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[0].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[1].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[2].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[3].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[4].Motor, Motor_Enable);
  //  DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[5].Motor, Motor_Enable);
    osDelay(200);

    /* 初始化各电机目标位置为当前位置 */
    for (uint8_t i = 0; i < 5; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;
        Arm_Joints[i].Target_Angle = Arm_Joints[i].Motor->Data.Position;
    }

    uint32_t print_cnt = 0;

    for (;;)
    {
        Arm_Read_Feedback();
        Arm_Cascade_PID_Update();

        if (++print_cnt >= 200)
        {
           print_cnt = 0;
           if (Arm_Joints[2].Motor != NULL)
            {
                usart_printf("V:%d ,P:%.2f\r\n",Arm_Joints[1].Current_Velocity, Arm_Joints[1].Motor->Data.Position);
            }
        }

       if (++print_cnt >= 1000)
        {
         print_cnt = 0;
         UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
         usart_printf("Stack free: %d\r\n", uxHighWaterMark);
        }

        osDelayUntil(&Arm_Task_SysTick, 5);
    }
}
