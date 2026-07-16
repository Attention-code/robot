
#include "main.h"
#include "cmsis_os.h"
#include "Arm_Task.h"           /* 自己 */
#include "Motor.h"              /* 达妙电机驱动 */
#include "Remote_Control.h"     /* SBUS遥控器（标准）*/
#include "Ix6_Remote.h"        /* FS-i6X 遥控器（专用解析）*/
#include "usart_printf_task.h"  /* 串口打印 */
#include "config.h"            /* Rad_to_angle, DegreesToRadians, VAL_LIMIT */

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

/**
 * @brief  初始化所有关节的PID参数、限位、电机指针
 * @note   此函数在任务启动时执行一次
 *
 *   使用说明：
 *     1. 各关节的电机指针需要在下面手动绑定到实际的DM_Motor_Info_Typedef
 *     2. PID参数和限位可在初始化后单独修改覆盖
 *     3. 角度限位必须根据实际机械结构设置，防止自撞
 */
static void Arm_Control_Init(void)
{
    Arm_Joints[0].Motor = &DM_4310_Motor[0];
    Arm_Joints[1].Motor = &DM_4310_Motor[1];
    Arm_Joints[2].Motor = &DM_4310_Motor[2];
    Arm_Joints[3].Motor = &DM_4310_Motor[3];
    Arm_Joints[4].Motor = &DM_4310_Motor[4];
    Arm_Joints[5].Motor = &DM_4310_Motor[5];

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

/**
 * @brief  从达妙电机反馈数据中读取各关节的当前角度和速度
 * @note   需要在每次控制循环开始时调用
 *         达妙电机的反馈数据由 CAN_Task 或 CAN 中断自动更新到 Motor->Data 中
 */
static void Arm_Read_Feedback(void)
{
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        /* 从达妙电机结构体中读取反馈数据 */
        /* Motor->Data.Position  单位：弧度（根据你的 Param_Range.P_MAX 决定范围）*/
        /* Motor->Data.Velocity  单位：弧度/秒 */
        Arm_Joints[i].Current_Angle    = Arm_Joints[i].Motor->Data.Position * Rad_to_angle;    /* 弧度 → 度 */
        Arm_Joints[i].Current_Velocity = Arm_Joints[i].Motor->Data.Velocity * Rad_to_angle;    /* 弧度/秒 → 度/秒 */
    }
}

/* ========================================================================= */
/* ======================== 3. 控制解算 =================================== */
/* ========================================================================= */

static void Arm_Cascade_PID_Update(void)
{
    /* 三电机 MIT 位置模式 */
    for (uint8_t i = 0; i < 3; i++)
    {
        float delta = (float)i6x_ctrl.ch[i] / 660.0f * 0.01f;

        /* 死区 */
        if (i6x_ctrl.ch[i] > -10 && i6x_ctrl.ch[i] < 10)
            delta = 0.0f;

        /* 累加目标位置，限幅 ±12.0 弧度 */
        Arm_Joints[i].Target_Angle += delta;
        if (Arm_Joints[i].Target_Angle > 12.0f)
            Arm_Joints[i].Target_Angle = 12.0f;
        if (Arm_Joints[i].Target_Angle < -12.0f)
            Arm_Joints[i].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[i].Motor,
            Arm_Joints[i].Target_Angle, /* 目标位置（弧度）*/
            0.0f,          /* 速度 */
            24.0f,         /* KP */
            0.8f,          /* KD */
            0.0f           /* 力矩 */
        );
    }
}       
/* ========================================================================= */
/* ======================== 6. 调试打印 ==================================== */
/* ========================================================================= */

/**
 * @brief  打印各关节的调试信息到串口
 * @note   可通过遥控器开关或宏定义启停，避免频繁打印影响控制周期
 */
#define ARM_DEBUG_PRINT_ENABLE   1   /* 设为0可关闭打印 */

static void Arm_Print_Debug(void)
{
#if ARM_DEBUG_PRINT_ENABLE
    static uint8_t cnt = 0;
    if (cnt++ >= 20)  /* 每20次(100ms)打印一次 */
    {
        cnt = 0;
        usart_printf("J0:St%d P%.2f V%.2f T%.2f | J1:St%d P%.2f V%.2f T%.2f | J2:St%d P%.2f V%.2f T%.2f\r\n",
                     Arm_Joints[0].Motor->Data.State, Arm_Joints[0].Motor->Data.Position, Arm_Joints[0].Motor->Data.Velocity, Arm_Joints[0].Motor->Data.Torque,
                     Arm_Joints[1].Motor->Data.State, Arm_Joints[1].Motor->Data.Position, Arm_Joints[1].Motor->Data.Velocity, Arm_Joints[1].Motor->Data.Torque,
                     Arm_Joints[2].Motor->Data.State, Arm_Joints[2].Motor->Data.Position, Arm_Joints[2].Motor->Data.Velocity, Arm_Joints[2].Motor->Data.Torque);
    }
#endif
}

void Arm_Control_Task(void const * argument)
{
    Arm_Control_Init();

    TickType_t Arm_Task_SysTick = osKernelSysTick();

    /* 使能三个电机 */
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[0].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[1].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[2].Motor, Motor_Enable);
    osDelay(200);  /* 等所有电机回传数据稳定 */

    /* 初始化各电机目标位置为当前位置 */
    for (uint8_t i = 0; i < 3; i++)
        Arm_Joints[i].Target_Angle = Arm_Joints[i].Motor->Data.Position;

    uint32_t print_cnt = 0;

    for (;;)
    {
        Arm_Read_Feedback();
        Arm_Cascade_PID_Update();

        /* 每 200 次（1秒）打印一次 */
        if (++print_cnt >= 200)
        {
            print_cnt = 0;
            usart_printf("J0:St%d P%.2f | J1:St%d P%.2f | J2:St%d P%.2f\r\n",
                         Arm_Joints[0].Motor->Data.State, Arm_Joints[0].Motor->Data.Position,
                         Arm_Joints[1].Motor->Data.State, Arm_Joints[1].Motor->Data.Position,
                         Arm_Joints[2].Motor->Data.State, Arm_Joints[2].Motor->Data.Position);
        }

        osDelayUntil(&Arm_Task_SysTick, 5);
    }
}
