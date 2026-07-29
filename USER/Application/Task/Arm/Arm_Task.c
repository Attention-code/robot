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

/* ========================================================================= */
/* ======================== 0. 集中参数配置区 ============================ */
/* ========================================================================= */
/**
 * @note 你只需要在这里修改对应关节的 Kp 和 Kd 参数即可！
 *       顺序依次对应关节 0 到 关节 5。
 */
static Arm_Motor_Config_t Arm_Config_Table[ARM_JOINT_NUM] = {
    /* 电机指针                Kp     Kd   */
    { &DM_4310_Motor[0],    20.0f,  0.5f },  // 关节 0: 小臂俯仰 (DM-4310)
    { &DM_4310_Motor[1],     8.0f,  0.5f },  // 关节 1: 夹爪旋转 (DM-4310)
    { &DM_4310_Motor[2],     8.0f,  0.5f },  // 关节 2: 夹爪电机 (DM-4310)
    { &DM_4340_Motor[1],    15.0f,  0.8f },  // 关节 3: 小臂俯仰 (DM-4340)
    { &DM_4340_Motor[2],    15.0f,  0.8f },  // 关节 4: 小臂旋转 (DM-4340)
    { &DM_8009_Motor[0],    20.0f,  0.5f },  // 关节 5: 大臂俯仰 (DM-8009)
};

/**
 * @brief 六轴机械臂关节控制数组
 *        每个元素对应一个关节
 */
Arm_Joint_Control_Typedef Arm_Joints[ARM_JOINT_NUM];
/* 死区 */
#define RC_DEADBAND            1

/* ========================================================================= */
/* ======================== 1. 初始化机械臂控制 ============================ */
/* ========================================================================= */

static void Arm_Control_Init(void)
{
    Arm_Joints[0].Motor = &DM_4310_Motor[0];
    Arm_Joints[1].Motor = &DM_4310_Motor[1];
    Arm_Joints[2].Motor = &DM_4310_Motor[2];
    Arm_Joints[3].Motor = &DM_4340_Motor[1];
    Arm_Joints[4].Motor = &DM_4340_Motor[2];
    Arm_Joints[5].Motor = &DM_8009_Motor[0];
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
    /* 只读取前6个已绑定的关节，避免访问 NULL 指针 */
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;
        Arm_Joints[i].Current_Angle    = Arm_Joints[i].Motor->Data.Position ;
        Arm_Joints[i].Current_Velocity = Arm_Joints[i].Motor->Data.Velocity ;
    }
}

/* ========================================================================= */
/* ======================== 遥控器急停 ==================================== */
/* ========================================================================= */
/**
 * SWA0 = 1:
 *      所有关节保持当前位置
 *      不接受任何遥控输入
 *      电机保持使
 * SWA0 = 0:
 *      返回正常控制
 * @return 1 急停状态
 *         0 正常状态
 */
static uint8_t Arm_Hold_Position(void)
{
    static uint8_t emergency_lock = 0;
    if(i6x_ctrl.s[0] == 1)
    {
        if(emergency_lock == 0)
        {
            /* 第一次进入急停，锁当前位置 */
            for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
            {
                if(Arm_Joints[i].Motor == NULL)
                    continue;
                Arm_Joints[i].Target_Angle =
                    Arm_Joints[i].Current_Angle;
            }
            emergency_lock = 1;
        }
        /* 持续发送锁定目标 */
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            if(Arm_Joints[i].Motor == NULL)
                continue;
            DM_Motor_CAN_TxMessage(
                &FDCAN2_TxFrame,
                Arm_Joints[i].Motor,
                Arm_Joints[i].Target_Angle,
                0.0f,
                30.0f,
                0.8f,
                0.0f
            );
        }
        return 1;
    }
    else
    {
        emergency_lock = 0;
    }
    return 0;
}
/* ========================================================================= */
/* ======================== 3. 控制解算 =================================== */
/* ========================================================================= */
static void Arm_Cascade_PID_Update(void)
{
      /* ===================================================== */
    /* ================== 遥控急停 SWA0 =================== */
    /* ===================================================== */
  /* ================= 急停检测 ================= */
    if(Arm_Hold_Position())
    {
        return;
    }

 /* ---- 夹爪电机，关节2（4310）：拨杆 swa 平滑过渡 ---- */
    if (Arm_Joints[2].Motor != NULL)
    {
        static float smooth_target_2 = 0.0f;
        static uint8_t j2_init = 1;
        if (j2_init) {
            smooth_target_2 = Arm_Joints[2].Motor->Data.Position;
            j2_init = 0;
        }

        float desired_2 = (i6x_ctrl.s[3] == 1) ? 0.5f : 2.8f;

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
                               Arm_Config_Table[2].kp,
                               Arm_Config_Table[2].kd,
                               0.0f);
    }


   /* ----j6, 夹爪旋转电机 关节1（4310）：swa2 拨杆正反转 ---- */
    if (Arm_Joints[1].Motor != NULL)
    {
        float delta_1 = 0.0f;

        if (i6x_ctrl.s[2] == 1)
        {
            /* swa2 上拨 → 正转 */
            delta_1 = 0.002f;
        }
        else if (i6x_ctrl.s[2] == -1)
        {
            /* swa2 下拨 → 反转 */
            delta_1 = -0.002f;
        }
        /* s[2] == 0 (中位) → 保持不动 */

        Arm_Joints[1].Target_Angle += delta_1;
        if (Arm_Joints[1].Target_Angle > 12.0f)  Arm_Joints[1].Target_Angle = 12.0f;
        if (Arm_Joints[1].Target_Angle < -12.0f) Arm_Joints[1].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[1].Motor,
            Arm_Joints[1].Target_Angle,
            0.0f,
            Arm_Config_Table[1].kp,
            Arm_Config_Table[1].kd,
            0.0f
        );
    }

/* ----j5，小臂俯仰，关节0(4340)，通道0增量控制---- */
    if (Arm_Joints[0].Motor != NULL)
    {
        float delta = (float)i6x_ctrl.ch[0] / 660.0f * 0.01f;
        if (i6x_ctrl.ch[0] > -RC_DEADBAND && i6x_ctrl.ch[0] < RC_DEADBAND)
            delta = 0.0f;

        Arm_Joints[0].Target_Angle += delta;
        if (Arm_Joints[0].Target_Angle > 12.0f)  Arm_Joints[0].Target_Angle = 12.0f;
        if (Arm_Joints[0].Target_Angle < -12.0f) Arm_Joints[0].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[0].Motor,
            Arm_Joints[0].Target_Angle,
            0.0f,
            Arm_Config_Table[0].kp,
            Arm_Config_Table[0].kd,
            0.0f
        );
    }

/* ----j4, 小臂旋转电机，关节4（4340）：摇杆 ch[3] 增量控制 ---- */
    if (Arm_Joints[4].Motor != NULL)
    {
        float delta_4 = (float)i6x_ctrl.ch[3] / 660.0f * 0.005f;
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
            Arm_Config_Table[4].kp,
            Arm_Config_Table[4].kd,
            0.0f
        );
    }

/* ----j3, 小臂俯仰电机，关节3（4340）：摇杆 ch[2] 增量控制 ---- */
    if (Arm_Joints[3].Motor != NULL)
    {
        float delta_3 = (float)i6x_ctrl.ch[2] / 660.0f * 0.005f;
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
            Arm_Config_Table[3].kp,
            Arm_Config_Table[3].kd,
            0.0f
        );
    }

    /* ----j2, 大臂俯仰电机，关节5（8009）：摇杆 ch[1] 增量控制 ---- */
    if (Arm_Joints[5].Motor != NULL)
    {
        float delta_5 = (float)i6x_ctrl.ch[1] / 660.0f * 0.01f;
        if (i6x_ctrl.ch[1] > -RC_DEADBAND && i6x_ctrl.ch[1] < RC_DEADBAND)
            delta_5 = 0.0f;

        Arm_Joints[5].Target_Angle += delta_5;
        if (Arm_Joints[5].Target_Angle > 12.0f)
            Arm_Joints[5].Target_Angle = 12.0f;
        if (Arm_Joints[5].Target_Angle < -12.0f)
            Arm_Joints[5].Target_Angle = -12.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[5].Motor,
            Arm_Joints[5].Target_Angle,
            0.0f,
            Arm_Config_Table[5].kp,
            Arm_Config_Table[5].kd,
            0.0f
        );
    }
}

void Arm_Control_Task(void const * argument)
{
    Arm_Control_Init();

    TickType_t Arm_Task_SysTick = osKernelSysTick();

    /* 初始化各电机目标位置为当前位置（必须在使能前，但使能前没有有效反馈） */
    /* 这里只做 Target_Angle 初始化，实际有效值在使能后第一次循环才设置 */

    /* 使能电机 */
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[0].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[1].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[2].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[3].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[4].Motor, Motor_Enable);
    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[5].Motor, Motor_Enable);
    osDelay(50);

    /* 使能后延迟50ms，等待电机反馈数据稳定，再锁定当前位置为目标位置 */
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;
         Arm_Joints[i].Target_Angle = Arm_Joints[i].Motor->Data.Position;
    }

    uint32_t print_cnt = 0;

    for (;;)
    {
        Arm_Read_Feedback();
        Arm_Cascade_PID_Update();

/* ===== 新增：串口打印电机位置 ===== */
        if (++print_cnt >= 100) // 5ms * 200 = 1000ms (1秒) 打印一次
        {
         
            usart_printf("Tar: %.3f | Cur: %.3f\r\n",Arm_Joints[0].Target_Angle, Arm_Joints[0].Current_Angle);
        }
        /* ================================== */

        osDelayUntil(&Arm_Task_SysTick, 5);
    }
}
