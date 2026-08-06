/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Controller.c
  * @brief          : 六轴机械臂控制器（Controller 层）实现
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : 本模块为机械臂控制逻辑的"单点拥有"：
  *                   - 配置表 / 零位表 / 重力补偿表
  *                   - 坐标转换（编码器角 <-> 关节角，零位偏移）
  *                   - 重力补偿 / 增量控制 / 夹爪 / 急停失联保护
  *                   Task 层（Arm_Task.c）只负责调用本模块接口做编排。
  *                   所有角度统一弧度 (rad)，与达妙 MIT 协议一致。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Arm_Controller.h"
#include "Ix6_Remote.h"        /* i6x_ctrl */
#include "config.h"            /* VAL_LIMIT */
#include "usart_printf_task.h" /* usart_printf */
#include "cmsis_os.h"          /* osKernelSysTick / osDelay（启动与失联计时）*/
#include <string.h>            /* memset */
#include <math.h>

/* 集中参数配置区 ------------------------------------------------------------ */
/* 关节配置表：电机指针 / Kp / Kd / 限位 / 最大力矩 / 控制类型 / 通道 / 步进 */
static const Arm_Motor_Config_t Arm_Config_Table[ARM_JOINT_NUM] = 
{
    /* 电机指针                Kp     Kd     min   max   maxVel maxTrq ctrl ch  step   */
    { &DM_4340_Motor[0],    20.0f,  0.8f, -12.0f, 12.0f, 0.0f,   9.0f, 0,   4, 0.005f },  // J0 底座旋转 (DM-4340P)
    { &DM_8009_Motor[0],    35.0f,  1.5f, -12.0f, 12.0f, 0.0f,  20.0f, 0,   1, 0.010f },  // J1 大臂俯仰 (DM-8009)
    { &DM_4340_Motor[1],    20.0f,  0.8f, -12.0f, 12.0f, 0.0f,   9.0f, 0,   2, 0.005f },  // J2 小臂俯仰 (DM-4340)
    { &DM_4340_Motor[2],    15.0f,  0.8f, -12.0f, 12.0f, 0.0f,   9.0f, 0,   3, 0.005f },  // J3 小臂旋转 (DM-4340)
    { &DM_4310_Motor[0],    20.0f,  0.5f, -12.0f, 12.0f, 0.0f,   3.0f,  0,   0, 0.010f },  // J4 腕俯仰   (DM-4310)
    { &DM_4310_Motor[1],     8.0f,  0.5f, -12.0f, 12.0f, 0.0f,   3.0f,  1,   2, 0.002f },  // J5 腕旋转   (DM-4310, 拨杆)
};

/**
 * @brief 各关节零位对应的编码器原始值（弧度，来自达妙电机反馈 Position）
 * @note  关节角   = 编码器角 - Zero_Encoder_Value   （反馈/目标/重力补偿/运动学均用关节角）
 *          编码器角 = 关节角 + Zero_Encoder_Value     （下发电机指令时回加零位）
 *        Target_Angle / Current_Angle 统一为关节坐标（弧度）。
 *        当前标定基准：整个机械臂竖直状态为关节角 0
 *        J0 零位当前为 0.0f（待标定后填入）。
 */
static const float Arm_Zero_Encoder_Table[ARM_JOINT_NUM] = {
     0.0f,       /* J0 底座旋转（待标定）*/
    -2.043000f,  /* J1 大臂俯仰*/
     3.099000f,  /* J2 小臂俯仰*/
     2.537000f,  /* J3 小臂旋转*/
    -0.511000f,  /* J4 腕俯仰*/
    -2.663000f,  /* J5 腕旋转*/
};

/* 坐标转换：编码器角 <-> 关节角*/
float Arm_Encoder_To_Joint(uint8_t idx, float encoder_angle)
{
    if (idx >= ARM_JOINT_NUM) return encoder_angle;
    return encoder_angle - Arm_Zero_Encoder_Table[idx];
}

float Arm_Joint_To_Encoder(uint8_t idx, float joint_angle)
{
    if (idx >= ARM_JOINT_NUM) return joint_angle;
    return joint_angle + Arm_Zero_Encoder_Table[idx];
}

/**
 * @brief 六轴机械臂关节控制数组
 *        每个元素对应一个关节
 */
Arm_Joint_Control_Typedef Arm_Joints[ARM_JOINT_NUM];

/* 摇杆死区（i6x 摇杆通道范围 ±660）*/
#define RC_DEADBAND         20
#define I6X_CH_RANGE        660.0f

/* 遥控器失联判定超时 (ms)：超过该时间未收到新遥控帧即判定失联 */
#define RC_LOST_TIMEOUT_MS  20000

/* ==================== 重力补偿配置（已启用） ==================== */
#if 1
/**
 * @brief 重力补偿配置（索引对应关节 0~5）
 * @note  补偿力矩 = sign * Σ( coeff[k] * sin(angle_expr[k]) )
 *        angle_expr: 0=θ1(大臂), 1=θ2(小臂), 2=θ1+θ2(小臂相对竖直)
 *        角度基准：零位=竖直（θ=0 时重力矩为 0，水平时最大）
 *        系数单位 N·m，按机械臂质量/质心标定：G = m * g * L_com
 *        输出限幅统一使用关节 Max_Torque（见 Arm_Config_Table.max_torque）
 */
typedef struct {
    float   sign;          /*!< 方向符号 +1/-1（按电机实际转向调整）*/
    uint8_t term_num;      /*!< 使用的补偿项数 (0~2) */
    float   coeff[2];      /*!< 各项系数 (N·m) */
    int8_t  angle_expr[2]; /*!< 各项角度表达式索引 */
} Arm_Gravity_Config_t;

/* 重力补偿低通滤波系数（200Hz 下 0.05 ≈ 截止 1.6Hz）*/
#define GRAVITY_LPF_ALPHA   0.05f

static const Arm_Gravity_Config_t Arm_Gravity_Table[ARM_JOINT_NUM] = {
    /* sign  term  coeff[]        angle_expr[] */
    {  1.0f, 0, { 0.0f,  0.0f }, { 0, 0 } },  /* J0 底座旋转：无补偿 */
    { -1.0f, 2, {20.0f,  2.0f }, { 0, 2 } },  /* J1 大臂俯仰：20·sinθ1 + 2·sin(θ1+θ2) */
    {  1.0f, 1, { 3.68f, 0.0f }, { 2, 0 } },  /* J2 小臂俯仰：3.68·sin(θ1+θ2) */
    {  1.0f, 0, { 0.0f,  0.0f }, { 0, 0 } },  /* J3 小臂旋转：无补偿 */
    {  1.0f, 0, { 0.0f,  0.0f }, { 0, 0 } },  /* J4 腕俯仰：无补偿 */
    {  1.0f, 0, { 0.0f,  0.0f }, { 0, 0 } },  /* J5 腕旋转：无补偿 */
};
#endif /* 重力补偿配置 */

/* 重力补偿计算函数前向声明（供 Arm_Hold_Position 在定义前调用）*/
static void Arm_Calc_Gravity_Torque(float gravity_torque[ARM_JOINT_NUM]);

/* ===== 遥控通道数量===== */
#define I6X_CH_NUM  6
#define I6X_SW_NUM  4

/* ===== 上电反馈稳定检测参数 ===== */
/* 循环周期 5ms：20 帧 = 100ms 连续稳定；位置变化阈值 0.01 rad；总超时 1000ms */
#define FEEDBACK_STABLE_FRAMES    20
#define FEEDBACK_STABLE_TOL       0.01f
#define FEEDBACK_WAIT_TIMEOUT_MS  1000u

/* ===== 上电状态机 ===== */
typedef enum
{
    ARM_CTRL_STATE_IDLE = 0,      /*!< 未开始 */
    ARM_CTRL_STATE_BOOT_ENABLE,   /*!< 使能电机 + 50ms 等待 */
    ARM_CTRL_STATE_WAIT_STABLE,   /*!< 反馈稳定检测（带超时）*/
    ARM_CTRL_STATE_RUN,           /*!< 正常运行（允许闭环）*/
} Arm_Ctrl_State_e;

static Arm_Ctrl_State_e arm_state = ARM_CTRL_STATE_IDLE;
static uint32_t arm_state_tick = 0;           /*!< 当前状态起始时刻 (ms) */
static uint8_t  arm_enable_sent = 0;          /*!< BOOT_ENABLE 是否已发送使能指令 */
static float    stable_last_pos[ARM_JOINT_NUM] = {0.0f}; /*!< 上一帧位置（关节坐标）*/
static uint8_t  stable_cnt[ARM_JOINT_NUM] = {0};         /*!< 连续稳定帧计数 */
static uint8_t  stable_seen[ARM_JOINT_NUM] = {0};        /*!< 首帧基准标志 */


/* ======================== 1. 初始化机械臂控制 ============================ */
void Arm_Controller_Init(void)
{
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        Arm_Joints[i].Motor            = Arm_Config_Table[i].motor_ptr;
        Arm_Joints[i].Target_Angle     = 0.0f;
        Arm_Joints[i].Current_Angle    = 0.0f;
        Arm_Joints[i].Current_Velocity = 0.0f;
        Arm_Joints[i].Target_Initialized = 0;   /* 上电未锁定目标，禁止闭环 */

        /* 关节限位由配置表同步 */
        Arm_Joints[i].Min_Angle    = Arm_Config_Table[i].min_angle;
        Arm_Joints[i].Max_Angle    = Arm_Config_Table[i].max_angle;
        Arm_Joints[i].Max_Velocity = Arm_Config_Table[i].max_velocity;
        Arm_Joints[i].Max_Torque   = Arm_Config_Table[i].max_torque;
    }

    /* 上电状态机复位 */
    arm_state = ARM_CTRL_STATE_IDLE;

    usart_printf("[Arm] Arm_Controller_Init OK, %d joints initialized.\r\n", ARM_JOINT_NUM);
}

/* ======================== 2. 读取各关节反馈 ============================== */
void Arm_Controller_Read_Feedback(void)
{
    /* 只读取前6个已绑定的关节，避免访问 NULL 指针 */
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        if (Arm_Joints[i].Motor == NULL) continue;
        /* 编码器角 → 关节角（减零位偏移）*/
        Arm_Joints[i].Current_Angle    = Arm_Encoder_To_Joint(i, Arm_Joints[i].Motor->Data.Position);
        Arm_Joints[i].Current_Velocity = Arm_Joints[i].Motor->Data.Velocity;
    }
}

/* ======================== 通用电机辅助 =================================== */
/**
 * @brief 判断电机是否有有效反馈（已收到反馈且非错误状态）
 * @param m 电机指针
 * @return 1 有效  0 无效
 */
static uint8_t Arm_Motor_Has_Valid_Feedback(DM_Motor_Info_Typedef *m)
{
    if (m == NULL) return 0;
    /* 排除 DM_STATE_NONE(0x0 无反馈) 与 0x8~0xD 错误状态 */
    return (m->Data.State != DM_STATE_NONE &&
            !DM_Motor_Is_Fault_State(m->Data.State));
}

/* ======================== 目标初始化（上电/晚到反馈）===================== */
/**
 * @brief 首次获得有效反馈时锁定当前位置并允许闭环
 * @note  反馈无效时保持 Target_Initialized=0，禁止闭环，避免把目标拉到 0/旧值造成突跳
 * @param idx 关节索引 0~5
 */
static void Arm_Joint_Try_Init_Target(uint8_t idx)
{
    Arm_Joint_Control_Typedef *joint = &Arm_Joints[idx];

    if (joint->Target_Initialized) return;
    if (joint->Motor == NULL) return;

    if (Arm_Motor_Has_Valid_Feedback(joint->Motor))
    {
        /* 锁定关节角目标（编码器角减零位），Target_Angle 统一为关节坐标 */
        joint->Target_Angle = Arm_Encoder_To_Joint(idx, joint->Motor->Data.Position);
        joint->Target_Initialized = 1;
    }
}

void Arm_Controller_Try_Init_Target_All(void)
{
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        Arm_Joint_Try_Init_Target(i);
    }
}

/* ======================== 遥控器失联检测 ================================= */
/**
 * @brief 检测遥控器是否失联
 * @note  通过 i6x_ctrl.frame_cnt（每收到一帧合法 SBUS 自增）判断：
 *        超过 RC_LOST_TIMEOUT_MS 未收到新帧即判定失联。
 * @return 1 失联  0 正常
 */
static uint8_t Arm_Remote_Lost(void)
{
    static uint32_t last_frame_cnt = 0;
    static uint32_t last_good_tick = 0;
    static uint8_t  initialized    = 0;

    if (!initialized)
    {
        /* 首次调用以当前时刻为基准，避免上电瞬间/遥控任务启动稍慢时误判失联 */
        last_good_tick = osKernelSysTick();
        initialized = 1;
        return 0;
    }

    if (i6x_ctrl.frame_cnt != last_frame_cnt)
    {
        last_frame_cnt = i6x_ctrl.frame_cnt;
        last_good_tick = osKernelSysTick();
        return 0;
    }

    return ((osKernelSysTick() - last_good_tick) > RC_LOST_TIMEOUT_MS) ? 1 : 0;
}
                                            
/* ========================================================================= */
/* ======================== 遥控器急停 / 失联保护 ========================= */
/* ========================================================================= */
/**
 * SWA0 = 1 或 遥控失联：
 *      所有关节保持当前位置
 *      不接受任何遥控输入
 * SWA0 = 0 且遥控正常：
 *      返回正常控制
 * @return 1 急停/保护状态
 *         0 正常状态
 */
static uint8_t Arm_Hold_Position(void)
{
    static uint8_t emergency_lock = 0;
    uint8_t hold = (i6x_ctrl.s[0] == 1) || Arm_Remote_Lost();

    if (hold)
    {
        if (emergency_lock == 0)
        {
            /* 第一次进入保护，锁当前位置（仅已初始化关节，避免锁到陈旧值）*/
            for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
            {
                if (Arm_Joints[i].Motor == NULL)
                    continue;
                if (!Arm_Joints[i].Target_Initialized)
                    continue;
                Arm_Joints[i].Target_Angle = Arm_Joints[i].Current_Angle;
            }
            emergency_lock = 1;
        }
        float gravity_torque[ARM_JOINT_NUM];
        Arm_Calc_Gravity_Torque(gravity_torque);

        /* 持续发送锁定目标（统一走 Arm_Controller_Send_Joint，含零位回加 + 安全下发）*/
        for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            if (Arm_Joints[i].Motor == NULL)
                continue;
            if (!Arm_Joints[i].Target_Initialized)
                continue;   /* 未锁定目标，不参与保持 */
            Arm_Controller_Send_Joint(i,
                                      Arm_Joints[i].Target_Angle,
                                      Arm_Config_Table[i].kp,
                                      Arm_Config_Table[i].kd,
                                      gravity_torque[i]);
        }
        return 1;
    }
    else
    {
        emergency_lock = 0;
    }
    return 0;
}
  
/* 重力补偿计算函数（已启用，与 Arm_Gravity_Table 配套） */
#if 1
/**
 * @brief  计算各关节所需的重力补偿力矩（前馈）
 * @note   假设角度单位为弧度，θ=0 时机械臂竖直（重力矩为 0，水平时最大）。
 *         系数/符号/限幅统一由 Arm_Gravity_Table 配置。
 * @param  gravity_torque: 输出，长度为 ARM_JOINT_NUM，单位 N·m
 */
static void Arm_Calc_Gravity_Torque(float gravity_torque[ARM_JOINT_NUM])
{
    /* 低通滤波历史值（静态保持） */
    static float filtered[ARM_JOINT_NUM] = {0.0f};

    /* 初始化所有关节重力补偿为 0 */
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        gravity_torque[i] = 0.0f;
    }

    /* 获取关节角（单位：弧度，已减零位偏移，Current_Angle 为关节坐标）*/
    float theta1 = Arm_Joints[1].Current_Angle;  /* 大臂俯仰关节角（肩关节） */
    float theta2 = Arm_Joints[2].Current_Angle;  /* 小臂俯仰关节角（肘关节） */

    /* 角度表达式查表：0=θ1, 1=θ2, 2=θ1+θ2(小臂相对竖直) */
    float angle_expr_val[3] = { theta1, theta2, theta1 + theta2 };

    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
    {
        const Arm_Gravity_Config_t *g = &Arm_Gravity_Table[j];

        if (g->term_num == 0)
        {
            filtered[j] = 0.0f;
            continue;
        }

        /* 边界保护：最多 2 项 */
        uint8_t term_num = (g->term_num > 2u) ? 2u : g->term_num;

        /* 按配置表叠加各项补偿 */
        float raw = 0.0f;
        for (uint8_t k = 0; k < term_num; k++)
        {
            int8_t expr = g->angle_expr[k];
            if (expr < 0 || expr > 2) continue;   /* 边界保护：无效角度表达式跳过 */
            raw += g->coeff[k] * sinf(angle_expr_val[expr]);   /* 竖直零位：sin */
        }
        raw *= g->sign;

        /* 低通滤波 + 限幅（限幅统一使用关节 Max_Torque，与配置表一致）*/
        filtered[j] = GRAVITY_LPF_ALPHA * raw
                    + (1.0f - GRAVITY_LPF_ALPHA) * filtered[j];
        VAL_LIMIT(filtered[j], -Arm_Joints[j].Max_Torque, Arm_Joints[j].Max_Torque);

        gravity_torque[j] = filtered[j];
    }
}
#endif /* Arm_Calc_Gravity_Torque */

/* ========================================================================= */
/* ======================== 4. 单关节增量控制 ============================== */
/* ========================================================================= */
/**
 * @brief 单个关节的遥控增量控制（摇杆或拨杆）
 * @param idx            关节索引 0~5
 * @param gravity_torque 重力前馈力矩数组（长度 ARM_JOINT_NUM，单位 N·m）
 */
static void Arm_Joint_Incremental_Control(uint8_t idx, const float gravity_torque[ARM_JOINT_NUM])
{
    Arm_Joint_Control_Typedef *joint = &Arm_Joints[idx];
    const Arm_Motor_Config_t  *cfg   = &Arm_Config_Table[idx];

    if (joint->Motor == NULL) return;

    /* 未锁定有效目标前禁止闭环，避免把目标拉到 0/旧值 */
    if (!joint->Target_Initialized) return;

    float delta = 0.0f;

    if (cfg->ctrl_type == 0)
    {
        /* ---- 摇杆增量控制 ---- */
        if (cfg->rc_ch < 0 || cfg->rc_ch >= I6X_CH_NUM) return;  /* 通道边界保护 */
        float stick = (float)i6x_ctrl.ch[cfg->rc_ch];
        if (stick > -RC_DEADBAND && stick < RC_DEADBAND)
        {
            delta = 0.0f;
        }
        else
        {
            /* 归一化死区补偿：将 (死区, 满偏] 映射到 (0, 1]，避免刚出死区时突跳 */
            float norm = (stick > 0.0f)
                       ? (stick - (float)RC_DEADBAND) / (I6X_CH_RANGE - (float)RC_DEADBAND)
                       : (stick + (float)RC_DEADBAND) / (I6X_CH_RANGE - (float)RC_DEADBAND);
            delta = norm * cfg->step;
        }
    }
    else
    {
        /* ---- 拨杆增量控制 ---- */
        if (cfg->rc_ch < 0 || cfg->rc_ch >= I6X_SW_NUM) return;  /* 通道边界保护 */
        if (i6x_ctrl.s[cfg->rc_ch] == 1)       delta =  cfg->step;
        else if (i6x_ctrl.s[cfg->rc_ch] == -1) delta = -cfg->step;
        /* 中位 → 保持不动 */
    }

    /* 累加目标角度并统一限幅（使用每关节机械限位）*/
    joint->Target_Angle += delta;
    VAL_LIMIT(joint->Target_Angle, joint->Min_Angle, joint->Max_Angle);

    /* 统一走 Arm_Controller_Send_Joint（零位回加 + DM_Motor_Ctrl_Safe 安全保护）*/
    Arm_Controller_Send_Joint(idx, joint->Target_Angle, cfg->kp, cfg->kd, gravity_torque[idx]);
}

/*  夹爪控制= */
/**
 * @brief 夹爪控制（独立于 Arm_Joints，但纳入安全状态）
 * @param safe_hold 1=安全保持（急停/失联），0=正常跟随拨杆
 */
static void Arm_Gripper_Control(uint8_t safe_hold)
{
    static float   smooth_target_grip = 0.0f;
    static uint8_t grip_init = 1;   /* 1=尚未从有效反馈锁定目标 */

    /* 首次获得有效反馈时才锁定当前位置，避免用无效值(0)初始化后突跳 */
    if (grip_init)
    {
        if (Arm_Motor_Has_Valid_Feedback(&DM_4310_Motor[2]))
        {
            smooth_target_grip = DM_4310_Motor[2].Data.Position;
            grip_init = 0;
        }
        else
        {
            return;   /* 反馈无效或故障：不发送位置指令 */
        }
    }

    if (!safe_hold)
    {
        /* 正常模式：swa3 拨杆平滑过渡 */
        float desired_grip = (i6x_ctrl.s[3] == 1) ? 0.5f : 2.8f;
        const float max_step = 0.015f;
        if (desired_grip > smooth_target_grip)
        {
            smooth_target_grip += max_step;
            if (smooth_target_grip > desired_grip) smooth_target_grip = desired_grip;
        }
        else if (desired_grip < smooth_target_grip)
        {
            smooth_target_grip -= max_step;
            if (smooth_target_grip < desired_grip) smooth_target_grip = desired_grip;
        }
    }
    /* 安全模式：smooth_target_grip 保持当前值 */

    DM_Motor_Ctrl_Safe(&FDCAN2_TxFrame,
                       &DM_4310_Motor[2],
                       smooth_target_grip,
                       0.0f,
                       8.0f,
                       0.5f,
                       0.0f);
}

/* 控制解算 */

void Arm_Controller_Update(void)
{
    /*急停 / 遥控失联保护 = */
    if (Arm_Hold_Position())
    {
        /* 夹爪同步保持，防止安全状态下夹爪仍动作 */
        Arm_Gripper_Control(1);
        return;
    }

    /* ============ 重力补偿计算 ============ */
    float gravity_torque[ARM_JOINT_NUM];
    Arm_Calc_Gravity_Torque(gravity_torque);

    /* 六关节增量控制  */
    for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    {
        Arm_Joint_Incremental_Control(i, gravity_torque);
    }

    /* 夹爪正常控制 */
    Arm_Gripper_Control(0);
}


/*  上电状态机（非阻塞）= */
void Arm_Controller_Poll(void)
{
    uint32_t now = osKernelSysTick();

    switch (arm_state)
    {
    case ARM_CTRL_STATE_IDLE:
        /* 首次调用：进入使能阶段 */
        arm_state = ARM_CTRL_STATE_BOOT_ENABLE;
        arm_state_tick = now;
        arm_enable_sent = 0;
        break;

    case ARM_CTRL_STATE_BOOT_ENABLE:
        /* 发送一次使能指令，然后等待 50ms */
        if (!arm_enable_sent)
        {
            for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
            {
                if (Arm_Joints[i].Motor != NULL)
                    DM_Motor_Command(&FDCAN2_TxFrame, Arm_Joints[i].Motor, Motor_Enable);
            }
            arm_enable_sent = 1;
            arm_state_tick = now;
        }
        if ((now - arm_state_tick) >= 50u)
        {
            /* 进入反馈稳定检测阶段 */
            arm_state = ARM_CTRL_STATE_WAIT_STABLE;
            arm_state_tick = now;
            memset(stable_last_pos, 0, sizeof(stable_last_pos));
            memset(stable_cnt, 0, sizeof(stable_cnt));
            memset(stable_seen, 0, sizeof(stable_seen));
        }
        break;

    case ARM_CTRL_STATE_WAIT_STABLE:
    {
        /* 连续多帧位置变化小于阈值才锁定目标（带超时保护）*/
        uint8_t all_stable = 1;

        for (uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            if (Arm_Joints[i].Motor == NULL) continue;

            /* 无有效反馈或处于错误状态：重新累积 */
            if (!Arm_Motor_Has_Valid_Feedback(Arm_Joints[i].Motor))
            {
                stable_cnt[i] = 0;
                stable_seen[i] = 0;
                all_stable = 0;
                continue;
            }

            /* 统一转为关节坐标（减零位），与 Current_Angle 语义一致；
             * 稳定性判断只依赖相邻帧差值，常量偏移不影响结果 */
            float pos = Arm_Encoder_To_Joint(i, Arm_Joints[i].Motor->Data.Position);
            if (!stable_seen[i])
            {
                /* 第一帧只记录基准位置，不算稳定帧 */
                stable_seen[i] = 1;
                stable_cnt[i] = 0;
                all_stable = 0;
            }
            else if (fabsf(pos - stable_last_pos[i]) < FEEDBACK_STABLE_TOL)
            {
                /* 位置稳定，累积计数 */
                if (++stable_cnt[i] < FEEDBACK_STABLE_FRAMES) all_stable = 0;
            }
            else
            {
                /* 位置仍在变化，重新计数 */
                stable_cnt[i] = 0;
                all_stable = 0;
            }
            stable_last_pos[i] = pos;
        }

        /* 全部稳定 或 超时：锁定有效反馈关节的目标并进入正常运行 */
        if (all_stable || ((now - arm_state_tick) > FEEDBACK_WAIT_TIMEOUT_MS))
        {
            Arm_Controller_Try_Init_Target_All();
            arm_state = ARM_CTRL_STATE_RUN;
            usart_printf("[Arm] Startup OK -> RUN.\r\n");
        }
        break;
    }

    case ARM_CTRL_STATE_RUN:
    default:
        break;
    }
}

uint8_t Arm_Controller_Is_Running(void)
{
    return (arm_state == ARM_CTRL_STATE_RUN) ? 1 : 0;
}

/*  单关节安全下发 */
void Arm_Controller_Send_Joint(uint8_t idx, float joint_angle, float kp, float kd, float torque)
{
    if (idx >= ARM_JOINT_NUM) return;
    if (Arm_Joints[idx].Motor == NULL) return;

    /* 关节角 → 编码器角（加回零位）后走安全下发（DM_Motor_Ctrl_Safe 内部做状态自检/故障锁存）*/
    DM_Motor_Ctrl_Safe(&FDCAN2_TxFrame,
                       Arm_Joints[idx].Motor,
                       Arm_Joint_To_Encoder(idx, joint_angle),
                       0.0f,
                       kp,
                       kd,
                       torque);
}
