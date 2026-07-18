/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Kinematics.c
  * @brief          : 六轴机械臂正/逆运动学接口实现
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      :
  *   1. DH 参数表（Arm_Kinematics_DH_Default）中的数值为占位数据，
  *      你必须根据实际机械臂的尺寸和构型修改！
  *   2. IK 函数为桩实现，请替换为真实的 IK 算法。
  *   3. 所有角度单位：弧度；位置单位：米。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Arm_Kinematics.h"
#include "string.h"
#include "math.h"
#include "usart_printf_task.h"
/* External variables --------------------------------------------------------*/
/** @brief FDCAN2 发送帧（由 bsp_can 或 fdcan.c 定义，用于下发电机控制指令）*/
extern FDCAN_TxFrame_TypeDef FDCAN2_TxFrame;

/* Private variables ---------------------------------------------------------*/

/**
  * @brief 全局运动学实例
  */
Arm_Kinematics_Info_Typedef Arm_Kinematics;

/* ========================================================================== */
/* =========== 默认 DH 参数表（用户必须根据实际机械臂修改！）=============== */
/* ========================================================================== */

/**
  * @brief 默认 DH 参数（标准 Craig 约定）
  *
  *  关节 |  a (m)  | alpha (rad) |  d (m)  | theta_offset (rad) |  限位
  *  ------|---------|-------------|---------|---------------------|----------
  *  J1   |  待填入  |   待填入    |  待填入  |      待填入         | 待填入
  *  J2   |  待填入  |   待填入    |  待填入  |      待填入         | 待填入
  *  ...
  *
  * @note  【重要】以下是占位数据，请替换为实际数值！
  *        你可以在此函数内硬编码，或在 Arm_Kinematics_Init 时从外部传入。
  */
static const DH_Param_Typedef Arm_Kinematics_DH_Default[KIN_JOINT_NUM] =
{
    /* ========== 关节 0 (J1 — 腰部旋转) ========== */
    [0] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -3.14f,     /* 约 -180° */
        .theta_max     =  3.14f,     /* 约 +180° */
    },
    /* ========== 关节 1 (J2 — 大臂俯仰) ========== */
    [1] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -2.62f,     /* 约 -150° */
        .theta_max     =  2.62f,     /* 约 +150° */
    },
    /* ========== 关节 2 (J3 — 小臂俯仰) ========== */
    [2] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -2.62f,     /* 约 -150° */
        .theta_max     =  2.62f,     /* 约 +150° */
    },
    /* ========== 关节 3 (J4 — 腕部旋转) ========== */
    [3] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -3.14f,     /* 约 -180° */
        .theta_max     =  3.14f,     /* 约 +180° */
    },
    /* ========== 关节 4 (J5 — 腕部俯仰) ========== */
    [4] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -2.09f,     /* 约 -120° */
        .theta_max     =  2.09f,     /* 约 +120° */
    },
    /* ========== 关节 5 (J6 — 腕部旋转/法兰) ========== */
    [5] = {
        .a             = 0.0f,       /* TODO: 填入实际值 */
        .alpha         = 0.0f,       /* TODO: 填入实际值 */
        .d             = 0.0f,       /* TODO: 填入实际值 */
        .theta_offset  = 0.0f,       /* TODO: 填入实际值 */
        .theta_min     = -3.14f,     /* 约 -180° */
        .theta_max     =  3.14f,     /* 约 +180° */
    },
};

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  根据 DH 参数计算单个连杆变换矩阵 (4x4)
  * @param  dh     : 当前关节的 DH 参数
  * @param  theta  : 当前关节角度 (弧度)
  * @param  T_out  : 输出 4x4 变换矩阵 (行优先)
  */
static void DH_To_Transform(const DH_Param_Typedef *dh, float theta, float T_out[16]);

/**
  * @brief  4x4 矩阵乘法 C = A * B (行优先)
  */
static void Mat4_Multiply(const float A[16], const float B[16], float C[16]);

/**
  * @brief  从 4x4 齐次变换矩阵中提取位姿 (位置 + 旋转矩阵 + 欧拉角)
  */
static void Pose_From_Transform(const float T[16], Arm_Pose_Typedef *pose);

/**
  * @brief  从旋转矩阵提取 ZYX 欧拉角 (yaw-pitch-roll)
  */
static void Euler_From_Rotation(const float R[9], float *yaw, float *pitch, float *roll);

/* ========================================================================== */
/* ======================== 工具函数 (私有) ================================= */
/* ========================================================================== */

static void DH_To_Transform(const DH_Param_Typedef *dh, float theta, float T_out[16])
{
    float ct = cosf(theta + dh->theta_offset);
    float st = sinf(theta + dh->theta_offset);
    float ca = cosf(dh->alpha);
    float sa = sinf(dh->alpha);
    float a  = dh->a;
    float d  = dh->d;

    /* 标准 Craig DH：T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
     *
     *  [ ct      -st      0       a     ]
     *  | st*ca   ct*ca   -sa    -d*sa   |
     *  | st*sa   ct*sa    ca     d*ca   |
     *  [ 0        0       0       1     ]
     */
    T_out[0]  = ct;    T_out[1]  = -st;   T_out[2]  = 0.0f;  T_out[3]  = a;
    T_out[4]  = st*ca; T_out[5]  = ct*ca; T_out[6]  = -sa;   T_out[7]  = -d*sa;
    T_out[8]  = st*sa; T_out[9]  = ct*sa; T_out[10] = ca;    T_out[11] = d*ca;
    T_out[12] = 0.0f;  T_out[13] = 0.0f;  T_out[14] = 0.0f; T_out[15] = 1.0f;
}

/* -------------------------------------------------------------------------- */

static void Mat4_Multiply(const float A[16], const float B[16], float C[16])
{
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
            {
                sum += A[row * 4 + k] * B[k * 4 + col];
            }
            C[row * 4 + col] = sum;
        }
    }
}

/* -------------------------------------------------------------------------- */

static void Euler_From_Rotation(const float R[9], float *yaw, float *pitch, float *roll)
{
    /* ZYX 欧拉角: R = Rz(yaw) * Ry(pitch) * Rx(roll) */
    if (pitch == NULL || yaw == NULL || roll == NULL) return;

    /* 防止 gimbal lock */
    float sy = sqrtf(R[0]*R[0] + R[3]*R[3]);

    if (sy > 1e-6f)
    {
        *yaw   = atan2f(R[5],  R[8]);
        *pitch = atan2f(-R[2], sy);
        *roll  = atan2f(R[1],  R[0]);
    }
    else
    {
        *yaw   = 0.0f;
        *pitch = atan2f(-R[2], sy);
        *roll  = atan2f(-R[3], R[4]);
    }
}

/* -------------------------------------------------------------------------- */

static void Pose_From_Transform(const float T[16], Arm_Pose_Typedef *pose)
{
    if (pose == NULL) return;

    /* 位置 = T[0..2][3] */
    pose->x = T[3];
    pose->y = T[7];
    pose->z = T[11];

    /* 旋转矩阵 (3x3, 行优先) */
    pose->R[0] = T[0]; pose->R[1] = T[1]; pose->R[2] = T[2];
    pose->R[3] = T[4]; pose->R[4] = T[5]; pose->R[5] = T[6];
    pose->R[6] = T[8]; pose->R[7] = T[9]; pose->R[8] = T[10];

    /* 欧拉角 */
    Euler_From_Rotation(pose->R, &pose->yaw, &pose->pitch, &pose->roll);
}

/* ========================================================================== */
/* ======================== 公开接口实现 ==================================== */
/* ========================================================================== */

void Arm_Kinematics_Init(Arm_Kinematics_Info_Typedef *kin,
                          const DH_Param_Typedef *dh_table)
{
    if (kin == NULL)
        kin = &Arm_Kinematics;

    /* 清空结构体 */
    memset(kin, 0, sizeof(Arm_Kinematics_Info_Typedef));

    /* 加载 DH 参数表 */
    if (dh_table != NULL)
    {
        /* 使用用户传入的 DH 表 */
        for (int i = 0; i < KIN_JOINT_NUM; i++)
        {
            kin->DH[i] = dh_table[i];
        }
    }
    else
    {
        /* 使用默认 DH 表（占位值，用户须修改！） */
        for (int i = 0; i < KIN_JOINT_NUM; i++)
        {
            kin->DH[i] = Arm_Kinematics_DH_Default[i];
        }
    }

    kin->Init = true;

    usart_printf("[Kinematics] Arm_Kinematics_Init OK (%d joints).\r\n"
                 "  [WARNING] DH parameters are placeholders! "
                 "Edit Arm_Kinematics_DH_Default in Arm_Kinematics.c\r\n",
                 KIN_JOINT_NUM);
}

/* -------------------------------------------------------------------------- */

bool Arm_Forward_Kinematics(Arm_Kinematics_Info_Typedef *kin,
                             const float joint_rad[KIN_JOINT_NUM],
                             Arm_Pose_Typedef *pose_out)
{
    if (kin == NULL || joint_rad == NULL)
        return false;

    /* 初始化总变换为单位矩阵 */
    float T_total[16] = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };

    float T_temp[16];

    /* 逐关节连乘：T_0_n = T_0_1 * T_1_2 * ... * T_{n-1}_n */
    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        /* 计算当前连杆的 DH 变换矩阵 */
        DH_To_Transform(&kin->DH[i], joint_rad[i], kin->T_matrices[i]);

        /* 累积到总变换 */
        memcpy(T_temp, T_total, sizeof(T_temp));
        Mat4_Multiply(T_temp, kin->T_matrices[i], T_total);
    }

    /* 保存总变换 */
    memcpy(kin->T_0_n, T_total, sizeof(kin->T_0_n));

    /* 保存当前关节角度快照 */
    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        kin->Joint_Current_Rad[i] = joint_rad[i];
    }

    /* 提取位姿 */
    if (pose_out != NULL)
    {
        Pose_From_Transform(T_total, pose_out);
    }
    else
    {
        Pose_From_Transform(T_total, &kin->Current_Pose);
    }

    return true;
}

/* -------------------------------------------------------------------------- */

bool Arm_Inverse_Kinematics(Arm_Kinematics_Info_Typedef *kin,
                             const Arm_Pose_Typedef *target_pose,
                             float joint_out[KIN_JOINT_NUM])
{
    if (kin == NULL || target_pose == NULL || joint_out == NULL)
        return false;

    /* ================================================================ */
    /* 【TODO】逆运动学核心算法 — 请根据你的机械臂构型实现              */
    /*                                                                    */
    /*  常见策略：                                                       */
    /*  1. 解析解（Pieper 准则）：如 6 轴球形腕 → 闭式公式              */
    /*  2. 数值解（迭代法）：                                           */
    /*     - Jacobian 伪逆: dq = J^+ * dx                               */
    /*     - 阻尼最小二乘 (DLS / Levenberg-Marquardt)                   */
    /*     - 每次迭代后检查关节限位 + 奇异检测                          */
    /*                                                                    */
    /*  参考接口：                                                       */
    /*    kin->Joint_Current_Rad[] — 当前角度（可作为迭代初值）         */
    /*    kin->DH[] — DH 参数                                          */
    /*    kin->Current_Pose — 当前末端位姿                              */
    /* ================================================================ */

    /* 占位实现：直接返回失败 */
    (void)target_pose;

    usart_printf("[Kinematics] WARNING: Arm_Inverse_Kinematics is a stub! "
                 "Implement your IK algorithm.\r\n");

    return false;
}

/* -------------------------------------------------------------------------- */

void Arm_Kinematics_FromFeedback(Arm_Kinematics_Info_Typedef *kin)
{
    if (kin == NULL)
        kin = &Arm_Kinematics;

    if (!kin->Init)
    {
        Arm_Kinematics_Init(kin, NULL);
    }

    /* 从 Arm_Joints[] 读取当前关节角度
     * Arm_Joints[i].Current_Angle 是 "度"，需要转弧度
     */
    float joint_rad[KIN_JOINT_NUM];
    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        joint_rad[i] = Arm_Joints[i].Current_Angle * DegreesToRadians;
    }

    /* 执行正向运动学 */
    Arm_Forward_Kinematics(kin, joint_rad, NULL);
}

/* -------------------------------------------------------------------------- */

bool Arm_Kinematics_ApplyIK(Arm_Kinematics_Info_Typedef *kin,
                             const Arm_Pose_Typedef *target_pose)
{
    if (kin == NULL)
        kin = &Arm_Kinematics;

    if (target_pose == NULL)
        return false;

    if (!kin->Init)
    {
        Arm_Kinematics_Init(kin, NULL);
    }

    float joint_target[KIN_JOINT_NUM];

    /* 调用 IK */
    bool success = Arm_Inverse_Kinematics(kin, target_pose, joint_target);

    if (!success)
    {
        usart_printf("[Kinematics] IK failed — no valid solution for target pose.\r\n");
        return false;
    }

    /* IK 成功：检查关节限位 */
    if (!Arm_Kinematics_CheckLimits(kin, joint_target))
    {
        usart_printf("[Kinematics] IK solution exceeds joint limits!\r\n");
        return false;
    }

    /* 写入 Arm_Joints[].Target_Angle
     * Target_Angle 与电机通信时使用弧度（根据现有代码逻辑），
     * 而 Current_Angle 是度。我们这里统一按弧度写入。
     * 注意：如果你将 Target_Angle 定义为度，请乘以 Rad_to_angle 转换。
     */
    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        Arm_Joints[i].Target_Angle = joint_target[i];
        kin->Joint_Target_Rad[i]   = joint_target[i];
    }

    return true;
}

/* -------------------------------------------------------------------------- */

void Arm_Kinematics_SendAll(Arm_Kinematics_Info_Typedef *kin,
                             float kp, float kd,
                             const float ff_torque[KIN_JOINT_NUM])
{
    if (kin == NULL)
        kin = &Arm_Kinematics;

    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        float torque = (ff_torque != NULL) ? ff_torque[i] : 0.0f;

        DM_Motor_CAN_TxMessage(
            &FDCAN2_TxFrame,
            Arm_Joints[i].Motor,
            Arm_Joints[i].Target_Angle,   /* 目标位置 (弧度) */
            0.0f,                         /* 目标速度 */
            kp,                           /* 刚度 */
            kd,                           /* 阻尼 */
            torque                        /* 前馈力矩 */
        );
    }
}

/* -------------------------------------------------------------------------- */

bool Arm_Kinematics_CheckLimits(Arm_Kinematics_Info_Typedef *kin,
                                 const float joint_rad[KIN_JOINT_NUM])
{
    if (kin == NULL || joint_rad == NULL)
        return false;

    for (int i = 0; i < KIN_JOINT_NUM; i++)
    {
        if (joint_rad[i] < kin->DH[i].theta_min ||
            joint_rad[i] > kin->DH[i].theta_max)
        {
            return false;
        }
    }
    return true;
}