/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Arm_Kinematics.h
  * @brief          : 六轴机械臂正/逆运动学接口，与 Arm_Task 控制联动
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 使用标准 DH 参数约定（Craig 版本）
  *                   所有角度单位为弧度，位置单位为米
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "arm_math.h"
#include "Motor.h"          /* DM_Motor_CAN_TxMessage 等 */
#include "Arm_Task.h"       /* Arm_Joints, ARM_JOINT_NUM */

/* Exported defines ----------------------------------------------------------*/

/**
 * @brief 4x4 齐次变换矩阵数据长度
 */
#define HOMO_MAT_SIZE   16

/**
 * @brief 旋转矩阵数据长度 (3x3)
 */
#define ROT_MAT_SIZE    9

/**
 * @brief 机械臂关节数量（与 Arm_Task 保持一致）
 */
#define KIN_JOINT_NUM   ARM_JOINT_NUM

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 标准 DH 参数（Craig 约定）
 *        相邻连杆 i-1 → i 的变换：
 *        T = Rot_x(alpha_i) * Trans_x(a_i) * Rot_z(theta_i) * Trans_z(d_i)
 */
typedef struct
{
    float a;              /*!< 连杆长度 (米)  — 沿 x_{i-1} 轴，从 z_{i-1} 到 z_i 的距离 */
    float alpha;          /*!< 连杆扭转 (弧度) — 绕 x_{i-1} 轴，从 z_{i-1} 到 z_i 的转角 */
    float d;              /*!< 连杆偏距 (米)  — 沿 z_i 轴，从 x_{i-1} 到 x_i 的距离 */
    float theta_offset;   /*!< 关节角度偏移 (弧度) — theta_i = 电机反馈 + theta_offset */
    float theta_min;      /*!< 关节角度下限 (弧度) — 运动学限位 */
    float theta_max;      /*!< 关节角度上限 (弧度) — 运动学限位 */
} DH_Param_Typedef;

/**
 * @brief 末端执行器位姿
 */
typedef struct
{
    float x;              /*!< 位置 x (米) */
    float y;              /*!< 位置 y (米) */
    float z;              /*!< 位置 z (米) */

    /* 旋转矩阵 R (3x3, 按行优先存储)
     * R[0] R[1] R[2]
     * R[3] R[4] R[5]
     * R[6] R[7] R[8]
     */
    float R[ROT_MAT_SIZE];

    /* 也可用欧拉角表示 (弧度)，ZYX 顺序 (yaw-pitch-roll) */
    float yaw;            /*!< 偏航角 (弧度) — 绕 Z 轴 */
    float pitch;          /*!< 俯仰角 (弧度) — 绕 Y 轴 */
    float roll;           /*!< 横滚角 (弧度) — 绕 X 轴 */
} Arm_Pose_Typedef;

/**
 * @brief 运动学总体信息结构体
 */
typedef struct
{
    /* DH 参数表 — 大小为 KIN_JOINT_NUM */
    DH_Param_Typedef DH[KIN_JOINT_NUM];

    /* 当前关节角度 (弧度) — 来自电机反馈，供 FK 使用 */
    float Joint_Current_Rad[KIN_JOINT_NUM];

    /* 目标关节角度 (弧度) — IK 结果，待下发到电机 */
    float Joint_Target_Rad[KIN_JOINT_NUM];

    /* 当前末端位姿 — FK 计算结果 */
    Arm_Pose_Typedef Current_Pose;

    /* 正向运动学中间变量：各连杆的齐次变换矩阵 (4x4, 行优先) */
    float T_matrices[KIN_JOINT_NUM][HOMO_MAT_SIZE];

    /* 总变换矩阵 T_0_n (基座→末端, 4x4, 行优先) */
    float T_0_n[HOMO_MAT_SIZE];

    /* 初始化标志 */
    bool Init;
} Arm_Kinematics_Info_Typedef;

/* Exported variables --------------------------------------------------------*/

/** @brief 全局运动学实例 (由 Arm_Kinematics_Init 初始化) */
extern Arm_Kinematics_Info_Typedef Arm_Kinematics;

/* Exported function prototypes ----------------------------------------------*/

/**
  * @brief  初始化运动学模块：加载 DH 参数表，清空状态
  * @param  kin  : 运动学实例指针（传 NULL 则初始化内部全局实例）
  * @param  dh_table : DH 参数表（传 NULL 则使用默认 DH 表）
  * @note   默认 DH 表为通用 6R 串联机械臂占位值，
  *         用户必须在 `Arm_Kinematics_DH_Default` 中填入实际参数！
  */
extern void Arm_Kinematics_Init(Arm_Kinematics_Info_Typedef *kin,
                                 const DH_Param_Typedef *dh_table);

/**
  * @brief  正向运动学 (FK)
  *         根据关节角度计算末端执行器位姿
  * @param  kin       : 运动学实例指针
  * @param  joint_rad : 6 个关节角度 (弧度)
  * @param  pose_out  : 输出末端位姿（传 NULL 则存入 kin->Current_Pose）
  * @retval true  — 计算成功
  *         false — 参数无效
  */
extern bool Arm_Forward_Kinematics(Arm_Kinematics_Info_Typedef *kin,
                                    const float joint_rad[KIN_JOINT_NUM],
                                    Arm_Pose_Typedef *pose_out);

/**
  * @brief  逆运动学 (IK)
  *         根据目标末端位姿计算 6 个关节角度
  * @param  kin         : 运动学实例指针
  * @param  target_pose : 目标末端位姿
  * @param  joint_out   : 输出关节角度 (弧度, 数组大小为 KIN_JOINT_NUM)
  * @retval true  — 求解成功
  *         false — 无解 / 超出关节限位 / 奇异位形
  * @note   【核心占位】当前为占位桩函数，返回 false。
  *         请根据你的具体机械臂构型实现 IK 算法。
  *         推荐方法：
  *           - 6 轴球形腕 => 解析解 (Pieper 准则)
  *           - 通用构型 => 数值迭代 (Jacobian 伪逆 / 阻尼最小二乘)
  */
extern bool Arm_Inverse_Kinematics(Arm_Kinematics_Info_Typedef *kin,
                                    const Arm_Pose_Typedef *target_pose,
                                    float joint_out[KIN_JOINT_NUM]);

/* ========== 与 Arm_Task / 电机控制联动的桥接接口 ========== */

/**
  * @brief  从 Arm_Joints[] 读取当前关节反馈，执行 FK
  *         将结果存入 kin->Current_Pose
  * @param  kin : 运动学实例指针
  * @note   在每次控制循环中先调用此函数，
  *         即可获知当前末端位姿（用于闭环 / 可视化 / 阻抗控制等）
  */
extern void Arm_Kinematics_FromFeedback(Arm_Kinematics_Info_Typedef *kin);

/**
  * @brief  计算 IK 并将结果写入 Arm_Joints[].Target_Angle
  *         若 IK 成功，关节目标角度被更新；若失败则不修改
  * @param  kin         : 运动学实例指针
  * @param  target_pose : 期望的末端位姿
  * @retval true  — IK 求解成功，目标已写入
  *         false — IK 无解，Arm_Joints 目标未修改
  */
extern bool Arm_Kinematics_ApplyIK(Arm_Kinematics_Info_Typedef *kin,
                                    const Arm_Pose_Typedef *target_pose);

/**
  * @brief  将 Arm_Joints[].Target_Angle 逐关节下发到电机
  * @param  kin     : 运动学实例指针
  * @param  kp      : MIT 模式 KP (刚度)
  * @param  kd      : MIT 模式 KD (阻尼)
  * @param  ff_torque : 前馈力矩数组 (可传 NULL 表示零)
  * @note   使用 FDCAN2_TxFrame 发送。
  *         配合 Arm_Kinematics_ApplyIK 使用：
  *           ApplyIK → SendAll
  */
extern void Arm_Kinematics_SendAll(Arm_Kinematics_Info_Typedef *kin,
                                    float kp, float kd,
                                    const float ff_torque[KIN_JOINT_NUM]);

/**
  * @brief  检查关节角度是否在 DH 限位范围内
  * @param  kin        : 运动学实例指针
  * @param  joint_rad  : 待检查的关节角度 (弧度)
  * @retval true  — 所有关节均在限位内
  *         false — 至少一个关节超限
  */
extern bool Arm_Kinematics_CheckLimits(Arm_Kinematics_Info_Typedef *kin,
                                        const float joint_rad[KIN_JOINT_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */