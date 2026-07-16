/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : B2B_Communication.h
  * @brief          : 赛级双板通信协议栈 (Board-to-Board)
  * @author         : GrassFan Wang
  * @date           : 2026/04/29
  * @version        : v2.1 (双遥控器架构兼容版)
  ******************************************************************************
  * @attention      : 所有结构体必须严格保证 8 字节，防止 CAN 总线错位！
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef B2B_COMMUNICATION_H
#define B2B_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* ========================================================================= */
/* ======================== 赛级 CAN ID 分配字典 =========================== */
/* ========================================================================= */

// ---------------- 【TX】云台板发往底盘板 ----------------
#define B2B_ID_GIMBAL_CMD        0x300  // 底盘运动控制指令

// 大疆遥控器透传 (2帧)
#define B2B_ID_RC_FRAME1         0x301
#define B2B_ID_RC_FRAME2         0x302

#define B2B_ID_VISION_DATA       0x303  // 【预留】视觉自瞄预测位姿透传
// 富斯 i6x 遥控器透传 (2帧) —— 完美兼容C板历史代码
#define B2B_ID_I6X_FRAME1        0x310
#define B2B_ID_I6X_FRAME2        0x311

// ---------------- 【RX】底盘板发往云台板 ----------------
// 为了避开 0x310 和 0x311，我们将接收 ID 往后顺延
#define B2B_ID_CHASSIS_STATE     0x315  // 底盘真实速度与底盘陀螺仪反馈
#define B2B_ID_REFEREE_HEAT      0x316  // 裁判系统：枪口热量与冷却反馈
#define B2B_ID_REFEREE_STATUS    0x317  // 裁判系统：血量、等级、增益反馈


  /* ========================================================================= */
  /* ======================== 数据载荷结构体 (严格8字节) ===================== */
  /* ========================================================================= */

  /* 1. 云台控制底盘指令 */
  typedef struct
  {
    int16_t vx;          // 目标 X 轴速度 (前后)
    int16_t vy;          // 目标 Y 轴速度 (左右)
    int16_t vw;          // 目标 W 轴速度 (自旋)
    uint8_t mode;        // 底盘模式 (0:无力 1:跟随 2:小陀螺 3:独立)
    uint8_t reserved;    // 补齐 8 字节
  } __attribute__((packed)) B2B_Gimbal_Cmd_t;

  /* 2. 遥控器透传帧1 (大疆 摇杆与拨杆) */
  typedef struct
  {
    int16_t ch0;         // 右摇杆左右
    int16_t ch1;         // 右摇杆上下
    int16_t ch2;         // 左摇杆左右
    uint8_t s0;          // 左拨杆
    uint8_t s1;          // 右拨杆
  } __attribute__((packed)) B2B_RC_Frame1_t;

  /* 3. 遥控器透传帧2 (大疆 键鼠与侧滑轮) */
  typedef struct
  {
    int16_t  ch3;        // 左摇杆上下
    uint16_t key_v;      // 键盘按键值
    int16_t  mouse_x;    // 鼠标 X 轴
    int16_t  mouse_y;    // 鼠标 Y 轴
  } __attribute__((packed)) B2B_RC_Frame2_t;

  /* 4. 【预留接收】底盘反馈的裁判系统核心状态 */
  typedef struct
  {
    uint16_t current_hp;     // 当前血量 (2字节)
    uint16_t shooter_heat;   // 枪口热量 (2字节)
    uint8_t  robot_level;    // 机器人等级 (1字节)

    // 【架构师技巧】：使用位域将 8 个 bool 状态无损压缩进 1 个字节
    uint8_t  is_shield_on:1;     // 基地护盾是否激活
    uint8_t  is_outpost_alive:1; // 前哨站是否存活
    uint8_t  color:1;            // 0红 1蓝
    uint8_t  reserved_bits:5;    // 剩余位

    uint8_t  reserved[2];    // 补齐 8 字节
  } __attribute__((packed)) B2B_Referee_Status_t;

extern B2B_Gimbal_Cmd_t     BoardCom_GimbalCmd;
extern B2B_Referee_Status_t BoardCom_RefereeStatus;

void B2B_Send_HighFreq_Data(void);
void B2B_Send_DjRemote_Data(void);
void B2B_Send_Ix6Remote_Data(void); // 新增：富斯 i6x 发送接口

void B2B_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf);

#ifdef __cplusplus
}
#endif

#endif // B2B_COMMUNICATION_H