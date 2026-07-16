#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

/* ================== 赛级底层参数 ================== */
// 【防卡死护甲】：为了 H7 的 Cache 32字节对齐，必须设为 32！(有效数据依然是 18)
#define SBUS_RX_BUF_NUM         32u

#define RC_CH_VALUE_OFFSET      1024.0f
#define RC_CH_VALUE_MAX         660.0f  // 摇杆最大偏移量 (1684 - 1024)
#define RC_CH_VALUE_MIN         -660.0f 

// 【防疯车护甲】：DBUS 协议数据合法区间
#define RC_CH_MIN_RAW           300     // 预留少量死区余量
#define RC_CH_MAX_RAW           1700
#define RC_SW_UP                1
#define RC_SW_DOWN              2
#define RC_SW_MID               3

/* ================== 赛级键盘状态机 ================== */
// 抛弃复杂的 enum，使用更符合操作直觉的边沿检测记录结构体
typedef struct
{
    bool IS_PRESSED;        // 当前是否处于按下状态
    bool RISING_EDGE;       // 按下的瞬间 (0->1，常用于切换模式：如开/关摩擦轮)
    bool FALLING_EDGE;      // 松开的瞬间 (1->0)
    uint16_t HOLD_COUNT;    // 持续按下的计数器 (常用于长按：如长按自爆、长按无敌小陀螺)
} KeyBoard_Info_Typedef;

typedef struct
{
    KeyBoard_Info_Typedef press_l;
    KeyBoard_Info_Typedef press_r;
    KeyBoard_Info_Typedef W, S, A, D, SHIFT, CTRL, Q, E, R, F, G, Z, X, C, V, B;
} Remote_Pressed_Typedef;

/* ================== 遥控器总信息 ================== */
typedef struct
{
    struct {
       int16_t ch[5];
       uint8_t s[2];
    } rc;
    
    struct {
       int16_t x, y, z;
       uint8_t press_l, press_r;
    } mouse;

    union {
       uint16_t v;
       struct {
          uint16_t W:1; uint16_t S:1; uint16_t A:1; uint16_t D:1;
          uint16_t SHIFT:1; uint16_t CTRL:1; uint16_t Q:1; uint16_t E:1;
          uint16_t R:1; uint16_t F:1; uint16_t G:1; uint16_t Z:1;
          uint16_t X:1; uint16_t C:1; uint16_t V:1; uint16_t B:1;
       } set;
    } key;

    Remote_Pressed_Typedef key_state; // 包含边沿触发状态的高级键盘库

    bool rc_lost;         // 失联标志位 (赛场断电保护)
    uint8_t online_cnt;   // 在线倒计时
} Remote_Info_Typedef;

extern Remote_Info_Typedef remote_ctrl;
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);

#endif //REMOTE_CONTROL_H