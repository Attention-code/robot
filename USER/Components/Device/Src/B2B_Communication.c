/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : B2B_Communication.c
  * @brief          : 赛级双板通信实现层
  * @author         : GrassFan Wang
  * @date           : 2026/04/29
  ******************************************************************************
  */
/* USER CODE END Header */

#include "B2B_Communication.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Ix6_Remote.h"
#include "string.h"
#include "main.h"



// ======================== 全局数据实例 ========================
B2B_Gimbal_Cmd_t     BoardCom_GimbalCmd;
B2B_Referee_Status_t BoardCom_RefereeStatus;

/* ========================================================================= */
/* ======================== 发送层 (TX Routing) ============================ */
/* ========================================================================= */

void B2B_Send_HighFreq_Data(void)
{
    FDCAN3_TxFrame.Header.Identifier = B2B_ID_GIMBAL_CMD;
    memcpy(FDCAN3_TxFrame.Data, &BoardCom_GimbalCmd, sizeof(B2B_Gimbal_Cmd_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

/**
  * @brief  发送大疆遥控器数据 (保留第二帧的 ch3，丢弃键鼠数据)
  */
void B2B_Send_DjRemote_Data(void)
{
    // 【关键细节】：加 = {0} 初始化，防止 CAN 发送出随机的垃圾内存
    B2B_RC_Frame1_t frame1 = {0};
    B2B_RC_Frame2_t frame2 = {0};

    // --- 1. 打包并发送第一帧 (3个摇杆 + 2个拨杆) ---
    frame1.ch0 = remote_ctrl.rc.ch[0];
    frame1.ch1 = remote_ctrl.rc.ch[1];
    frame1.ch2 = remote_ctrl.rc.ch[2];
    frame1.s0  = remote_ctrl.rc.s[0];
    frame1.s1  = remote_ctrl.rc.s[1];

    FDCAN3_TxFrame.Header.Identifier = B2B_ID_RC_FRAME1;
    memcpy(FDCAN3_TxFrame.Data, &frame1, sizeof(B2B_RC_Frame1_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

    // --- 2. 打包并发送第二帧 (仅保留 ch3) ---
    frame2.ch3 = remote_ctrl.rc.ch[3];
    // key_v, mouse_x, mouse_y 都不管了，它们现在全是干净的 0

    FDCAN3_TxFrame.Header.Identifier = B2B_ID_RC_FRAME2;
    memcpy(FDCAN3_TxFrame.Data, &frame2, sizeof(B2B_RC_Frame2_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

/**
  * @brief  发送富斯 i6x 遥控器数据 (采用大端模式手动封包)
  * @note   此格式完美迎合 C 板底盘的 0x310 和 0x311 接收代码
  */
void B2B_Send_Ix6Remote_Data(void)
{
    i6x_ctrl_t *i6x = get_i6x_point();

    // --- 1. 发送第一帧 (ID: 0x310) 包含 ch0 ~ ch3 ---
    FDCAN3_TxFrame.Header.Identifier = B2B_ID_I6X_FRAME1;

    // 因为 C 板是使用 (rx_data[0] << 8) | rx_data[1] 解包的
    // 我们必须手动进行大端移位组装！
    FDCAN3_TxFrame.Data[0] = (uint8_t)(i6x->ch[0] >> 8);
    FDCAN3_TxFrame.Data[1] = (uint8_t)(i6x->ch[0] & 0xFF);
    FDCAN3_TxFrame.Data[2] = (uint8_t)(i6x->ch[1] >> 8);
    FDCAN3_TxFrame.Data[3] = (uint8_t)(i6x->ch[1] & 0xFF);
    FDCAN3_TxFrame.Data[4] = (uint8_t)(i6x->ch[2] >> 8);
    FDCAN3_TxFrame.Data[5] = (uint8_t)(i6x->ch[2] & 0xFF);
    FDCAN3_TxFrame.Data[6] = (uint8_t)(i6x->ch[3] >> 8);
    FDCAN3_TxFrame.Data[7] = (uint8_t)(i6x->ch[3] & 0xFF);

    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

    // --- 2. 发送第二帧 (ID: 0x311) 包含 ch4, ch5, s0~s3 ---
    FDCAN3_TxFrame.Header.Identifier = B2B_ID_I6X_FRAME2;

    FDCAN3_TxFrame.Data[0] = (uint8_t)(i6x->ch[4] >> 8);
    FDCAN3_TxFrame.Data[1] = (uint8_t)(i6x->ch[4] & 0xFF);
    FDCAN3_TxFrame.Data[2] = (uint8_t)(i6x->ch[5] >> 8);
    FDCAN3_TxFrame.Data[3] = (uint8_t)(i6x->ch[5] & 0xFF);

    // 拨杆数据是 1 字节有符号整数，直接装入即可
    FDCAN3_TxFrame.Data[4] = (uint8_t)i6x->s[0];
    FDCAN3_TxFrame.Data[5] = (uint8_t)i6x->s[1];
    FDCAN3_TxFrame.Data[6] = (uint8_t)i6x->s[2];
    FDCAN3_TxFrame.Data[7] = (uint8_t)i6x->s[3];

    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}




/* ========================================================================= */
/* ======================== 接收层 (RX Routing) ============================ */
/* ========================================================================= */

void B2B_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf)
{
    switch (*Identifier)
    {
        case B2B_ID_REFEREE_STATUS:
            memcpy(&BoardCom_RefereeStatus, Rx_Buf, sizeof(B2B_Referee_Status_t));
            break;

        default:
            break;
    }
}