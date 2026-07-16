/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct{
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
}FDCAN_RxFrame_TypeDef;

/* ========================================================= */
/* 双板通讯：遥控器数据透传专用结构体 (每帧严格限制 8 字节) */
/* ========================================================= */

// 第一帧：负责传输 3 个摇杆和 2 个拨杆 (刚好 8 字节)
typedef __packed struct
{
    int16_t ch0;        // 右摇杆左右 (2字节)
    int16_t ch1;        // 右摇杆上下 (2字节)
    int16_t ch2;        // 左摇杆左右 (2字节)
    uint8_t s0;         // 左拨杆    (1字节)
    uint8_t s1;         // 右拨杆    (1字节)
} BoardCom_RC_Frame1_t;

// 第二帧：负责传输剩下的 1 个摇杆、键盘和鼠标 (刚好 8 字节)
typedef __packed struct
{
    int16_t  ch3;       // 左摇杆上下 (2字节)
    uint16_t key_v;     // 键盘按键值 (2字节)
    int16_t  mouse_x;   // 鼠标 X 轴  (2字节)
    int16_t  mouse_y;   // 鼠标 Y 轴  (2字节)
} BoardCom_RC_Frame2_t;


/* Externs ------------------------------------------------------------------*/
extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN2_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN3_TxFrame;
extern void  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
extern void BSP_FDCAN_Init(void);

	   
#endif