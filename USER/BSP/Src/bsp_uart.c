/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions (HAL Callback 赛级优化版)
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v2.1 (双遥控器架构)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "Remote_Control.h" // 旧大疆遥控头文件
#include "Referee_System.h"
#include "Image_Transmission.h"
#include "Ix6_Remote.h"
#include "main.h"

// 内部函数声明
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

PLL2_ClocksTypeDef PLL2_ClockFreq;

// ======================== 设备切换宏定义 ========================
#define USART1_RX_Switch   1  // 0: Referee_System | 1: Image_Transmission
#define REMOTE_TYPE_Switch 1  // 0: 大疆 DBUS 遥控 | 1: 富斯 i6X SBUS 遥控
// ================================================================

/**
  * @brief  初始化所有串口与双缓冲 DMA
  */
void BSP_USART_Init(void)
{
    HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq);
    uint32_t USART1_ClockFreq = PLL2_ClockFreq.PLL2_Q_Frequency;

    // ---------------- UART1 初始化 ----------------
    #if USART1_RX_Switch
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR = (uint32_t)(USART1_ClockFreq/921600); // 图传高波特率
        USART1->CR1 |= USART_CR1_UE;
        USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)Image_Trans_MultiRx_Buff[0], (uint32_t *)Image_Trans_MultiRx_Buff[1], IMAGE_TRANS_RX_LENGTH);
    #else
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR = (uint32_t)(USART1_ClockFreq/115200); // 裁判系统标准波特率
        USART1->CR1 |= USART_CR1_UE;
        USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)Referee_System_Info_MultiRx_Buf[0], (uint32_t *)Referee_System_Info_MultiRx_Buf[1], REFEREE_RXFRAME_LENGTH);
    #endif

    // ---------------- UART5 初始化 (遥控器) ----------------
    // 无论是大疆还是i6x，底层数组都是32字节，无需修改配置
    USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief  双缓冲 DMA 极速底层配置
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxXferSize    = DataLength * 2;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR  = (uint32_t)&huart->Instance->RDR;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = DataLength;

    SET_BIT(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/* ================================================================================= */
/* ======================== 赛级架构：统一事件与错误回调 =========================== */
/* ================================================================================= */

/**
  * @brief  全局串口空闲/接收完成回调中心 (无 Cache 干扰极速版)
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 获取当前串口的 DMA 数据流指针
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    // ---------------- 1. UART5 遥控器数据处理 ----------------
    if(huart == &huart5)
    {
        uint16_t rx_len = SBUS_RX_BUF_NUM - dma_stream->NDTR; // 算出真实接收长度
        uint8_t finished_buf;

        /* 1. 强行关闭 DMA，准备手动翻转 */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* 2. 判定刚才写的是哪个缓冲，强行切到另一个 */
        if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
            finished_buf = 0; dma_stream->CR |= DMA_SxCR_CT;
        } else {
            finished_buf = 1; dma_stream->CR &= ~DMA_SxCR_CT;
        }

        /* 3. 重新装填弹药 (重置长度) 并开启 DMA */
        dma_stream->NDTR = SBUS_RX_BUF_NUM;
        __HAL_DMA_ENABLE(huart->hdmarx);

        /* 4. 【已删除】SCB_InvalidateDCache 操作。因为数组在 D1 无 Cache 区，直接读！ */

        /* 5. 长度严格校验后解算 (利用宏切换不同协议) */
        #if (REMOTE_TYPE_Switch == 0)
            if(rx_len == 18) // 大疆 DBUS 协议固定 18 字节
            {
                SBUS_TO_RC(SBUS_MultiRx_Buf[finished_buf], &remote_ctrl);
            }
        #elif (REMOTE_TYPE_Switch == 1)
            if(rx_len == 25) // 富斯 i6X 标准 SBUS 协议固定 25 字节
            {
                sbus_to_i6x(&i6x_ctrl, SBUS_MultiRx_Buf[finished_buf]);
            }
        #endif
    }

    // ---------------- 2. UART1 图传/裁判系统数据处理 ----------------
    else if(huart == &huart1)
    {
        #if USART1_RX_Switch
            uint16_t rx_len = IMAGE_TRANS_RX_LENGTH - dma_stream->NDTR;
            uint8_t finished_buf;

            __HAL_DMA_DISABLE(huart->hdmarx);
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 0; dma_stream->CR |= DMA_SxCR_CT;
            } else {
                finished_buf = 1; dma_stream->CR &= ~DMA_SxCR_CT;
            }
            dma_stream->NDTR = IMAGE_TRANS_RX_LENGTH;
            __HAL_DMA_ENABLE(huart->hdmarx);

            /* 【已删除】SCB_InvalidateDCache 操作 */

            if(rx_len > 10) {
                Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[finished_buf]);
            }
        #else
            uint16_t rx_len = REFEREE_RXFRAME_LENGTH - dma_stream->NDTR;
            uint8_t finished_buf;

            __HAL_DMA_DISABLE(huart->hdmarx);
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 0; dma_stream->CR |= DMA_SxCR_CT;
            } else {
                finished_buf = 1; dma_stream->CR &= ~DMA_SxCR_CT;
            }
            dma_stream->NDTR = REFEREE_RXFRAME_LENGTH;
            __HAL_DMA_ENABLE(huart->hdmarx);

            /* 【已删除】SCB_InvalidateDCache 操作 */

            if(rx_len > 10) {
                Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[finished_buf]);
                memset(Referee_System_Info_MultiRx_Buf[finished_buf], 0, REFEREE_RXFRAME_LENGTH); // 提取后清空
            }
        #endif
    }

    // 状态机复位
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
}
/**
  * @brief  全局串口错误回调中心 (防断联的无敌复苏装甲)
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1 || huart == &huart5)
    {
        // 1. 暴力清除所有硬件错误标志位
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

        // 2. 安抚 HAL 库，重置其状态机
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState   = HAL_UART_STATE_READY;

        // 3. 强行重启 DMA 空闲接收链路，坚决不断联！
        huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief  VOFA+ 调试协议浮点数发送
  */
void USART_Vofa_Justfloat_Transmit(float SendValue1, float SendValue2, float SendValue3)
{
    __attribute__((section (".AXI_SRAM"))) static uint8_t Rx_Buf[16];

    uint8_t *SendValue1_Pointer = (uint8_t *)&SendValue1;
    uint8_t *SendValue2_Pointer = (uint8_t *)&SendValue2;
    uint8_t *SendValue3_Pointer = (uint8_t *)&SendValue3;

    Rx_Buf[0] = *SendValue1_Pointer;     Rx_Buf[1] = *(SendValue1_Pointer + 1);
    Rx_Buf[2] = *(SendValue1_Pointer + 2); Rx_Buf[3] = *(SendValue1_Pointer + 3);

    Rx_Buf[4] = *SendValue2_Pointer;     Rx_Buf[5] = *(SendValue2_Pointer + 1);
    Rx_Buf[6] = *(SendValue2_Pointer + 2); Rx_Buf[7] = *(SendValue2_Pointer + 3);

    Rx_Buf[8] = *SendValue3_Pointer;     Rx_Buf[9] = *(SendValue3_Pointer + 1);
    Rx_Buf[10]= *(SendValue3_Pointer + 2); Rx_Buf[11]= *(SendValue3_Pointer + 3);

    Rx_Buf[12] = 0x00; Rx_Buf[13] = 0x00;
    Rx_Buf[14] = 0x80; Rx_Buf[15] = 0x7F; // VOFA+ 帧尾

    HAL_UART_Transmit_DMA(&huart7, Rx_Buf, sizeof(Rx_Buf));
}