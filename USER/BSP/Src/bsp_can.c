/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"

/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  // .Header.DataLength = 8,
	.Header.DataLength = FDCAN_DLC_BYTES_8,  // <======== 必须改成这个宏！绝对不能写数字8！
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
 */
// FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
//   .hcan = &hfdcan2,
//   .Header.IdType = FDCAN_STANDARD_ID,
//   .Header.TxFrameType = FDCAN_DATA_FRAME,
//   .Header.DataLength = 8,
// 	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
//   .Header.BitRateSwitch = FDCAN_BRS_ON,
//   .Header.FDFormat =  FDCAN_FD_CAN,
//   .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
//   .Header.MessageMarker = 0,
// };
//
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
	.hcan = &hfdcan2,
	.Header.IdType = FDCAN_STANDARD_ID,
	.Header.TxFrameType = FDCAN_DATA_FRAME,
	.Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
	.Header.BitRateSwitch = FDCAN_BRS_OFF,
	.Header.FDFormat =  FDCAN_CLASSIC_CAN,
	.Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
  };

/**
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // ����ID����ѡ�� ��׼ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //��ǰFDCAN��������ţ��������ö�����������˲�ͬ��ID ��������0��1��2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //������Maskģʽ �غ�������ID1��ID2������
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//ѡ���ĸ�FIFO�����գ�����CubeMX����������FIFO1�͸ĳ�FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // ������У�ֻҪID2����0x00000000�Ͳ�����˵��κ�ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //��������
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //���������õ�CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //����CAN1��ȫ�ֹ��ˣ����ǿ���������

	// 【必须新增这一行！！！】：将 FIFO1 的新消息中断路由到 中断线 1，与 NVIC 匹配！
	HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//��FIFO0���������ݽ����ж�
  
  HAL_FDCAN_Start(&hfdcan1);//ʹ��CAN1
 	
	
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;//FDCAN2过滤器结构体
	
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;//过滤器过滤ID类型
  FDCAN2_FilterConfig.FilterIndex = 0;//过滤器编号0
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;//过滤器类型：经典位屏蔽过滤器
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//过滤后进入FIFO1区
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; //消息ID过滤器
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; //过滤器屏蔽 每个位均设置0，即不过滤任何ID
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);//根据过滤器结构体中指定的参数配置FDCAN接收过滤器

  //配置 FDCAN 全局过滤器
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能中断，FIFO1新消息中断

  HAL_FDCAN_Start(&hfdcan2);//FDCAN开始工作
	
	
	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
}

/**
  * @brief  Function to transmit the FDCAN message.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
// void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){
//
//     HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,&FDCAN_TxFrame->Header,FDCAN_TxFrame->Data);
//
// }
/**
  * @brief  Function to transmit the FDCAN message.
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){

	// 【终极防御】：发送前必须检查 CAN 硬件邮箱有没有满！
	// 只有邮箱有空位时才塞数据。如果因干扰导致发送失败阻塞，新数据直接丢弃，保全系统不卡死！
	if (HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_TxFrame->hcan) > 0)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan, &FDCAN_TxFrame->Header, FDCAN_TxFrame->Data);
	}
}

/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
   
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[1]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[2]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[3]);


	// 【新增】解析 GM6020 的反馈数据
	DJI_Motor_Info_Update(Identifier, Data, &DJI_GIMBAL_Motor[0]);
	DJI_Motor_Info_Update(Identifier, Data, &DJI_GIMBAL_Motor[1]);

}

/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{



}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
	// 原有的 8009 保持不变
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[0]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[1]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[2]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[3]);

		// 【必须新增】让 CAN2 把收到的数据也喂给 4310 ！
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[0]);
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[1]);
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[2]);
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[3]);
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[4]);
	DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[5]);
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
	
  if(hfdcan == &hfdcan1){	
	
   FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	 
	}

  if(hfdcan == &hfdcan3){
	
	 FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	
	}
	
}
	
/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);
	
  FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,FDCAN_RxFIFO1Frame.Data);
	 
}