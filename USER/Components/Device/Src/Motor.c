/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/**
 * @brief The structure that contains the Information of yaw motor.Use DJI GM6020 motor.
 */
DJI_Motor_Info_Typedef DJI_GIMBAL_Motor [2] =
{
	[0] = {
		 .Type = DJI_GM6020,
		.FDCANFrame = {
					  .TxIdentifier = 0x1ff,
					  .RxIdentifier = 0x206,
		}
	},
	[1] = {
		.Type = DJI_GM6020,
		.FDCANFrame = {
					.TxIdentifier = 0x1ff,
					.RxIdentifier = 0x205,
			}
	},
};
//------------------------------------------------------------------------------

/**
 * @brief The structure that contains the Information of chassis motor.Use DJI M3508 motor.
 */ 
DJI_Motor_Info_Typedef Chassis_Motor[4] = {

    [0] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x201,
				}
    },
    [1] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x202,
				}
    },
	  [2] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x203,
				}
		},
		[3] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x204,
				}
    },

};
//------------------------------------------------------------------------------

/**
 * @brief The structure that contains the Information of joint motor.Use DM 8009 motor.
 */
DM_Motor_Info_Typedef DM_8009_Motor[2]= {
    
	  [0] = {
			.Control_Mode = MIT,
			.Param_Range ={
			   .P_MAX = 12.5f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
			},
		  .FDCANFrame = {
				 .TxIdentifier = 0x07,
				 .RxIdentifier = 0x17,
			},
		},
		
    [1] = {
			.Control_Mode = MIT,	
			.Param_Range ={
			   .P_MAX = 12.5f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
				
			},	
		  .FDCANFrame = {
				 .TxIdentifier = 0x08,
				 .RxIdentifier = 0x18,
			},
		},
		
};

/**
 * @brief The structure that contains the Information of DM J4310-2EC motor.
 *        六轴机械臂用，6个电机数组，ID 自行修改
 */
DM_Motor_Info_Typedef DM_4310_Motor[ARM_JOINT_NUM] =
{
        [0] = {
            .Control_Mode = MIT,
            .Param_Range = {
            // 【重要提醒】以下三个参数必须与你用达妙上位机连上电机后，里面显示的 PMAX, VMAX, TMAX 完全一致！
            // J4310 一般默认典型值为：
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 10.0f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x01, // TODO: 自行修改
            .RxIdentifier = 0x11, // TODO: 自行修改
        },
    },
    [1] = {
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 10.0f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x02, // TODO: 自行修改
            .RxIdentifier = 0x12, // TODO: 自行修改
        },
    },
    [2] = {
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 10.0f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x03, // TODO: 自行修改
            .RxIdentifier = 0x13, // TODO: 自行修改
        },
    },
};
      /*达妙4340电机。两个4340p，两个4340*/
DM_Motor_Info_Typedef DM_4340_Motor[4] = {
    [0] = { // 第1个电机：DM4340P
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,   // TODO: 请以上位机显示为准
            .V_MAX = 10.0f,   // TODO: 请以上位机显示为准
            .T_MAX = 28.0f    // 4340P 扭矩限幅建议先设大一点，实际以上位机为准
        },
        .FDCANFrame = {
            .TxIdentifier = 0x04, // TODO: 根据实际CAN总线分配（避免与0x01~0x0A冲突）
            .RxIdentifier = 0x14, // TODO: 需与Tx配成一对
        },
    },
    [1] = { // 第2个电机：DM4340P
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 18.0f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x0C,
            .RxIdentifier = 0x1C,
        },
    },
    [2] = { // 第3个电机：DM4340（非P版）
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 12.0f    // 普通4340扭矩限幅比P版略小
        },
        .FDCANFrame = {
            .TxIdentifier = 0x0D,
            .RxIdentifier = 0x1D,
        },
    },
    [3] = { // 第4个电机：DM4340（非P版）
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 12.5f,
            .V_MAX = 30.0f,
            .T_MAX = 12.0f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x0E,
            .RxIdentifier = 0x1E,
        },
    },
};





//------------------------------------------------------------------------------

/**
  * @brief  ������ֵת��Ϊ�Ƕ�(�ۼ� ���float���ֵ)
  */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *,float ,uint16_t );
/**
  * @brief  ������ֵת��Ϊ�Ƕ�(��Χ ����180��)
  */
static float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *,float ,uint16_t );

float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static int float_to_uint(float x, float x_min, float x_max, int bits);

//------------------------------------------------------------------------------

/**
  * @brief  CAN Transmit DJI motor (GM6020 / M3508 ID: 1~4) Information
  * @param  *FDCAN_TxFrame  pointer to the FDCAN_TxFrame_TypeDef.
  * @param  motor1_v  Control voltage/current for ID=1 (-30000 ~ 30000)
  * @param  motor2_v  Control voltage/current for ID=2 (-30000 ~ 30000) -> 对应你的 Yaw 轴
  * @param  motor3_v  Control voltage/current for ID=3 (-30000 ~ 30000)
  * @param  motor4_v  Control voltage/current for ID=4 (-30000 ~ 30000)
  * @retval None
  */
void DJI_Motor_Send_0x1FF(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, int16_t motor1_v, int16_t motor2_v, int16_t motor3_v, int16_t motor4_v)
{
	// 大疆电机 ID 1~4 的控制帧 ID 固定为 0x1FF
	FDCAN_TxFrame->Header.Identifier = 0x1FF;

	// 大疆协议：高八位在前，低八位在后
	FDCAN_TxFrame->Data[0] = (uint8_t)(motor1_v >> 8);
	FDCAN_TxFrame->Data[1] = (uint8_t)motor1_v;
	FDCAN_TxFrame->Data[2] = (uint8_t)(motor2_v >> 8);
	FDCAN_TxFrame->Data[3] = (uint8_t)motor2_v;
	FDCAN_TxFrame->Data[4] = (uint8_t)(motor3_v >> 8);
	FDCAN_TxFrame->Data[5] = (uint8_t)motor3_v;
	FDCAN_TxFrame->Data[6] = (uint8_t)(motor4_v >> 8);
	FDCAN_TxFrame->Data[7] = (uint8_t)motor4_v;

	// 调用 bsp_can.c 中的底层发送接口
	USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
  * @brief  【专属大疆 6020】CAN 发送电流控制指令
  * @param  motor1_c ~ motor4_c: 给 ID 1~4 的电流值 (-16384 ~ +16384)
  */
void DJI_Motor_Send_0x1FE(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, int16_t motor1_c, int16_t motor2_c, int16_t motor3_c, int16_t motor4_c)
{
	// 【核心关键】：这是大疆 6020 的专属电流控制 ID
	FDCAN_TxFrame->Header.Identifier = 0x1FE;

	// 大疆协议：高八位在前，低八位在后
	FDCAN_TxFrame->Data[0] = (uint8_t)(motor1_c >> 8);
	FDCAN_TxFrame->Data[1] = (uint8_t)motor1_c;
	FDCAN_TxFrame->Data[2] = (uint8_t)(motor2_c >> 8);
	FDCAN_TxFrame->Data[3] = (uint8_t)motor2_c;
	FDCAN_TxFrame->Data[4] = (uint8_t)(motor3_c >> 8);
	FDCAN_TxFrame->Data[5] = (uint8_t)motor3_c;
	FDCAN_TxFrame->Data[6] = (uint8_t)(motor4_c >> 8);
	FDCAN_TxFrame->Data[7] = (uint8_t)motor4_c;

	USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
  * @brief  Update the DJI motor Information
  * @param  Identifier  pointer to the specifies the standard identifier.
  * @param  Rx_Buf  pointer to the can receive data
  * @param  DJI_Motor pointer to a DJI_Motor_Info_t structure 
  *         that contains the information of DJI motor
  * @retval None
  */
void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* check the Identifier */
	if(*Identifier != DJI_Motor->FDCANFrame.RxIdentifier) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = Rx_Buf[6];
	DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
	DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
	DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);

	/* transform the Encoder to angle */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:

		DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192); //6020������ٱ�Ϊ1:1 ӵ�о���λ��
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,3591.f/187.f,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,36.f,8192);
		break;
		
		default:break;
	}
}
//------------------------------------------------------------------------------

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }
  
  float len = Max_Value - Min_Value;    

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a Motor_Data_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval anglesum
  */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = 0;

    /* Set the init flag */
    Data->Initlized = true;
  }
  
  /* get the possiable min Encoder err */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* transforms the Encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
	}
	else
	{
		Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
	}
  
  return Data->Angle;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to angle(-180-180)
  * @param  *Data        pointer to a Motor_Data_Typedef structure that 
	*					             contains the Data for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval angle
  */
float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,float torque_ratio,uint16_t MAXEncoder)
{	
  float Encoder_Err = 0.f;
  
  /* check the motor init */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = Data->Encoder/(MAXEncoder*torque_ratio)*360.f;

    /* config the init flag */
    Data->Initlized = true;
  }
  
  Encoder_Err = Data->Encoder - Data->Last_Encoder;
  
  /* 0 -> MAXEncoder */		
  if(Encoder_Err > MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  /* MAXEncoder-> 0 */		
  else if(Encoder_Err < -MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  else
  {
    Data->Angle += (float)(Encoder_Err)/(MAXEncoder*torque_ratio)*360.f;
  }
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* loop constrain */
  Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);

  return Data->Angle;
}

/**
  * @brief  Transmit enable disable save zero position Command to DM motor 
  * @param  *FDCAN_TxFrame��pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *DM_Motor��pointer to the DM_Motor
  * @param  CMD��Transmit Command  (DJI_Motor_Type_e)
  * @retval None
  */
// void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD){
//
// 	 // FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.RxIdentifier;
// 	// 必须改成 TxIdentifier
// 	FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
//
// 	 FDCAN_TxFrame->Data[0] = 0xFF;
//    FDCAN_TxFrame->Data[1] = 0xFF;
//  	 FDCAN_TxFrame->Data[2] = 0xFF;
// 	 FDCAN_TxFrame->Data[3] = 0xFF;
// 	 FDCAN_TxFrame->Data[4] = 0xFF;
// 	 FDCAN_TxFrame->Data[5] = 0xFF;
// 	 FDCAN_TxFrame->Data[6] = 0xFF;
//
// 	 switch(CMD){
//
// 		  case Motor_Enable :
// 	        FDCAN_TxFrame->Data[7] = 0xFC;
// 	    break;
//
// 			case Motor_Disable :
// 	        FDCAN_TxFrame->Data[7] = 0xFD;
//       break;
//
// 			case Motor_Save_Zero_Position :
// 	        FDCAN_TxFrame->Data[7] = 0xFE;
// 			break;
//
// 			default:
// 	    break;
// 	}
//
//    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
//
// }

/**
  * @brief  Transmit enable, disable, save zero position, or clear error Command to DM motor
  * @param  *FDCAN_TxFrame：pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *DM_Motor：pointer to the DM_Motor
  * @param  CMD：Transmit Command
  * @retval None
  */
void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, uint8_t CMD)
{
	// 强制使用 TxIdentifier 进行命令下发
	FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;

	FDCAN_TxFrame->Data[0] = 0xFF;
	FDCAN_TxFrame->Data[1] = 0xFF;
	FDCAN_TxFrame->Data[2] = 0xFF;
	FDCAN_TxFrame->Data[3] = 0xFF;
	FDCAN_TxFrame->Data[4] = 0xFF;
	FDCAN_TxFrame->Data[5] = 0xFF;
	FDCAN_TxFrame->Data[6] = 0xFF;

	switch(CMD){
		case Motor_Enable :
			FDCAN_TxFrame->Data[7] = 0xFC;
			break;
		case Motor_Disable :
			FDCAN_TxFrame->Data[7] = 0xFD;
			break;
		case Motor_Save_Zero_Position :
			FDCAN_TxFrame->Data[7] = 0xFE;
			break;
			// 【赛级补丁】：0xFB 达妙专属清除错误指令
		case 0xFB :
			FDCAN_TxFrame->Data[7] = 0xFB;
			break;
		default:
			break;
	}

	USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
  * @brief  RM赛级专属：带状态自检与自愈的达妙控制函数
  * @param  全参数同 DM_Motor_CAN_TxMessage
  */
void DM_Motor_Ctrl_Safe(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, float Postion, float Velocity, float KP, float KD, float Torque)
{
	// 1. 检查电机是否处于正常工作状态 (State == 1)
	if (DM_Motor->Data.State != 1)
	{
		// 如果处于 13 号报错状态，偷偷发一帧清错指令
		if (DM_Motor->Data.State == 13) {
			DM_Motor_Command(FDCAN_TxFrame, DM_Motor, 0xFB);
		}
		// 只要不是 1，就发使能指令，直到把它叫醒
		DM_Motor_Command(FDCAN_TxFrame, DM_Motor, Motor_Enable);
	}
	else
	{
		// 2. 只有在完全健康的状态下，才下发真实的控制指令
		DM_Motor_CAN_TxMessage(FDCAN_TxFrame, DM_Motor, Postion, Velocity, KP, KD, Torque);
	}
}

/**
  * @brief  CAN Transmit DM motor Information
  * @param  *FDCAN_TxFrame  pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *DM_Motor  pointer to the DM_Motor
  * @param  Postion Velocity KP KD Torgue: Target
  * @retval None
  */
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){
	
   if(DM_Motor->Control_Mode == MIT){
		 
		 uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
		 
		 Postion_Tmp  =  float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16) ;
		 Velocity_Tmp =  float_to_uint(Velocity,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
		 Torque_Tmp   =  float_to_uint(Torque,  -DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
		 KP_Tmp = float_to_uint(KP,0,500,12);
		 KD_Tmp = float_to_uint(KD,0,5,12);
		
		 FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
		 
		 FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
		 FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
		 FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
		 FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
		 FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
		 FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
		 FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
		 FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);

	}else if(DM_Motor->Control_Mode == POSITION_VELOCITY){
	
		 uint8_t *Postion_Tmp,*Velocity_Tmp;
		
		 Postion_Tmp  = (uint8_t*) & Postion;
		 Velocity_Tmp = (uint8_t*) & Velocity;
		
	   FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;
		
		 FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
		 FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
		 FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
		 FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
	   FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
		 FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
		 FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
		 FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
		
	}else if(DM_Motor->Control_Mode == VELOCITY){
	
	  uint8_t *Velocity_Tmp;
		Velocity_Tmp = (uint8_t*) & Velocity;
		
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;
		
		FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
		FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
		FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
		FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
		FDCAN_TxFrame->Data[4] = 0;
 		FDCAN_TxFrame->Data[5] = 0;
		FDCAN_TxFrame->Data[6] = 0;
		FDCAN_TxFrame->Data[7] = 0;

	}
	 
	  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);

}
//------------------------------------------------------------------------------

/**
  * @brief  Update the DM_Motor Information
  * @param  Identifier:  pointer to the specifies the standard identifier.
  * @param  Rx_Buf:  pointer to the can receive data
  * @param  DM_Motor: pointer to a DM_Motor_Info_Typedef structure that contains the information of DM_Motor
  * @retval None
  */
// void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor)
// {
//
// 	// if(*Identifier != DM_Motor->FDCANFrame.RxIdentifier) return;
//
// 	  DM_Motor->Data.State = Rx_Buf[0]>>4;
// 		DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));
// 		DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);
// 		DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));
// 		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
// 		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
//     DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
//
//     DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
// 		DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);
//
// }

void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor)
{
	// 提取真正的电机 ID（藏在数据第0字节的低4位，主要针对 4310）
	uint8_t feedback_motor_id = Rx_Buf[0] & 0x0F;

	// 【核心修复】：兼容两种返回模式
	// 模式1：传统的 RxIdentifier 匹配 (服务于你的 8009 电机)
	// 模式2：Master ID (0x00) 匹配 (服务于这台 4310 电机)
	if((*Identifier == DM_Motor->FDCANFrame.RxIdentifier) ||
	   (*Identifier == 0x00 && feedback_motor_id == DM_Motor->FDCANFrame.TxIdentifier))
	{
		// 校验通过，开始解析数据
		DM_Motor->Data.State = Rx_Buf[0]>>4;
		DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));
		DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);
		DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
		DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);

		DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);
	}
}
//------------------------------------------------------------------------------	
	
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
	
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
