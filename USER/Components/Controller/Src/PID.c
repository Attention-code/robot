/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID functions 
  * @author         : GrassFan Wang
  * @date           : 2024/12/29
  * @version        : v1.1
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
/* Includes ------------------------------------------------------------------*/

/**
 * @brief ��ʼ��PID����.
 * @Param PID: ָ��PID_Info_TypeDef�ṹ��ָ�룬����PID����������Ϣ.
 * @Param Param: ָ��PID�����ĸ�����ָ�룬����PID������Ϣ.
 * @retval PID����״̬.
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID,float Param[PID_PARAMETER_NUM])
{
    //�ж�PID���ͺͲ����Ƿ�Ϊ�� ��Ϊ�� ����PID_FAILED_INIT����ʼ��ʧ�ܣ�
    if(PID->Type == PID_Type_None || Param == NULL)
    {
      return PID_FAILED_INIT;
    }
    
    //��ʼ��PID����
    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];
    if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f) 
		LowPassFilter1p_Init(&PID->Dout_LPF,PID->Param.Alpha);
		
		PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    //���PID�������
    PID->ERRORHandler.ErrorCount = 0;

		//����PID_ERROR_NONE���޴���״̬��
    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief ���PID����ֵ�������������0.
 * @Param PID: ָ��PID_Info_TypeDef�ṹ��ָ�룬����PID����������Ϣ.
 * @retval ��.
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
	//�����������0
	memset(PID->Err,0,sizeof(PID->Err));
	PID->Integral = 0;
		
	PID->Pout = 0;
	PID->Iout = 0;
	PID->Dout = 0;
	PID->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief ��ʼ��PID������.
 * @Param PID: ָ��PID_Info_TypeDef�ṹ��ָ�룬����PID����������Ϣ.
 * @Param Type: PID����������.
 * @Param Param:ָ��PID�����ĸ�����ָ�룬����PID������Ϣ.
 * @retval PID����״̬.
 */
void PID_Init(PID_Info_TypeDef *PID,PID_Type_e Type,float Param[PID_PARAMETER_NUM])
{

		PID->Type = Type;

		PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

		PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}
//------------------------------------------------------------------------------


/**
  * @brief �ж�PID����״̬
  * @Param PID: ָ��PID_Info_TypeDef�ṹ��ָ�룬����PID����������Ϣ.
  * @retval ��.
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
		/* Judge NAN/INF */
		if(isnan(PID->Output) == true || isinf(PID->Output)==true)
		{
				PID->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  PID���������㺯��
  * @Param  *PID pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @Param  Target  Target for the PID controller
  * @Param  Measure Measure for the PID controller
  * @retval the PID Output
  */
float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure)
{		
  /* update the PID error status */
  PID_ErrorHandle(PID);
  if(PID->ERRORHandler.Status != PID_ERROR_NONE)
  {
    PID->PID_Calc_Clear(PID);
    return 0;
  }
  
  /* update the target/measure */
  PID->Target =  Target;
  PID->Measure = Measure;

  /* update the error */
	PID->Err[2] = PID->Err[1];
	PID->Err[1] = PID->Err[0];
	PID->Err[0] = PID->Target - PID->Measure;
		
  if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
  {
		/* update the PID controller output */
		if(PID->Type == PID_POSITION)
		{
      /* Update the PID Integral */
      if(PID->Param.KI != 0)
        PID->Integral += PID->Err[0];
      else
        PID->Integral = 0;

      VAL_LIMIT(PID->Integral,-PID->Param.LimitIntegral,PID->Param.LimitIntegral);
      
      /* Update the Proportional Output,Integral Output,Derivative Output */
      PID->Pout = PID->Param.KP * PID->Err[0];
      PID->Iout = PID->Param.KI * PID->Integral;
      PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
      if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){
			
				 PID->Dout_LPF.Alpha = PID->Param.Alpha;
			   PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
			
			}
      /* update the PID output */
      PID->Output = PID->Pout + PID->Iout + PID->Dout;
      VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
		}
		else if(PID->Type == PID_VELOCITY)
		{
      /* Update the Proportional Output,Integral Output,Derivative Output */
      PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
      PID->Iout = PID->Param.KI * (PID->Err[0]);
      PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f*PID->Err[1] + PID->Err[2]);
      if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){
			
				 PID->Dout_LPF.Alpha = PID->Param.Alpha;
			   PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
			
			}
      /* update the PID output */
      PID->Output += PID->Pout + PID->Iout + PID->Dout;
      VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
		}
  }

  return PID->Output;
}
//------------------------------------------------------------------------------

