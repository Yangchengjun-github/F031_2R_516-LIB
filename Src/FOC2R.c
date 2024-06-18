/**
  ******************************************************************************
  * @file    FOC2R.c
  * @author  MCU Application Team
  * @Version V1.0.0
  * @Date    2022-11-11
  * @brief   BLDC FOC Sensorless under 2R Sensing Current
  ******************************************************************************
********************************************************************************
 Copyright (c) 2022 Puya Semiconductor Co., Ltd.
 Creating by Michael.Wang(Wang Weizi)
 All rights reserved.
********************************************************************************
 .....
********************************************************************************/

/************************************************************************
 Included File:
*************************************************************************/
#include "main.h"
extern void  PUYA_SEMI_MC_SMO_Calc(P_PYFOC_SMO *foc_smo);
extern void PUYA_SEMI_MC_FocPWMduty_Calc(PUYA_SEMI_MCBase_TYPE *mcfoc);
PUYA_SEMI_MC_PLL_Base_TYPE mc_PLL;	
PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidId;
PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidIq;
PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidSpd;

PUYA_SEMI_MC_FOC2R_PI_TYPE mc_stPidposition;
SIN_COS FocSinCos;
//extern LL_GPIO_InitTypeDef TIM1CH1MapInit;
uint8_t counter=0;
extern int16_t mcOffset,mcOffset1;
extern uint16_t IQAngle_motor,IQAngle1;
PUYA_SEMI_MCBase_TYPE   foc;
extern void GetmcOffset(void);
uint16_t IQAngle_0,IQAngle_motor1=0;
//******************************************************************************************************************************//
/**
  * @brief  Motor MC_FOC2R_Schedule 
  * @param  None
  * @retval None
  */
int16_t focIqCmd;
uint32_t focSita;
extern uint32_t   gADCxConvertedData[4];
__attribute__((section("RAMCODE")))
void PUYA_SEMI_MC_FOC2R_Schedule(void)
{  

   int16_t  value;
   int32_t temp;
   

           
	       foc.SinAdc=gADCxConvertedData[2]-2030;
	       foc.CosAdc=gADCxConvertedData[3]-2030;
               angle_cla();
               
              
   if(foc.SysStateId)
		{
			if((foc.Ialfa>MAX_CUR_THRESHOLD)||(foc.Ialfa<-MAX_CUR_THRESHOLD)||(foc.Ibeta>MAX_CUR_THRESHOLD)||(foc.Ibeta<-MAX_CUR_THRESHOLD))
			{
				foc.MotorErrCode=1;
				foc.SysStateId=STATE_STOP_ERROR;
			}
		}
	  foc.Id=(foc.Ialfa*FocSinCos.swCosPu>>15)+(foc.Ibeta*FocSinCos.swSinPu>>15);
    foc.Iq=(foc.Ibeta*FocSinCos.swCosPu>>15)-(foc.Ialfa*FocSinCos.swSinPu>>15); 
		
		foc.PwmIsr_cnt++;
		foc.Cycle_cnt++;
		if(counter>20)
			{
      if(IQAngle_motor1>=IQAngle_motor)
			foc.We=IQAngle_motor1-IQAngle_motor;
			else
		  foc.We=0xffff-IQAngle_motor+IQAngle_motor1;
			IQAngle_motor1=IQAngle_motor;
			value=foc.We>>3;
		  temp=foc.WeLPF>>3;
			foc.WeLPF=foc.WeLPF+value-temp;
			counter=0;
			}
		


	/******************************************************************************************************/
	//StateMachine Control
	/******************************************************************************************************/	
	switch(foc.SysStateId)
	{
	 /*************************************************************************************************************************************************************************************************************/
		case STATE_POWERON:// STATE_POWERON
				
			PUYA_SEMI_MC_FocPWM_ON();
			/******************************************************************************************************/
			// 2.Current Offset Calc
			/******************************************************************************************************/	
			GetmcOffset();
		
			/******************************************************************************************************/
			// 2.System-State-Transition Control
			/******************************************************************************************************/
			if(foc.PwmIsr_cnt>1000)
			{
					foc.SysStateId=STATE_STOP_NOMARL;
					foc.PwmIsr_cnt=0;
			}    

			break;

	 /*************************************************************************************************************************************************************************************************************/
		case STATE_STOP_NOMARL:
			
			{
					foc.SysStateId=STATE_RUN_DC;
					mc_stPidIq.Ref=1000;//1000;
					mc_stPidId.Ref=0;
					foc.PwmIsr_cnt=0;
					PUYA_SEMI_MC_PLL_Init(&mc_PLL);
			    focIqCmd=0;
				  mc_stPidSpd.Ref=1000;
					mc_stPidSpd.swOut=500;
			}
        
			break;	
			case STATE_RUN_DC:
                           
					mc_stPidId.Ref=0;
					mc_stPidId.Yk=foc.Id;        
					PUYA_SEMI_MC_FOC2R_PI_Ctrl(&mc_stPidId);
					foc.Vd=mc_stPidId.swOut;

					mc_stPidIq.Max=pySqrt_31000(foc.Vd);
					mc_stPidIq.Min=-mc_stPidIq.Max;


					mc_stPidSpd.Yk=foc.We;
					PUYA_SEMI_MC_FOC2R_SPDPI_Ctrl(&mc_stPidSpd);
					mc_stPidIq.Ref=mc_stPidSpd.swOut;

					mc_stPidIq.Yk=foc.Iq;
					PUYA_SEMI_MC_FOC2R_PI_Ctrl(&mc_stPidIq);
					foc.Vq=mc_stPidIq.swOut;
					FocSinCos.uwSitaPu=IQAngle_motor;
					voTabSinCos(&FocSinCos);
					foc.wSin=FocSinCos.swSinPu;
					foc.wCos=FocSinCos.swCosPu;
					PUYA_SEMI_MC_FocPWMduty_Calc(&foc);
					break;
		case STATE_STOP_ERROR:// STATE_STOP_ERROR
			/******************************************************************************************************/
			// 1.Mission Service of Current System-State
			/******************************************************************************************************/           
			PUYA_SEMI_MC_FocPWM_OFF();
			
			/******************************************************************************************************/
			// 2.System-State-Transition Control
			/******************************************************************************************************/
			if(foc.MotorErrCode==0)
			{
					foc.SysStateId=STATE_STOP_NOMARL;
			}
			break;
			
	}			
}

#if 1
void PUYA_SEMI_MC_FOC2R_INIT(void)
{
	mc_stPidId.Ref=0;mc_stPidId.ek=0;mc_stPidId.ek_1=0;mc_stPidId.dek=0;mc_stPidId.swOut=0;
	mc_stPidIq.Ref=0;mc_stPidIq.ek=0;mc_stPidIq.ek_1=0;mc_stPidIq.dek=0;mc_stPidIq.swOut=0;
	mc_stPidSpd.Ref=0;mc_stPidSpd.ek=0;mc_stPidSpd.ek_1=0;mc_stPidSpd.dek=0;mc_stPidSpd.swOut=0;

	mc_stPidId.Kp=250;mc_stPidId.Ki=250;mc_stPidId.Max=30000;mc_stPidId.Min=-mc_stPidId.Max;  mc_stPidId.ek=0;mc_stPidId.ek_1=0;mc_stPidId.dek=0;mc_stPidId.swOut=0;
	mc_stPidIq.Kp=250;mc_stPidIq.Ki=250;mc_stPidIq.Max=30000;mc_stPidIq.Min=-mc_stPidIq.Max;  mc_stPidIq.ek=0;mc_stPidIq.ek_1=0;mc_stPidIq.dek=0;mc_stPidIq.swOut=0;
	//mc_stPidSpd.Kp=1000;mc_stPidSpd.Ki=100;mc_stPidSpd.Max=10000;mc_stPidSpd.Min=-1000;     

     
	mc_stPidSpd.Kp=1500;mc_stPidSpd.Ki=500;mc_stPidSpd.Max=3000;mc_stPidSpd.Min=-3000;    
}
#endif


void PUYA_SEMI_MC_FocPWM_OFF(void)
{
	TIM1->CCER&=0xfaaa;
	TIM1->BDTR&=0xf7ff;
}

void PUYA_SEMI_MC_FocPWM_ON(void)
{
	TIM1->CCER|=0x555;
	TIM1->BDTR|=0x8800;
}

void PUYA_SEMI_MC_PLL_Init(PUYA_SEMI_MC_PLL_Base_TYPE *PLL)
{
	//======================================================================================================================================//
	//  Var Set
	//======================================================================================================================================//
	PLL->ValfaLPF.sl = 0;
	PLL->VbetaLPF.sl = 0;
	PLL->IalfaLPF.sl = 0;
	PLL->IbetaLPF.sl = 0;
	PLL->IalfaLPF0 = 0;
	PLL->IbetaLPF0 = 0;
	PLL->EalfaLPF = 0;
	PLL->EbetaLPF = 0;
	PLL->We = 0;
	PLL->EdError0 = 0;
	PLL->WeLPF = 0;
	PLL->slPLLSitaPu.sl = 0;
	PLL->Ed = 0;
	PLL->SinCosFOC.uwSitaPu = 0;
	PLL->SinCosPLL.uwSitaPu = 0;
	PLL->slWePu.sl = 0;

	// fbase=150Hz,  We=32767---->150Hz
	//======================================================================================================================================//
	//  Motor-Type Setting
	//======================================================================================================================================//
	PLL->swLsDivTc[3]=20; 	
	PLL->swRs[3]=2457;		
	PLL->InitSitaCWC[3]=0;			
	PLL->InitSitaCCW[3]=0;
    
    
//	PLL->swLsDivTc[3]=2975; 	PLL->swRs[3]=2047;		PLL->InitSitaCWC[3]=0;			PLL->InitSitaCCW[3]=0;		PLL->InitIqRef[3]=5000;		PLL->MaxIqRef[3]=6000;	//10000;	//OK  	
//	PLL->swLsDivTc[3]=1312; 	PLL->swRs[3]=5215;		PLL->InitSitaCWC[3]=0;			PLL->InitSitaCCW[3]=0;		PLL->InitIqRef[3]=5000;		PLL->MaxIqRef[3]=6000;	//10000;	//OK  
	PLL->swLsDivTc[3]=2000; 	PLL->swRs[3]=5215;		PLL->InitSitaCWC[3]=0;			PLL->InitSitaCCW[3]=0;		PLL->InitIqRef[3]=5000;		PLL->MaxIqRef[3]=6000;	//10000;	//OK  
	
	// No.21 Motor is Selected Here
	PLL->MotorTypeID=3;	PLL->swLsDivTcPu=PLL->swLsDivTc[PLL->MotorTypeID];	PLL->swRsPu=PLL->swRs[PLL->MotorTypeID];

	PLL->swDetaSitaPu=PLL->InitSitaCWC[PLL->MotorTypeID];
	
	PLL->swTc2PiPu=3700;//2200;//983 @20kHz, fbase=300Hz
    PLL->swWeMaxPu=32000;
	PLL->swLpfCofPu=30000;
	
	PLL->swKpPu=400;//500;
	PLL->swKitPu=100;//100;
	PLL->swCurTh=170;
}			

//******************************************************************************************************************************//

/*************************************************************************
 End of this File (EOF):
 Do not put anything after this part!
*************************************************************************/
