/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include <stdio.h>
ADC_HandleTypeDef      AdcHandle;
DMA_HandleTypeDef      HdmaCh1;
uint32_t  gADCxConvertedData[7]={0,0,0,0};


uint32_t adc_temp[7];

//uint8_t ADCxConvertedData[6];
ADC_ChannelConfTypeDef sConfig;
TIM_HandleTypeDef        TimHandle;
//TIM_HandleTypeDef        TimHandle3;

TIM_MasterConfigTypeDef  sMasterConfig;
TIM_OC_InitTypeDef PWMConfig;
TIM_BreakDeadTimeConfigTypeDef sBreakConfig;
OPA_HandleTypeDef OpaHandle;
UART_HandleTypeDef UartHandle;
#if 1
PMSM_Motor Motor_pra;
extern int32_t mcOffsetOffsetS,mcOffsetOffsetS1;
extern int16_t mcOffset,mcOffset1;
//PMSM_Motor Motor_pra;
//focOffset    mcOffset;
extern PUYA_SEMI_MCBase_TYPE   foc;
//focOffset mcOffset;
#endif
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcConfig(void);
static void APP_TimConfig(void);
static void UART_Config(void);
void  Motor_Pare_init(void );
void InnerComp_Init(void);  
void InnerComp1_Init(void);
void get_current(void);
void GPIO_init(void);
static void APP_PvdConfig(void);
static void APP_SystemClockConfigBackHsi(void);

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef    TimHandle;
//TIM_OC_InitTypeDef   sConfig;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief   Main program
  * @retval  int
  */
#if 1
#if (defined(__CC_ARM)) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
int fputc(int ch, FILE *f)
{	/* Send a byte to USART */	
    DEBUG_USART->DR = ch;
    while ((DEBUG_USART->SR& 0x40) == 0);	
    DEBUG_USART->SR&=0xffffffbf;
    return (ch);
}

int fgetc(FILE *f)
{	
    int ch;	
    while ((DEBUG_USART->SR& 0x20) == 0);	
    ch=DEBUG_USART->DR;
    DEBUG_USART->SR&=0xffffffdf;
    return (ch);
}
#endif
#endif

uint16_t   adcidc,theta1,cmode,cntdelay=0;
CORDIC_HandleTypeDef CordicHandle = {0};
CORDIC_ConfigTypeDef CordicCfg = {0};
CORDIC_CalculatedTypeDef CordicCalculate = {0};

OPA_HandleTypeDef OpaHandle;


extern uint16_t IQAngle1,IQAngle_motor;



int main(void)
{
    GPIO_init();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    
    /* Reset of all peripherals, Initializes the Systick */
    HAL_Init();

    /* Configure Systemclock */
    APP_SystemClockConfig(); 
    foc.SysStateId=STATE_POWERON;
    PUYA_SEMI_MC_FOC2R_INIT();

    UART_Config();

    /* 使能OPA2*/
    __HAL_RCC_OPA_CLK_ENABLE();
    // OPA->CR=1<<6;
    // OPA->OENR=1<<6;


   OpaHandle.Instance = OPA;
   OpaHandle.Init.Part = OPA1;
   HAL_OPA_Init(&OpaHandle);
   HAL_OPA_Start(&OpaHandle);

   OpaHandle.Init.Part = OPA2;
   HAL_OPA_Init(&OpaHandle);
   HAL_OPA_Start(&OpaHandle);
__HAL_RCC_HDIV_CLK_ENABLE(); 
    APP_AdcConfig();
HAL_ADC_Start_DMA(&AdcHandle,gADCxConvertedData,7);
    
	
	
    APP_TimConfig();
	generate_trig_tables();
	HAL_Delay(1000);//延时稳定，进行下面的电流偏置补偿
   // Motor_Pare_init();
    
    
    get_current_offset();

    /* Configure PVD */
    APP_PvdConfig();

    /* Enable PVD */
    HAL_PWR_EnablePVD();
    //__HAL_RCC_CORDIC_CLK_ENABLE();
     //cordic_init();

    while (1)
    {
      // if (foc.Cycle_cnt > 5000)
      // {
      //   foc.Cycle_cnt = 0;
      //   //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
      // }
     // log_out();
      // printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", foc.SinAdc,foc.CosAdc,FocSinCos.uwSitaPu,mc_PLL.SinCosPLL.uwSitaPu,mc_PLL.slWePu.sw.hi,foc.SinCosSita,gADCxConvertedData[3],foc.AdcIb,foc.SysStateId);
      // printf("%d,%d,%d,%d,%d,%d\n",gADCxConvertedData[2],gADCxConvertedData[3],foc.Ialfa,foc.Valfa,foc.Ibeta,foc.Vbeta);

      // printf("%d,%d,%d,%d,%d\n",foc.SinAdc,foc.CosAdc, FocSinCos.uwSitaPu,IQAngle1,IQAngle_motor);
      // printf("%d,%d,%d,%d,%d,%d\n",IQAngle_motor,foc.Ialfa,foc.Valfa,foc.Ibeta,foc.Vbeta,FocSinCos.uwSitaPu);
      // printf("%d,%d\n",IQAngle_motor,FocSinCos.uwSitaPu);
      // printf("%d,%d,%d,%d,%d\n",mc_stPidIq.Ref,foc.We,foc.WeLPF,mc_stPidSpd.Yk,mc_stPidSpd.ek);
      // printf("%d,%d,%d\n",IQAngle_motor,IQAngle1,foc.SysStateId);
      //  printf("%d,%d,%d,%d,%d\n",foc.We,IQAngle_motor,mc_stPidSpd.ek,mc_stPidIq.Ref,foc.WeLPF);
      // printf("%d,%d\n",foc.SysStateId,foc.MotorErrCode);

      //printf("%d,%d\n", gADCxConvertedData[CHANNEL_IU], gADCxConvertedData[CHANNEL_IV]);
      //printf("%d,%d,%d\n", ia_mea, ib_mea, ic_mea);
     // printf("%f,%f\n", Ialpha,Ibeta);
     
      //printf("%d,%d\n", Id_mea, Iq_mea);
    }
}



/**
  * @brief  Configure PVD
  * @param  None
  * @retval None
  */
static void APP_PvdConfig(void)
{
  /* Enable PWR clock and GPIOB clock */
  //GPIO_InitTypeDef  GPIO_InitStruct;
  PWR_PVDTypeDef sConfigPVD;
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

//  /* Initialize PB7 */
//  GPIO_InitStruct.Pin = GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;     /* Configure PVD for rising/falling edge interrupt */
  sConfigPVD.PVDFilter = PWR_PVD_FILTER_NONE;           /* Disable filtering */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;                 /* This parameter is invalid as PB07 is used as the detection source */
  sConfigPVD.PVDSource = PWR_PVD_SOURCE_VCC;           /* PVD detection for PB07 */
  HAL_PWR_ConfigPVD(&sConfigPVD);  
  /* Initialize PVD */
  HAL_NVIC_EnableIRQ(PVD_IRQn);
}

/**
  * @brief  PVD callback function
  * @param  None
  * @retval None
  */
void HAL_PWR_PVD_Callback(void)
{
  /* ·½°¸1£º»ùÓÚPVDÄÜ¼ì²â³öÉÏÏÂÑØÍê³É£¬×î¼òµ¥ºÍ·½±ã */
  if (__HAL_PWR_GET_FLAG(PWR_SR_PVDO))
  {
    APP_SystemClockConfigBackHsi();
    //delay_ms = 1000;
  }
  else
  {
    APP_SystemClockConfig();
    //delay_ms = 100;
  }
}

/**
  * @brief  System clock configuration function, change System clock back to HSI and disable PLL
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfigBackHsi(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Choose to configure clock HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select PLL as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB clock 1 division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB clock 1 division */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* diasble pll */
  __HAL_RCC_PLL_DISABLE();
}




/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 配置时钟源HSE/HSI/LSE/LSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                                    /* 开启HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                                    /* 不分频 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;                           /* 配置HSI输出时钟为16MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                                                   /* 关闭HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                                               /* HSE工作频率范围16M~32M */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                                                   /* 关闭LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                                                   /* 关闭LSE */
  RCC_OscInitStruct.LSEDriver = RCC_ECSCR_LSE_DRIVER_1;                                       /* LSE默认驱动能力 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                                                /* 开启PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                                        /* PLL的时钟源 */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;                                                /* PLL 2倍频输出 */

  /* 初始化RCC振荡器 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /*初始化CPU,AHB,APB总线时钟*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* RCC系统时钟类型 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;                                      /* 配置PLL为系统时钟 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                             /* APH时钟不分频 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                              /* APB时钟不分频 */
  /* 初始化RCC系统时钟 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}
//static void APP_TimConfig(void)
//{
//    __HAL_RCC_TIM1_CLK_ENABLE();                                       	/* TIM1时钟使能 */
//    TimHandle.Instance = TIM1;                                         	/* TIM1 */
//    TimHandle.Init.Period            = MOTOR_PWM_TIM1_DUTY-1;                 	/* TIM1重装载值位8000-1 */
//    TimHandle.Init.Prescaler         = 1 - 1;                        		/* 预分频为1000-1 */
//    TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;          /* 时钟不分频 */
//    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED3;  /* 向上计数 */
//    TimHandle.Init.RepetitionCounter = 0;                               /* 不重复 */
//    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 自动重装载寄存器没有缓冲 */

//    /* 基础时钟初始化 */
//    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }

//    PWMConfig.OCMode       = TIM_OCMODE_PWM1;                                     /* 输出配置为PWM1 */
//    PWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 /* OC通道输出高电平有效 */
//    PWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;                                  /* 输出快速使能关闭 */
//    PWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* OCN通道输出高电平有效 */
//    PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* 空闲状态OC1N输出低电平 */
//    PWMConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* 空闲状态OC1输出低电平 */	
//    PWMConfig.Pulse = 900;                                               /* CC1值为10，占空比=10/50=20% */
//    /* 配置通道1 */
//    #if 1
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_1) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }
//    /* 配置通道2 */
//    PWMConfig.Pulse = 1000;   
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_2) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }	
//    #endif
//    PWMConfig.Pulse = 800;   

//    /* 配置通道3 */
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_3) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }

//    /* 配置通道4 */
//    PWMConfig.Pulse = 500;    
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_4) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }
//    TIM1->CCMR2&=0x8fff;
//    TIM1->CCMR2|=0x7000;

//    sMasterConfig.MasterOutputTrigger =TIM_TRGO_OC4REF;                /* 选择更新事件作为触发源 */
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;        /* 主/从模式无作用 */
//    HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);  /* 配置TIM15*/


//    #if 1	
//    /* 开启通道1输出 */
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }
//    /* 开启通道2输出 */
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }

//    #endif
//    /* 开启通道3输出 */
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }

//    #if 1	
//    /* 开启通道1输出 */
//    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }
//    /* 开启通道2输出 */
//    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }
//    #endif	
//    /* 开启通道3输出 */
//    if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }	

//    /* 开启通道4输出 */
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
//    {
//    APP_ErrorHandler();
//    }

//    #if 1
//    __HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_UPDATE);/*使能UPDATA中断*/  
//    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);/*开启UPDATA中断请求*/
//    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,1);
//    #endif
//	
//}


static void APP_TimConfig(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();                                       	/* TIM1时钟使能 */
  TimHandle.Instance = TIM1;                                         	/* TIM1 */
  TimHandle.Init.Period            = MOTOR_PWM_TIM1_DUTY-1;                 	/* TIM1重装载值位8000-1 */
  TimHandle.Init.Prescaler         = 2 - 1;                        		/* 预分频为1000-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;          /* 时钟不分频 */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED3;  /* 向上计数 */
  TimHandle.Init.RepetitionCounter = 0;                               /* 不重复 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 自动重装载寄存器没有缓冲 */
	
  /* 基础时钟初始化 */
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

	PWMConfig.OCMode       = TIM_OCMODE_PWM1;                                     /* 输出配置为PWM1 */
  PWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 /* OC通道输出高电平有效 */
  PWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;                                  /* 输出快速使能关闭 */
  PWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* OCN通道输出高电平有效 */
  PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* 空闲状态OC1N输出低电平 */
  PWMConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* 空闲状态OC1输出低电平 */	
	PWMConfig.Pulse = 300;                                               /* CC1值为10，占空比=10/50=20% */
  /* 配置通道1 */
  #if 1
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* 配置通道2 */
  PWMConfig.Pulse = 300;   
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }	
  #endif
    PWMConfig.Pulse = 300;   

  /* 配置通道3 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
	
  /* 配置通道4 */
 //  PWMConfig.Pulse = 500;    
  PWMConfig.Pulse =MOTOR_PWM_TIM1_DUTY-20;   
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &PWMConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  TIM1->CCMR2&=0x8fff;
  TIM1->CCMR2|=0x7000;
	
  sMasterConfig.MasterOutputTrigger =TIM_TRGO_OC4REF;                /* 选择更新事件作为触发源 */
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;        /* 主/从模式无作用 */
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);  /* 配置TIM15*/
   #if 1
  sBreakConfig.BreakState       = TIM_BREAK_ENABLE;//TIM_BREAK_DISABLE;//TIM_BREAK_ENABLE;  /* 鍒硅溅鍔熻兘浣胯兘 */
  sBreakConfig.DeadTime         = DEADTIME_CNT;                                    /* 璁剧疆姝诲尯鏃堕棿 */
  sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;                       /* 杩愯妯″紡涓嬪叧闂姸鎬侀€夋嫨 OSSR=1 */
  sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;                       /* 绌洪棽鐘舵€佷笅鍏抽棴鐘舵€侀€夋嫨 OSSI=1 */
  sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;                     /* 閿佸畾鍏抽棴 */
  sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;                 /* 鍒硅溅杈撳叆浣庣數骞虫湁鏁?*/
  sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;//TIM_AUTOMATICOUTPUT_ENABLE;            /* 鑷姩杈撳嚭浣胯兘 */
  /* 鍒硅溅鍜屾鍖虹姸鍐甸厤缃?*/
  if (HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  #endif


#if 1
		  /* 开启通道1输出 */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
		  /* 开启通道2输出 */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }


		  /* 开启通道3输出 */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }


		  /* 开启通道1输出 */
  if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
		  /* 开启通道2输出 */
  if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
	
		  /* 开启通道3输出 */
  if (HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
#endif

  /* 开启通道4输出 */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
  {
    APP_ErrorHandler();
  }	


    #if 1
    __HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_UPDATE);/*使能UPDATA中断*/  
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);/*开启UPDATA中断请求*/
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,1);
    #endif
}



static void APP_AdcConfig()
{   
    __HAL_RCC_ADC_CLK_ENABLE();
    /** Common config
    */
    AdcHandle.Instance=ADC1;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;  
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 分辨率12位 */
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;//ADC_DATAALIGN_LEFT;//ADC_DATAALIGN_RIGHT;            /* 对齐方式右对齐 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;                /* 扫描方式使能 */
    AdcHandle.Init.ContinuousConvMode    = DISABLE;                        /* 单次模式 */
    // AdcHandle.Init.NbrOfConversion       = 1;                              /* 转换通道数6 */
    AdcHandle.Init.DiscontinuousConvMode =DISABLE;//             /* 间断模式不使能 */
    //AdcHandle.Init.NbrOfDiscConversion   = 1;                              /* 间断模式短序列长度为3 */
    AdcHandle.Init.ExternalTrigConv      =ADC_EXTERNALTRIGCONV_T1_CC4;  	 /* 软件触发 */
    AdcHandle.Init.ExternalTrigConvEdge  =ADC_EXTERNALTRIGCONVEDGE_RISING;// ADC_EXTERNALTRIGCONVEDGE_NONE; 
    AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN; 
    AdcHandle.Init.DMAContinuousRequests = ENABLE;
    /* ADC初始化 */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    sConfig.Channel=ADC_CHANNEL_1;//ADC_CHANNEL_OPA2_VIN;
    sConfig.Rank=ADC_REGULAR_RANK_1;
    sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES_5;  
    HAL_ADC_ConfigChannel(&AdcHandle,&sConfig);


    sConfig.Channel=ADC_CHANNEL_2;
    sConfig.Rank=ADC_REGULAR_RANK_2;
    sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES_5;  
    HAL_ADC_ConfigChannel(&AdcHandle,&sConfig);


    sConfig.Channel=ADC_CHANNEL_3;
    sConfig.Rank=ADC_REGULAR_RANK_3;
    sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES_5;  
    HAL_ADC_ConfigChannel(&AdcHandle,&sConfig);

    sConfig.Channel=ADC_CHANNEL_4;
    sConfig.Rank=ADC_REGULAR_RANK_4;
    sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES_5;  
    HAL_ADC_ConfigChannel(&AdcHandle,&sConfig);

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
    HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_6;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
    HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_7;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
    HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

    if(HAL_ADC_Calibration_Start(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }

  
}

 static void UART_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    DEBUG_USART_CLK_ENABLE();
	
    UartHandle.Instance          = USART2;
    UartHandle.Init.BaudRate     = 2000000;//
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    HAL_UART_Init(&UartHandle);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
}

void GPIO_init(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

  //   /* PA5-CH1 */
  // GPIO_InitStruct.Pin = GPIO_PIN_5;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
    
//    /* PA5-ADC */
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 


    /* EC11 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* LED */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //MD410:
    //PB6-CH3; PF5-CH2; PF8-CH1
    //PB1-CH3N;PF6-CH2N;PB13-CH1N
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* Alternate Function Push Pull Mode */
    GPIO_InitStruct.Pull = GPIO_PULLUP;            /* Pull up */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;


    /* PF8-CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    /* PB13-CH1N*/
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* PF6-CH2N*/
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    /* PF5-CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF13_TIM1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* PB6-CH3 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* PB1-CH3N */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    
}

void  Motor_Pare_init(void )  // ���������ʼ��
{

        mcOffsetOffsetS=2048;
	mcOffsetOffsetS1=2048;
	
}
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Users can add their own printing information as needed,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
