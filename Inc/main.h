/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU Application Team
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"
#include "py32f031xx_Start_Kit.h"

/* Private includes ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
typedef int8_t      SBYTE; //1 bytes signed
typedef uint8_t     UBYTE; //1 bytes unsigned
typedef int16_t     SWORD; //2 bytes signed
typedef uint16_t    UWORD; //2 bytes unsigned
typedef int32_t     SLONG; //4 bytes signed
typedef uint32_t    ULONG; //4 bytes unsigned
typedef int64_t    	SQWORD;//8 bytes unsigned
typedef uint64_t  	UQWORD;//8 bytes unsigned
typedef void        VOID;

/*----------------------------------------------------------------*/
typedef union{
          struct{           
            UWORD low;
            SWORD hi;             
          }sw;
        SLONG sl;
}SLONG_UNION;
/*----------------------------------------------------------------*/
typedef union{
          struct{
            UWORD low;
            UWORD hi; 
          }uw;
        ULONG ul;
}ULONG_UNION;



//#include "uartHMI.h"
	
#include "FOC2R.h"
#include "PuyaMath.h"
#include "foc.h"	
//extern UART_HandleTypeDef UartHandle;

enum
{
  CHANNEL_IV = 0,
  CHANNEL_IU,
  CHANNEL_VU,
  CHANNEL_VV,
  CHANNEL_BUS,
  CHANNEL_VW,
  CHANNEL_,
};


extern TIM_OC_InitTypeDef PWMConfig;
extern TIM_HandleTypeDef TimHandle;
extern uint32_t gADCxConvertedData[7]  ;

extern uint32_t adc_temp[7];
#define   ADCBUGLENGTH 6000

/* Private includes ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Exported variables prototypes ---------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
