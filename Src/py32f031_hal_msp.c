/**
  ******************************************************************************
  * @file    py32f031_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief Initialize global MSP
  */
extern DMA_HandleTypeDef HdmaCh1;
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize TIM1 related MSP
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /* Enable TIM1 clock */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* Alternate Function Push Pull Mode */
  GPIO_InitStruct.Pull = GPIO_PULLUP;            /* Pull up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  #if 0
  /* Initialize GPIOA8 */
  GPIO_InitStruct.Pin = TIMx_GPIO_PIN_CHANNEL1;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(TIMx_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);
  /* Initialize GPIOA9 */
  GPIO_InitStruct.Pin = TIMx_GPIO_PIN_CHANNEL2;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(TIMx_GPIO_PORT_CHANNEL2, &GPIO_InitStruct);
  /* Initialize GPIOA10 */
  GPIO_InitStruct.Pin = TIMx_GPIO_PIN_CHANNEL3;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(TIMx_GPIO_PORT_CHANNEL3, &GPIO_InitStruct);
  /* Initialize GPIOA11 */
  GPIO_InitStruct.Pin = TIMx_GPIO_PIN_CHANNEL4;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(TIMx_GPIO_PORT_CHANNEL4, &GPIO_InitStruct);
  #endif
}
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
  __HAL_RCC_SYSCFG_CLK_ENABLE();                              /* SYSCFG clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();                                 /* ADC clock enable */
  __HAL_RCC_DMA_CLK_ENABLE();                                 /* DMA clock enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();                               /* GPIOA clock enable */
  
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

  #if 0
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  #endif
  
  HdmaCh1.Instance                 = DMA1_Channel1;           /* Select DMA channel 1 */
  HdmaCh1.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* Direction from peripheral to memory */
  HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Prohibit peripheral address increment */
  HdmaCh1.Init.MemInc              = DMA_MINC_ENABLE;         /* Enable memory address increment */
  HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;     /* Peripheral data width is 32 bits */
  HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;     /* Memory data width 32-bit */
  HdmaCh1.Init.Mode                = DMA_CIRCULAR;            /* Cycle Mode */
  HdmaCh1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;  /* Channel priority is very high */

  HAL_DMA_DeInit(&HdmaCh1);                                   /* DMA deinitialization */
  HAL_DMA_Init(&HdmaCh1);                                     /* Initialize DMA channel 1 */
  
  HAL_DMA_ChannelMap(&HdmaCh1,DMA_CHANNEL_MAP_ADC);
  __HAL_LINKDMA(hadc, DMA_Handle, HdmaCh1);                   /* Connect DMA handle */
}
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
