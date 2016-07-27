/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "gpio.h"


/* USER CODE BEGIN 0 */
/*############################################################################*/
/* PWM ile ilgili bütün fonksiyonlar buraya yazýlacak. Ana programda belirlenen
senaryoya göre PWm üretimi buradaki fonksiyonlar kullanýlarak gerçekleþtirilecek.
27.07.2016 ZG */

#include "registersTIM1.h"             /* TIMER1  Register adresleri*/

/* Defines*/
#define  PERIOD_VALUE       (1200 - 1)  /* Period Value:40kHz için 1200  */
#define  PULSE1_VALUE_MAX   540         /* Maximum Capture Compare Value:%45 dutyCycle */
#define  PULSE1_VALUE       240         /* Baþlangýç dutyCycle deðeri*/
#define  DEAD_TIME          0x30        /* Complementary PWM ölü zamaný = 1u0s*/ 

#define  DELAY_VALUE         10   /* Delay Value default 100 ms  */
#define  DELTA_FRQ           3    /* Frekans tarama adýmlarý; 3=100 Hz(Clk:48Mhz)*/
#define  SWEEP_RANGE         20   /* Taramanýn kaç adýmda yapýlacaðý:1-20 adým  */

/* Fonksiyon prototipleri                                                     */
uint32_t sweepBand (uint32_t nextF, uint32_t stp, int delta, uint32_t dutyC);
void generatePwm ( uint32_t nextF, uint32_t dutyC);
uint32_t getDutyCyle (void);


/* Fonksiyonlar:                                                               */

/** PWM Frekansýný Tarama fonksiyonu*******************************************/
uint32_t sweepBand (uint32_t nextF, uint32_t stp, int delta, uint32_t dutyC)
 {
    if (stp == 0) {
    stp = 1;}
    for (uint32_t i=0; i<= stp; i++){
     nextF += delta;  /*Frekansý 1 adým artýr/azalt*/
     generatePwm (nextF, dutyC);
     delay (DELAY_VALUE); /*... ms bekle */
    }
  return (nextF -= delta);
 } 

/* PWM center frekansýnýn deðiþtirilmesi***************************************/
void generatePwm ( uint32_t nextF, uint32_t dutyC)
{
  /* Set the pulse value for channel 1 : Yani DC deðeri bu þekilde de ayarlanabilir */
  //sConfig.Pulse = PULSE1_VALUE;
  // if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  // {/* Configuration Error */
  // Error_Handler();}
  
 /*Burada periyot ve DC deðerleri kullanýlarak*/
 /* PWM üretimi için ilgili registerler yüklenecek. Hata durumu için */
 /* lib. fonksiyonlarý kullanýlabilir. */
  uint32_t *ptrToReg;
//  BSP_LED_Toggle(LED5);
  ptrToReg = &TIM1_ARR;
  *ptrToReg = nextF; /* Yeni frekans deðerini ARR registerine yükle **/
  ptrToReg = &TIM1_CCR1;
  *ptrToReg = dutyC; /* Yeni duty cycle deðerini CCR1 registerine yükle **/
  ptrToReg = &TIM1_BDTR;
 /* PWM baþlatmak için MOE biti (TIM1_BDTR registeri) set edilecek*/
  SET_BIT(*ptrToReg, 0x00008000);   /*((REG) |= (BIT))*/
}

/* Duty Cycle tesbiti**********************************************************/
uint32_t getDutyCyle (void)
{
 /*Bu fonksiyon DC deðerini elde edecek*/
  uint32_t tempDC;
  uint32_t *ptrDC;
  ptrDC = &TIM1_CCR1;
  tempDC = (*ptrDC);
  //tempDC = ((*ptrDC)+(*ptrDC)/10); /* DC %10 artýr*/
  if (tempDC >= 1100) { /* DC %90'dan büyük mü*/
    *ptrDC = 240; /* DC %10 yap*/
  } else {
    (*ptrDC) = tempDC;
  }
  return (*ptrDC);
}
/******************************************************************************/
/*############################################################################*/

/* USER CODE END 0 */





TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  
    /**TIM1 GPIO Configuration    
    PA6     ------> TIM1_BKIN 
    */
    GPIO_InitStruct.Pin = BREAK_IN_PA6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(BREAK_IN_PA6_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
  
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1 
    */
    GPIO_InitStruct.Pin = PWM_PA8_TIM1_CH1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(PWM_PA8_TIM1_CH1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  
    /**TIM1 GPIO Configuration    
    PA6     ------> TIM1_BKIN
    PA8     ------> TIM1_CH1 
    */
    HAL_GPIO_DeInit(GPIOA, BREAK_IN_PA6_Pin|PWM_PA8_TIM1_CH1_Pin);

  }
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
