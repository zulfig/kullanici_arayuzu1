/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/


/* USER CODE BEGIN 1 */
/*############################################################################*/
/* GPIO iþlerini buradan yapacaðýz                                            */
/* Displayi sürmek için PC0 ve PC1 pinlerinden display modülüne STCP sinyali  */
/* verilecek*/    
 
  void outDisplayData ( uint8_t rakamHane)
  {
   uint8_t pinNo;
   /* 6 adet 7-seg display sürebilecek bir fonksiyon*/
   switch(rakamHane) {
      case 0 :
         pinNo = GPIO_PIN_0; /* PC0 portundan STCP sinyali*/
         break;
     case 1 :
         pinNo = GPIO_PIN_4; /* PC4 portundan STCP sinyali*/
         break;
/*     case 2 :
         pinNo = GPIO_PIN_2;
         break;
     case 3 :
         pinNo = GPIO_PIN_3;
         break;    
     case 4 :
         pinNo = GPIO_PIN_4;
         break;    
     case 5 :
         pinNo = GPIO_PIN_5;
         break;    
*/
         
      default :
         /* Geçersiz pin numarasý gönderilmiþ; Hata mesajý verilebilir */
         pinNo = GPIO_PIN_0;
   }
    
//void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
/*  Ýlgili 7-seg display için latch sinyalini yükselen kenar olarak gönder    */
    HAL_GPIO_WritePin (GPIOC, pinNo, GPIO_PIN_RESET);
    delay(1);
    HAL_GPIO_WritePin (GPIOC, pinNo, GPIO_PIN_SET);
    delay(1);
    HAL_GPIO_WritePin (GPIOC, pinNo, GPIO_PIN_RESET);
  }

/*############################################################################*/    
/* USER CODE END 1 */



/** Configure pins  */

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SERIAL_CHIPSELECT_0_Pin|SERIAL_CHIPSELECT_2_Pin
                           |EXT_RESET_Pin|LD3_Pin |LD6_Pin|LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(SERIAL_CHIPSELECT_2_GPIO_Port, SERIAL_CHIPSELECT_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin 
                           PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SERIAL_CHIPSELECT_0_Pin|SERIAL_CHIPSELECT_2_Pin
                        |EXT_RESET_Pin|LD3_Pin |LD6_Pin|LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin 
                           PAPin PAPin */
  GPIO_InitStruct.Pin = SET_PA1_Pin|UP_PA1_Pin|DOWN_PA2_Pin|MENU_PA3_Pin 
                          |DEGAS_PA4_Pin|PULSE_PA5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin 
  GPIO_InitStruct.Pin = SERIAL_CHIPSELECT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERIAL_CHIPSELECT_2_GPIO_Port, &GPIO_InitStruct); */

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
