/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#include "spi.h"

#include "gpio.h"




/* USER CODE BEGIN 0 */
/*############################################################################*/
/* Seri iletiþim ile ilgili bütün fonksiyonlar buraya yazýlacak. Ana programda 
belirlenen deðerler buradaki fonksiyonlar kullanýlarak 'display' edilecek.
27.07.2016 ZG */

/* Fonksiyon prototipleri:                                                    */
void seriPortaYaz (uint8_t *ptrDizi, uint8_t rakamSayisi);



/* Fonksiyonlar:                                                               */


/* opData[4][8] dizisi içinde display edilecek veri bulunuyor.
   opData[1][1]: Temperature
   opData[2][1]: Time
   opData[3][1]: UPower */
 
// HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/* Bu fonksiyon ile 8-bit veri (*ptrRakam) SPI2_OUTPUT (MOSI) portundan     */
/* CLK sinyali ile birlikte baðlanan birime (7-SEG Display) gönderilir.*/

void seriPortaYaz (uint8_t *ptrDizi, uint8_t rakamSayisi)
{
 while (rakamSayisi > 0)
   {
    HAL_SPI_Transmit(&hspi2, ptrDizi, 1, 1);/*Rakamý display Shift Reg.e yükle*/
    outDisplayData (rakamSayisi-1);    /* Ýlgili haneye yazdýr           */
    rakamSayisi--;                     /* Hane sayýsýný azalt ve devam et*/
    ptrDizi++;                         /* Göstergeyi ilerlet */
   }
  delay(500); 
  rakamSayisi--;
 }




/*############################################################################*/




/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
  //  PB10     ------> SPI2_SCK 
    PB13     ------> SPI2_SCK :Deðiþtirdim. Sanki problem oluyordu
    */
    GPIO_InitStruct.Pin = SERIAL_INPUT_Pin|SERIAL_OUTPUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = SERIAL_CLOCK_Pin;
    GPIO_InitStruct.Pin = GPIO_PIN_13; /*SERIAL_CLOCK_GPIO_Pin yerine */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2; /* AF5 yerine AF0*/
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    //PB10     ------> SPI2_SCK 
    PB13     ------> SPI2_SCK :Deðiþtirdim. 10 Problem oluyordu
    */
    HAL_GPIO_DeInit(GPIOC, SERIAL_INPUT_Pin|SERIAL_OUTPUT_Pin);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13); /*GPIOB yazýldý*/

  }
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
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
