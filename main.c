/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f072b_discovery.h"

/* USER CODE BEGIN Includes */
#include "registersTIM1.h"             /* TIMER1  Register adresleri*/
#include "registerOperations.h"        /* Register iþlemleri için tanýmlar*/
#include "registersNVIC.h"             /* NVIC  Register adresleri*/
#include "registersEXTI.h"             /* EXTI  Register adresleri*/
#include "registersGPIOA.h"            /* GPIOA Register adresleri*/
#include "pendingBitsEXTI.h"           /* EXTIn lines için pending bitleri:PendingBitn*/

#define  PERIOD_VALUE       (1200 - 1)  /* Period Value:40kHz için 1200  */
#define  PULSE1_VALUE_MAX   540         /* Maximum Capture Compare Value:%45 dutyCycle */
#define  PULSE1_VALUE       240         /* Baþlangýç dutyCycle deðeri*/
#define  DEAD_TIME          0x30        /* Complementary PWM ölü zamaný = 1u0s*/ 

#define  DELAY_VALUE         10   /* Delay Value default 100 ms  */
#define  DELTA_FRQ           3    /* Frekans tarama adýmlarý; 3=100 Hz(Clk:48Mhz)*/
#define  SWEEP_RANGE         20   /* Taramanýn kaç adýmda yapýlacaðý:1-20 adým  */

#define  IRQn01              5    /* EXTI0_1 line için IRQ numarasý*/
#define  IRQn23              6    /* EXTI2_3 line için IRQ numarasý*/
#define  IRQn415             7    /* EXTI4_15 line için IRQ numarasý*/
#define  PendingBit0 (0x00000001) /* EXTI0 line Pending bit;Hepsi .h dosyasýnda*/

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//typedef struct
//{
//  volatile uint32_t ISER[1U];         /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
//        uint32_t RESERVED0[31U];
//  volatile uint32_t ICER[1U];      /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
//        uint32_t RSERVED1[31U];
//  volatile uint32_t ISPR[1U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
//        uint32_t RESERVED2[31U];
//  volatile uint32_t ICPR[1U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
//        uint32_t RESERVED3[31U];
//        uint32_t RESERVED4[64U];
//  volatile uint32_t IP[8U];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register */
//}  NVIC_TypeZG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*Benim yazdýðým fonksiyonlarýn prototipleri*/
uint32_t sweepBand (uint32_t nextF, uint32_t stp, int delta, uint32_t dutyC);
void generatePwm (uint32_t nextF, uint32_t dutyC);
uint32_t getDutyCyle (void);
void delay (uint32_t msWait);

void MENU_Function(void); /* MENU butonu iþlemleri*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Global variables */
 uint32_t temp1=0,intSource = 99;
 uint32_t *ptr;

/* Global static variables. They must be in non-volatile memory*/
static uint32_t opData[4][8];
static uint32_t workingSet[4];
uint32_t tempD[2][2];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  int32_t  tara=0;
  uint32_t temp=0;
  uint32_t *ptrToRegGenel, *pR, *ptr;
  uint32_t step[21]={1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,10};
  uint32_t numberOfSteps=1;
  uint32_t centerFr = PERIOD_VALUE;/*Tarama merkez frekansý. 1200 için 40kHz*/
  uint32_t nextFr = centerFr; /* Tedbiren ilk deðer veriliyor*/
  int delta = DELTA_FRQ;
  uint32_t dutyCycleSet = PULSE1_VALUE; /* DC için baþlangýç deðeri*/
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
   /* Disable interrupts for channel 2,3,4 and DMA*/
  ptr = &TIM1_DIER;
  *ptr &= (0x000000C3);
 
  numberOfSteps = SWEEP_RANGE;/* Taramanýn kaç adýmda yapýlacaðý*/
  if ((numberOfSteps) < 0 | (numberOfSteps > 20)) { 
  Error_Handler(); /* Adým sayýsý sýnýrlarýn dýþýnda ise hata ver*/
  }
  
  /* Configure LED4-6: izleme amaçlý kullanýlabilir */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4); 
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Off(LED3) ; /*LED3 söndür*/
   
  /* Start PWM signal generation on TIM1_Channel 1 */ 
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  { Error_Handler();} /* Configuration Error */
  

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (tara=0; tara <5; tara++){
      MENU_Function ();}
    
    for (tara=0; tara <8; tara++){
      UP_Function ();}
    
    for (tara=0; tara <15; tara++){
      DOWN_Function ();}
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* Private functions ---------------------------------------------------------*/


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
  BSP_LED_Toggle(LED5);
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

 void delay (uint32_t msWait)
 {
  /*msWait ms bekleyen fonksiyon*/
  // BSP_LED_On(LED5); /*LED5 yak*/
   HAL_Delay(msWait);
  //BSP_LED_Off(LED5); /*LED5 söndür*/
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  while (1)
  {
    BSP_LED_Toggle(LED3);
    BSP_LED_Toggle(LED4);
    BSP_LED_Toggle(LED5);
    BSP_LED_Toggle(LED6);
    delay(500);
  /* USER CODE END Error_Handler */ 
}
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
