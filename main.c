/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f072b_discovery.h"
#include "stm32f072b_discovery.c"

/* USER CODE BEGIN Includes */
#include "registersTIM1.h"             /* TIMER1  Register adresleri*/
#include "registerOperations.h"        /* Register iþlemleri için tanýmlar*/
#include "registersNVIC.h"             /* NVIC  Register adresleri*/
#include "registersEXTI.h"             /* EXTI  Register adresleri*/
#include "registersGPIOA.h"            /* GPIOA Register adresleri*/
#include "pendingBitsEXTI.h"           /* EXTIn lines için pending bitleri:PendingBitn*/

//#define  PERIOD_VALUE       (1200 - 1)  /* Period Value:40kHz için 1200  */
//#define  PULSE1_VALUE_MAX   540         /* Maximum Capture Compare Value:%45 dutyCycle */
//#define  PULSE1_VALUE       240         /* Baþlangýç dutyCycle deðeri*/
//#define  DEAD_TIME          0x30        /* Complementary PWM ölü zamaný = 1u0s*/ 

#define  PERIOD_VALUE       (1734 - 1)  /* Period Value:27.685kHz için 1734  */
#define  PULSE1_VALUE_MAX   780         /* Maximum Capture Compare Value:%45 dutyCycle */
#define  PULSE1_VALUE       350         /* Baþlangýç dutyCycle deðeri*/
   
#define  DELAY_VALUE         10   /* Delay Value default 100 ms  */
#define  DELTA_FRQ           6    /* Frekans tarama adýmlarý; 6=96 Hz(Clk:48Mhz)*/
#define  SWEEP_RANGE         20   /* Taramanýn kaç adýmda yapýlacaðý:1-20 adým  */

#define  IRQn01              5    /* EXTI0_1 line için IRQ numarasý*/
#define  IRQn23              6    /* EXTI2_3 line için IRQ numarasý*/
#define  IRQn415             7    /* EXTI4_15 line için IRQ numarasý*/

#define  INT_PRIORITY_0      0x00  /* Interrupt priority : 0 (Highest)  */
#define  INT_PRIORITY_1      0x40  /* Interrupt priority : 64           */
#define  INT_PRIORITY_2      0x80  /* Interrupt priority : 128          */
#define  INT_PRIORITY_3      0xC0  /* Interrupt priority : 192 (Lowest) */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*Benim yazdýðým fonksiyonlarýn prototipleri*/
//uint32_t sweepBand (uint32_t nextF, uint32_t stp, int delta, uint32_t dutyC);
//void generatePwm (uint32_t nextF, uint32_t dutyC);
//uint32_t getDutyCyle (void);
extern void stopPwm (void);
void delay (uint32_t msWait);
void ZG_INT_PRI_Set(void);
void Hex2Decimal(uint8_t num);
void convertHexToSegment(void); /*opData dizisine 7seg verilerini yazar*/

extern uint32_t seriPortaYaz (uint8_t *ptrDizi, uint8_t rakamSayisi);
extern void outDisplayData ( uint8_t rakamHane);
void MENU_Function(void); /* MENU butonu iþlemleri*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Global variables */
 uint32_t temp1=0,intSource = 99;
 uint32_t *ptr;
 uint8_t  workingSet[4]; /* Cihaz çalýþýrken geçici olarak param. tutar */
 uint8_t  sonucRakam[10];/*Display edilecek HEX rakamlarýn BCD karþýlýðý*/
 uint32_t sysTickBasla; /* Zaman hesaplama için kullanýlabilir. SysTick 1 ms*/

/* Global static variables. They must be in non-volatile memory. ZG           */
static uint8_t  opData[4][26];  /* Cihazýn çalýþma parametrelerini tutar      */


static uint8_t  ledSegment [33]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,
                          0x7F,0XFF,0x88,0xC6,0xA1,0x83,0x84,0x86,0x8E,0x89,
                          0x8B,0xF9,0xF1,0xC7,0xAB,0xA3,0x8C,0xA1,0xE3,0x91,
                          0x98,0xBF,0xF7};

static uint8_t opData[4][26] = {
     {10,12,14,16,18,20,22,24,0x86,0x8B,0xBF,0x8B,0x86,0xBF,0x86,0x8B,0xBF,0x8B,
      0x86,0xBF,0x86,0x8B,0xBF,0x8B,0x86,0xBF},   /*  row indexed by 0 */
     {30,31,32, 33,20,95,1,0x86,0x8B,0xBF},        /*  row indexed by 1 */
     {50,50,60, 65,20,85,5,0x86,0x8B,0x86},         /*  row indexed by 2 */
     {1,  2, 3, 4,1,99,1,125,0x86,0x8B}              /*  row indexed by 3 */    
};
/* USER CODE END 0 */



int main(void)
{

  /* USER CODE BEGIN 1 */
  int32_t  tara=0;
  uint32_t temp=0;
  
  uint8_t kacHane=6, *ptrDisplayBuffer;
  
  uint32_t *ptrToRegGenel, *pR, *ptr;
  uint8_t  *ptrChar, rakamBcd, temp8;
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
  ZG_INT_PRI_Set(); /* Interrupt seviyelerini ayarla*/
 
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
    
  
  /** deneme alaný ??????????????????????????????????????????????????????????*/
  sysTickBasla = HAL_GetTick();
//  SysTick_Handler(); /*bakalým nasýl çalýþýyor*/
  temp8 = HAL_GetTick() - sysTickBasla;
//  opData[1][1] = temp8;
  
  /* Hex sayýlarýn Ondalýk karþýlýðýný Hex2Decimal() fonksiyonu               */
  /* sonucRakam[] dizisinin 0 ve 1. hücresine yazar.                          */
  /* Bu sayýlarýn 7-segment karþýlýðý ise ledSegment[] dizisinden alýnarak    */
  /* opData[][] dizisinin 10. sütundan sonrasýna bu fonksiyon yazar.          */
  
  convertHexToSegment(); 
 

  /*testingen*/  
  generatePwm (PERIOD_VALUE, PULSE1_VALUE);

  /* Infinite loop */
 /* USER CODE BEGIN WHILE */ 
  opData [0][10] = 0xFF;
  opData [0][11] = 0xFF;
  opData [0][12] = 0xFF;
  opData [0][13] = 0xFF;
  opData [0][14] = 0xFF;
  opData [0][15] = 0xFF;
  ptrDisplayBuffer = &opData [0][10];
  temp = seriPortaYaz (ptrDisplayBuffer, 6);
  BSP_LED_Toggle(LED6);
  delay(1000);
  
  convertHexToSegment();
  kacHane=6;
  ptrDisplayBuffer = &opData [0][10];
  temp = seriPortaYaz (ptrDisplayBuffer, kacHane); /*kacHane: gösterilecek 7-seg sayýsý*/
  if (temp = 11)
  {  
   BSP_LED_Toggle(LED6);
   delay(500);
  }
  else 
  {
   BSP_LED_On(LED3); /* HATA!!*/
  }
  
 while (1)
 { 
 
  
/** deneme alaný ??????????????????????????????????????????????????????????*/


 }
  /* USER CODE END 2 */
 }

  /* USER CODE END WHILE */



  /* USER CODE BEGIN 3 */



/**ZG
   * @brief Display edilecek verileri içeren opData[][] dizisine 7-Seg led dönüþümünü
   * yapan fonksiyon
   * @param file: yok
   * @param line: -
   * @retval None : 7-seg verisine dönüþüm opData içinde yapýlacaðý için 
   */
void convertHexToSegment(void)
{
  /* opData[4][25] boyutundaki dizide yer alan ve 7-SEG led displaya gönderilecek*/
  /* verileri ilgili hücrelere yazar. Bu fonksiyon çaðýrýldýðýnda                */
  /* opData dizisinin içeriði yüklenmiþ olduðu varsayýlacak ve 100 deðerinden    */
  /* küçük olan sayýlar dönüþüme tabi tututalacaktýr.                            */
  /* Kapsam: opData dizisinin 1., 2. ve 3. satýrlarý ve 0. ile 7. sütunlarý      */

  uint8_t satir  = 3, sutun  = 8;
  uint8_t rakkam = 0, indis  = 0;
  uint8_t *ptrDisplayBuffer, kacHane=2;/* Seri porta gönderilecek buffer adresini taþýr*/

   while (satir > 0)
   {
    while (sutun > 0)
    {
      rakkam = opData[satir][sutun-1];
      if (rakkam < 100)
      {
       Hex2Decimal(rakkam); /* Hex sayýyý Ondalýka çevir ve sonucRakam dizisine yaz*/
       indis = opData[0][sutun-1]; /* 7-SEG bilgisinin yazýlacaðý hücre adresi     */
       opData [satir][indis]   = ledSegment[sonucRakam [1]]; /*7-SEG 10lar hanesi */
       opData [satir][indis+1] = ledSegment[sonucRakam [0]]; /*7-SEG 1ler hanesi  */
      }
      else
      {
        opData[indis][sutun-1]   = 0xFF; /* Boþluk (Blank)*/
        opData[indis+1][sutun-1] = 0xFF; 
      }
      sutun--;
     }
     satir--;
     sutun = 8;
   }
  /* Tablo dolduruldu. Þimdi display alanýna ilgili segment deðerlerini taþý  */
  /* Bu bölüm cihazda display edilecek veriler ve diziliþlerine göre yazýlýr  */
  /* 020816 durumu:: temp10-temp1 -- uP10-uP1 -- tim10-tim1 */
   ptrDisplayBuffer = &opData [0][15];
   for (satir=1; satir < 4; satir++)
   {
    *ptrDisplayBuffer = opData [satir][14];
    ptrDisplayBuffer--;
    *ptrDisplayBuffer = opData [satir][15];
    ptrDisplayBuffer--;
   }

}


/* 8-bit HEX sayýyý 8-bit BCD sayýya dönüþtüren fonksiyon*/
/* LSD sonucRakam[0]. elemanýnda */
/* MSD sonucRakam[1]. elemanýnda */
void Hex2Decimal(uint8_t num)
{
  uint8_t i = 0;
  /*Test if num < 10*/
  if (num < 10)
  {
   sonucRakam[1] = 0;
  }
  while(num > 0)
  {
   sonucRakam[i] = num % 10;
   num /= 10;
   i = i + 1;
  }
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

void MENU_Fonksiyonu (void)
{
  
}

void UP_Fonksiyonu (void)
{
  
}

void DOWN_Fonksiyonu (void)
{
  
}

void DEGAS_Fonksiyonu (void)
{
  
}

void PULSE_Fonksiyonu (void)
{
  
}


void ZG_INT_PRI_Set(void)
{
  uint32_t *ptr;
 /* Disable interrupts and DMAs for channel 1,2,3 and 4 */
  ptr = &TIM1_DIER;
  *ptr &= (0x00000000);
  
  NVIC_SetPriority(IRQn01, INT_PRIORITY_3);
  NVIC_SetPriority(IRQn23, INT_PRIORITY_0);
  NVIC_SetPriority(IRQn415, INT_PRIORITY_2);
}



 void delay (uint32_t msWait)
 {
  /*msWait ms bekleyen fonksiyon*/
  // BSP_LED_On(LED5); /*LED5 yak*/
   HAL_Delay(msWait);
  //BSP_LED_Off(LED5); /*LED5 söndür*/
}
/* USER CODE END 4 */




/**
  * @brief  This function is executed in case of error occurrence.*/
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
