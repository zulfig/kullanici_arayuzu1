/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32f072b_discovery.h"
#include "registersEXTI.h"     /* EXTI  Register adresleri*/
#include "pendingBitsEXTI.h"   /* EXTIn lines için pending bitleri:PendingBitn*/
    
#define  IRQn01              5  /* EXTI0_1 line için IRQ numarasý*/
#define  IRQn23              6  /* EXTI2_3 line için IRQ numarasý*/
#define  IRQn415             7  /* EXTI4_15 line için IRQ numarasý*/
    
#define  INT_PRIORITY_0      0x00  /* Interrupt priority : 0   */
#define  INT_PRIORITY_1      0x40  /* Interrupt priority : 64  */
#define  INT_PRIORITY_2      0x80  /* Interrupt priority : 128 */
#define  INT_PRIORITY_3      0xC0  /* Interrupt priority : 192 */
    
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 0 and 1 interrupts.
EXTI0 : PA0 pin
EXTI1 : PA1 pin
/*=============================================================================*/
void EXTI0_1_IRQHandler(void)
{
  uint32_t intLine ;
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
  /* Önce interruptlarý disable et ************************/
  NVIC_DisableIRQ(IRQn01);
  NVIC_DisableIRQ(IRQn23);
  NVIC_DisableIRQ(IRQn415);
  (EXTI_IMR)&= (0xFFFFFFC0);   /* Mask EXTI0-5 interrupts (disable)*******/
 
 /* Hangi tuþa basýldýðýnýn tespit edilmesi. EXTI_PR registerýndan oku */
  intLine = (EXTI_PR) &(PendingBit0); /* EXTI0 biti al = EXTI0_1_IRQn 0*/
  if (intLine) 
  {/* IRQ 0 set; yani EXTI0'dan int. gelmiþ = RESERVED tuþuna basýlmýþ *********/
    BSP_LED_Toggle(LED6);
//    RESERVED_Fonksiyonu (); /* Þimdilik bu tuþu kullanmayacaðýz*/
  } 
  else
  { 
    intLine = (EXTI_PR) &(PendingBit1); /* EXTI1 biti al = EXTI0_1_IRQn 1*/
     if (intLine)
     { /* IRQ 1 set; yani EXTI1'den int. gelmiþ = UP tuþuna basýlmýþ *********/
       BSP_LED_Toggle(LED5);
//       UP_Fonksiyonu ();
     }
  }
  /* Çýkmadan önce interruptlarý tekrar enable et; pending bitleri clear*****/
  NVIC_EnableIRQ(IRQn01);
  NVIC_EnableIRQ(IRQn23);
  NVIC_EnableIRQ(IRQn415);
  NVIC_ClearPendingIRQ(IRQn01); /* NVIC ICPR reg. clear bit IRQ0_1   */
  (EXTI_IMR) |= (0x0000003F);   /* Unmask EXTI0-5 interrupts (enable)*******/
  (EXTI_PR)  |= (0x000000FF);    /* Clear Pending bit in EXTI_PR register */
  
  /* IRQn 1 set edip deneme yapalým */
//  (EXTI_SWIER) |= (0x00000008); 
//  (EXTI_SWIER) |= (0x00000010);
  
  /* USER CODE END EXTI0_1_IRQn 0 */
}



/*=============================================================================*/
/* @brief This function handles EXTI line 2 and 3 interrupts.
EXTI2 : PA2 pin
EXTI3 : PA3 pin
*=============================================================================*/
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */
   uint32_t intLine ;
  /* Önce interruptlarý disable et ************************/
  NVIC_DisableIRQ(IRQn01);
  NVIC_DisableIRQ(IRQn23);
  NVIC_DisableIRQ(IRQn415);
  (EXTI_IMR)&= (0xFFFFFFC0);   /* Mask EXTI0-5 interrupts (disable)*******/
 
 /* Hangi tuþa basýldýðýnýn tespit edilmesi. EXTI_PR registerýndan oku */
  intLine = (EXTI_PR) &(PendingBit2); /* EXTI2 biti al = EXTI0_1_IRQn 2 */
  if (intLine) 
  {/* IRQ 2 set; yani EXTI2'den int. gelmiþ = DOWN tuþuna basýlmýþ *********/
    BSP_LED_Toggle(LED4);
//  DOWN_Fonksiyonu ();
  } 
  else
  { 
    intLine = (EXTI_PR) &(PendingBit3); /* EXTI3 biti al = EXTI0_1_IRQn 3 */
     if (intLine)
     { /* IRQ 3 set; yani EXTI3'den int. gelmiþ = MENU tuþuna basýlmýþ *********/
       BSP_LED_Toggle(LED5);
//       MENU_Fonksiyonu ();
     }
  }
  /* Çýkmadan önce interruptlarý tekrar enable et; pending bitleri clear*****/
  NVIC_EnableIRQ(IRQn01);
  NVIC_EnableIRQ(IRQn23);
  NVIC_EnableIRQ(IRQn415);
  NVIC_ClearPendingIRQ(IRQn01); /* NVIC ICPR reg. clear bit IRQ0_1   */
  (EXTI_IMR) |= (0x0000003F);   /* Unmask EXTI0-5 interrupts (enable)*******/
  (EXTI_PR)  |= (0x000000FF);    /* Clear Pending bit in EXTI_PR register */
  
  /* USER CODE END EXTI2_3_IRQn 0 */
}



/*============================================================================*/
/* @brief This function handles EXTI line 4 to 15 interrupts.
EXTI4 : PA4 pin
EXTI5 : PA5 pin
*=============================================================================*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  uint32_t intLine ;
  /* Önce interruptlarý disable et ************************/
  NVIC_DisableIRQ(IRQn01);
  NVIC_DisableIRQ(IRQn23);
  NVIC_DisableIRQ(IRQn415);
  (EXTI_IMR)&= (0xFFFFFFC0);   /* Mask EXTI0-5 interrupts (disable)*******/
 
 /* Hangi tuþa basýldýðýnýn tespit edilmesi. EXTI_PR registerýndan oku */
  intLine = (EXTI_PR) &(PendingBit4); /* EXTI4 biti al = EXTI0_1_IRQn 4 */
  if (intLine) 
  {/* IRQ 4 set; yani EXTI4'den int. gelmiþ = DEGAS tuþuna basýlmýþ *********/
    BSP_LED_Toggle(LED4);
//  DEGAS_Fonksiyonu ();
  } 
  else
  { 
    intLine = (EXTI_PR) &(PendingBit5); /* EXTI5 biti al = EXTI0_1_IRQn 5 */
     if (intLine)
     { /* IRQ 5 set; yani EXTI5'den int. gelmiþ = PULSE tuþuna basýlmýþ *********/
       BSP_LED_Toggle(LED5);
//       PULSE_Fonksiyonu ();
     }
  }
  /* Çýkmadan önce interruptlarý tekrar enable et; pending bitleri clear*****/
  NVIC_EnableIRQ(IRQn01);
  NVIC_EnableIRQ(IRQn23);
  NVIC_EnableIRQ(IRQn415);
  NVIC_ClearPendingIRQ(IRQn01); /* NVIC ICPR reg. clear bit IRQ0_1   */
  (EXTI_IMR) |= (0x0000003F);   /* Unmask EXTI0-5 interrupts (enable)*******/
  (EXTI_PR)  |= (0x000000FF);    /* Clear Pending bit in EXTI_PR register */
  
  /* USER CODE END EXTI4_15_IRQn 0 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
